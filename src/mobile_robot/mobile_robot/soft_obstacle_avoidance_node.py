#!/usr/bin/env python3
###############################################################################
# Soft Obstacle Avoidance Node
#
# Sole owner of /cmd_vel during recharge. Subscribes to /recharge/cmd_vel
# from auto_recharge_node and either forwards it unchanged (no obstacle) or
# overrides it with avoidance commands (obstacle detected). No boolean
# handshake with the recharge node — arbitration is purely local.
#
# States
# ──────
#   IDLE       – /recharge_active False → silent, publishes nothing to /cmd_vel
#   WATCHING   – monitoring lidar; forwards /recharge/cmd_vel each tick
#   SUPPRESSED – near-station latch; forwards /recharge/cmd_vel, ignores obstacles
#                Releases only when target_station_dist resets to inf (arrived/done)
#                NOT on distance comparison — prevents corner flicker
#   AVOIDING   – obstacle within trigger_distance; publishes rotation command
#   NUDGING    – short forward nudge after clearing; then back to WATCHING
#
# Topics
# ──────
#   Sub  /scan                 sensor_msgs/LaserScan
#   Sub  /recharge/cmd_vel     geometry_msgs/Twist   (from auto_recharge_node)
#   Sub  /recharge_active      std_msgs/Bool
#   Sub  /target_station_dist  std_msgs/Float32
#   Pub  /cmd_vel              geometry_msgs/Twist   (sole publisher)
###############################################################################

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


IDLE       = 'IDLE'
WATCHING   = 'WATCHING'
SUPPRESSED = 'SUPPRESSED'
AVOIDING   = 'AVOIDING'
NUDGING    = 'NUDGING'


class SoftObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('soft_obstacle_avoidance_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('trigger_distance',   0.70)
        self.declare_parameter('clear_distance',     0.90)
        self.declare_parameter('forward_arc_deg',    70.0)
        self.declare_parameter('avoid_angular_spd',  0.45)
        self.declare_parameter('avoid_linear_spd',   0.15)
        self.declare_parameter('avoid_linear_dist',  0.40)
        self.declare_parameter('min_range',          0.10)
        self.declare_parameter('max_range',          8.0)
        self.declare_parameter('control_hz',         20.0)
        self.declare_parameter('robot_width',        0.35)
        self.declare_parameter('wiggle_room',        0.10)
        self.declare_parameter('station_nearness',   1.40)

        self._trigger_dist     = self.get_parameter('trigger_distance').value
        self._clear_dist       = self.get_parameter('clear_distance').value
        self._forward_arc      = math.radians(self.get_parameter('forward_arc_deg').value)
        self._avoid_ang_spd    = self.get_parameter('avoid_angular_spd').value
        self._avoid_lin_spd    = self.get_parameter('avoid_linear_spd').value
        self._avoid_lin_dist   = self.get_parameter('avoid_linear_dist').value
        self._min_range        = self.get_parameter('min_range').value
        self._max_range        = self.get_parameter('max_range').value
        self._control_hz       = self.get_parameter('control_hz').value
        self._robot_width      = self.get_parameter('robot_width').value
        self._wiggle_room      = self.get_parameter('wiggle_room').value
        self._station_nearness = self.get_parameter('station_nearness').value

        self._min_side_clearance = (self._robot_width / 2.0) + self._wiggle_room

        # ── State ─────────────────────────────────────────────────────────────
        self._state               = IDLE
        self._scan_points         = []
        self._scan_received       = False
        self._nav_active          = False
        self._recharge_cmd        = Twist()      # latest from /recharge/cmd_vel
        self._nudge_steps         = 0
        self._target_station_dist = float('inf')

        # Stuck / symmetry-deadlock detector
        self._avoiding_ticks      = 0
        self._stuck_threshold     = int(self._control_hz * 3.0)
        self._committed_direction = 0.0
        self._commit_ticks        = 0
        self._commit_duration     = int(self._control_hz * 2.5)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, '/scan',
                                 self._scan_cb,         sensor_qos)
        self.create_subscription(Twist,     '/recharge/cmd_vel',
                                 self._recharge_cmd_cb, 10)
        self.create_subscription(Bool,      '/recharge/recharge_active',
                                 self._active_cb,       10)
        self.create_subscription(Float32,   '/recharge/target_station_dist',
                                 self._station_dist_cb, 10)
        self._battery_pct = 1.0
        self.create_subscription(BatteryState, 'battery_status', self._battery_cb, 10)

        # Sole publisher to /cmd_vel
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(1.0 / self._control_hz, self._safety_loop)

        self.get_logger().info(
            f'SoftObstacleAvoidanceNode ready — sole /cmd_vel owner.\n'
            f'  trigger={self._trigger_dist}m  clear={self._clear_dist}m  '
            f'arc=±{math.degrees(self._forward_arc):.0f}°\n'
            f'  station_nearness={self._station_nearness}m  '
            f'min_side_clearance={self._min_side_clearance:.2f}m'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _battery_cb(self, msg: BatteryState):
        self._battery_pct = msg.percentage

    def _scan_cb(self, msg: LaserScan):
        pts   = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and self._min_range < r < self._max_range:
                pts.append((angle, r))
            angle += msg.angle_increment
        self._scan_points   = pts
        self._scan_received = True

    def _recharge_cmd_cb(self, msg: Twist):
        self._recharge_cmd = msg

    def _active_cb(self, msg: Bool):
        self._nav_active = msg.data
        if not self._nav_active:
            # Recharge finished — reset state, stop publishing to /cmd_vel
            self._state               = IDLE
            self._nudge_steps         = 0
            self._avoiding_ticks      = 0
            self._committed_direction = 0.0
            self._commit_ticks        = 0
            self._recharge_cmd        = Twist()

    def _station_dist_cb(self, msg: Float32):
        prev = self._target_station_dist
        self._target_station_dist = msg.data

        # Release suppression latch when recharge is done (dist resets to inf)
        # NOT on distance comparison — that caused the corner flicker
        if self._state == SUPPRESSED and msg.data == float('inf') and prev != float('inf'):
            self.get_logger().info('Suppression latch released — recharge complete.')
            self._state = IDLE

    # ── Safety loop ───────────────────────────────────────────────────────────

    def _safety_loop(self):
        # Silent when recharge is not active
        if not self._nav_active:
            return
        if self._battery_pct <= 0.0:
            self._cmd_pub.publish(Twist())
            #edit
            return

        if not self._scan_received:
            self._forward_recharge()
            return

        # ── SUPPRESSED ────────────────────────────────────────────────────────
        # Fully suppress avoidance for the entire docking approach.
        # Only releases when recharge reports done (target_station_dist → inf).
        if self._state == SUPPRESSED:
            self._forward_recharge()
            return

        nearest_range, nearest_bearing = self._nearest_in_arc()

        # ── WATCHING ──────────────────────────────────────────────────────────
        if self._state in (IDLE, WATCHING):
            self._state = WATCHING

            if nearest_range < self._trigger_dist:

                # Station-proximity suppression — latch for entire docking approach
                if self._suppress_near_station(nearest_range):
                    self.get_logger().info(
                        f'Near-station suppression LATCHED: '
                        f'obstacle={nearest_range:.2f}m  '
                        f'station={self._target_station_dist:.2f}m  '
                        f'— avoidance disabled until docking complete.'
                    )
                    self._state = SUPPRESSED
                    self._forward_recharge()
                    return

                # Flanking passthrough
                if self._check_flanking_passthrough():
                    self.get_logger().info(
                        'Flanking passthrough — corridor wide enough, driving through.',
                        throttle_duration_sec=2.0,
                    )
                    self._forward_recharge()
                    return

                # Real obstacle — take over cmd_vel
                self.get_logger().warn(
                    f'Obstacle at {nearest_range:.2f}m '
                    f'(bearing {math.degrees(nearest_bearing):.1f}°) — avoiding.'
                )
                self._state           = AVOIDING
                self._avoiding_ticks  = 0
                self._committed_direction = 0.0
                self._commit_ticks    = 0
                self._publish_stop()
            else:
                # Clear — forward recharge command
                self._forward_recharge()

        # ── AVOIDING ──────────────────────────────────────────────────────────
        elif self._state == AVOIDING:
            if nearest_range >= self._clear_dist:
                steps = int(
                    (self._avoid_lin_dist / self._avoid_lin_spd) * self._control_hz
                )
                self._nudge_steps         = steps
                self._avoiding_ticks      = 0
                self._committed_direction = 0.0
                self._state               = NUDGING
                self.get_logger().info('Obstacle cleared — nudging forward.')
            else:
                self._avoiding_ticks += 1
                self._do_avoidance_rotation()

        # ── NUDGING ───────────────────────────────────────────────────────────
        elif self._state == NUDGING:
            if self._nudge_steps > 0:
                nudge_range, _ = self._nearest_in_arc(arc_override=math.radians(30.0))
                if nudge_range < self._trigger_dist:
                    if self._suppress_near_station(nudge_range):
                        self.get_logger().info(
                            'Near-station suppression during nudge — latching.'
                        )
                        self._state       = SUPPRESSED
                        self._nudge_steps = 0
                        self._forward_recharge()
                        return
                    self.get_logger().warn('Obstacle during nudge — back to AVOIDING.')
                    self._state          = AVOIDING
                    self._avoiding_ticks = 0
                    self._nudge_steps    = 0
                    self._publish_stop()
                    return
                twist = Twist()
                twist.linear.x = self._avoid_lin_spd
                self._cmd_pub.publish(twist)
                self._nudge_steps -= 1
            else:
                self.get_logger().info('Nudge complete — resuming navigation.')
                self._state = WATCHING
                # Immediately forward recharge command — no gap, no zero publish
                self._forward_recharge()

    # ── Passthrough ───────────────────────────────────────────────────────────

    def _forward_recharge(self):
        """Publish the latest recharge command to /cmd_vel unchanged."""
        self._cmd_pub.publish(self._recharge_cmd)

    def _publish_stop(self):
        """Publish a single zero twist to halt the robot before avoidance starts."""
        self._cmd_pub.publish(Twist())

    # ── Suppression checks ────────────────────────────────────────────────────

    def _suppress_near_station(self, nearest_obstacle_range: float) -> bool:
        """
        Suppress avoidance when close to station AND the nearest obstacle is not
        actually blocking the docking corridor.

        Logic:
            - If we're not within station_nearness, never suppress.
            - If we are close, check whether the nearest obstacle lies along the
                bearing toward the station (within a narrow cone).
            - If the obstacle IS on the station bearing → real blocker → don't suppress.
            - If the obstacle is off-axis → it's a corner wall → suppress.
        """
        if self._target_station_dist > self._station_nearness:
            return False  # not close enough to station yet

        # Find the bearing to the station from the latest recharge cmd
        # We don't have station coords here, but we can use the angular component
        # of the recharge cmd as a proxy: if recharge says "turn hard", the station
        # is off-axis; if it says "go straight", station is ahead.
        # Better: check if the nearest forward obstacle is within a tight cone
        # centered on dead-ahead (the robot should be roughly aligned by now).

        DOCKING_CONE_RAD = math.radians(25.0)  # tight corridor ±25° from dead-ahead

        # Find the nearest obstacle specifically within the tight docking cone
        blocker_range = float('inf')
        for angle, r in self._scan_points:
            if abs(angle) <= DOCKING_CONE_RAD and r < blocker_range:
                blocker_range = r

        # If something is close AND dead-ahead → real blocker, don't suppress
        if blocker_range < self._trigger_dist:
            return False

        # Obstacle is off-axis (corner wall) → safe to suppress
        return True

    def _check_flanking_passthrough(self) -> bool:
        left_min, right_min = self._sector_clearance()
        dead_ahead, _       = self._nearest_in_arc(arc_override=math.radians(15.0))

        if not (left_min < self._trigger_dist and right_min < self._trigger_dist):
            return False
        if dead_ahead < self._trigger_dist:
            return False
        return (left_min >= self._min_side_clearance and
                right_min >= self._min_side_clearance)

    # ── Avoidance rotation ────────────────────────────────────────────────────

    def _do_avoidance_rotation(self):
        left_clear, right_clear = self._sector_clearance()

        if self._avoiding_ticks >= self._stuck_threshold:
            if self._committed_direction == 0.0:
                self._committed_direction = (
                    +1.0 if left_clear > right_clear + 0.05 else -1.0
                )
                self._commit_ticks = self._commit_duration
                self.get_logger().warn(
                    f'Stuck — committing '
                    f'{"LEFT" if self._committed_direction > 0 else "RIGHT"} '
                    f'for {self._commit_duration} ticks.'
                )
            if self._commit_ticks > 0:
                twist = Twist()
                twist.angular.z = self._avoid_ang_spd * self._committed_direction
                self._cmd_pub.publish(twist)
                self._commit_ticks -= 1
                return
            else:
                self._committed_direction = 0.0
                self._avoiding_ticks      = 0

        turn_sign = +1.0 if left_clear >= right_clear else -1.0
        twist = Twist()
        twist.angular.z = self._avoid_ang_spd * turn_sign
        self._cmd_pub.publish(twist)

        self.get_logger().warn(
            f'Avoiding: left={left_clear:.2f}m  right={right_clear:.2f}m  '
            f'→ {"LEFT" if turn_sign > 0 else "RIGHT"}  '
            f'tick {self._avoiding_ticks}/{self._stuck_threshold}',
            throttle_duration_sec=0.5,
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _nearest_in_arc(self, arc_override: float | None = None):
        arc = arc_override if arc_override is not None else self._forward_arc
        nearest_r, nearest_b = float('inf'), 0.0
        for angle, r in self._scan_points:
            if abs(angle) <= arc and r < nearest_r:
                nearest_r, nearest_b = r, angle
        return nearest_r, nearest_b

    def _sector_clearance(self):
        left_min = right_min = float('inf')
        for angle, r in self._scan_points:
            if abs(angle) <= self._forward_arc:
                if angle >= 0.0:
                    left_min  = min(left_min,  r)
                else:
                    right_min = min(right_min, r)
        if left_min  == float('inf'): left_min  = self._max_range
        if right_min == float('inf'): right_min = self._max_range
        return left_min, right_min


def main(args=None):
    rclpy.init(args=args)
    node = SoftObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()