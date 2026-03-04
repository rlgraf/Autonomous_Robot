#!/usr/bin/env python3
###############################################################################
# Soft Obstacle Avoidance Node
#
# Works alongside auto_recharge_node as a safety interrupt:
#
#   IDLE     – /recharge_active False → silent.
#   WATCHING – /recharge_active True  → monitors lidar each tick.
#   AVOIDING – Obstacle within trigger_distance → takes over /cmd_vel,
#              publishes /navigation_paused = True, rotates toward clearer
#              corridor, then nudges forward.
#   NUDGING  – Short forward drive after clearing, then hands back.
#
# Two suppression conditions skip avoidance entirely:
#
#   1. Flanking passthrough
#      Obstacles on both sides but the gap is wide enough for the robot
#      (robot_width + 2*wiggle_room) and dead-ahead is clear → drive through.
#
#   2. Station-proximity suppression
#      Robot is within station_nearness of its target station AND the
#      detected obstacle is farther than the station → the obstacle is a
#      wall/corner beside the station, not blocking the docking path.
#      Avoidance is suppressed so the robot can complete docking.
#
# Topics
# ──────
#   Sub  /scan                 sensor_msgs/LaserScan
#   Sub  /recharge_active      std_msgs/Bool    (from auto_recharge_node)
#   Sub  /target_station_dist  std_msgs/Float32 (from auto_recharge_node)
#   Pub  /cmd_vel              geometry_msgs/Twist
#   Pub  /navigation_paused    std_msgs/Bool
###############################################################################

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


IDLE     = 'IDLE'
WATCHING = 'WATCHING'
AVOIDING = 'AVOIDING'
NUDGING  = 'NUDGING'


class SoftObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('soft_obstacle_avoidance_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('trigger_distance',   0.70)
        self.declare_parameter('clear_distance',     0.95)
        self.declare_parameter('forward_arc_deg',   70.0)
        self.declare_parameter('avoid_angular_spd',  0.45)
        self.declare_parameter('avoid_linear_spd',   0.15)
        self.declare_parameter('avoid_linear_dist',  0.40)
        self.declare_parameter('min_range',          0.10)
        self.declare_parameter('max_range',          8.0)
        self.declare_parameter('control_hz',        20.0)
        self.declare_parameter('robot_width',        0.35)
        self.declare_parameter('wiggle_room',        0.10)
        self.declare_parameter('station_nearness',   1.20)

        self._trigger_dist    = self.get_parameter('trigger_distance').value
        self._clear_dist      = self.get_parameter('clear_distance').value
        self._forward_arc     = math.radians(self.get_parameter('forward_arc_deg').value)
        self._avoid_ang_spd   = self.get_parameter('avoid_angular_spd').value
        self._avoid_lin_spd   = self.get_parameter('avoid_linear_spd').value
        self._avoid_lin_dist  = self.get_parameter('avoid_linear_dist').value
        self._min_range       = self.get_parameter('min_range').value
        self._max_range       = self.get_parameter('max_range').value
        self._control_hz      = self.get_parameter('control_hz').value
        self._robot_width     = self.get_parameter('robot_width').value
        self._wiggle_room     = self.get_parameter('wiggle_room').value
        self._station_nearness = self.get_parameter('station_nearness').value

        # Minimum per-side clearance for flanking passthrough
        self._min_side_clearance = (self._robot_width / 2.0) + self._wiggle_room

        # ── State ─────────────────────────────────────────────────────────────
        self._state               = IDLE
        self._scan_points         = []       # [(angle_rad, range_m), ...]
        self._scan_received       = False
        self._nav_active          = False
        self._nudge_steps         = 0
        self._target_station_dist = float('inf')   # from /target_station_dist

        # Stuck / symmetry-deadlock detector
        self._avoiding_ticks      = 0
        self._stuck_threshold     = int(self._control_hz * 3.0)   # 3 s
        self._committed_direction = 0.0    # +1 = CCW (left), -1 = CW (right)
        self._commit_ticks        = 0
        self._commit_duration     = int(self._control_hz * 2.5)   # 2.5 s

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_cb,   sensor_qos)
        self.create_subscription(Bool,      '/recharge_active',
                                 self._active_cb, 10)
        self.create_subscription(Float32,   '/target_station_dist',
                                 self._station_dist_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist, '/cmd_vel',           10)
        self._paused_pub = self.create_publisher(Bool,  '/navigation_paused', 10)

        self.create_timer(1.0 / self._control_hz, self._safety_loop)

        self.get_logger().info(
            f'SoftObstacleAvoidanceNode ready.  '
            f'trigger={self._trigger_dist}m  clear={self._clear_dist}m  '
            f'arc=±{math.degrees(self._forward_arc):.0f}°  '
            f'min_side_clearance={self._min_side_clearance:.2f}m  '
            f'station_nearness={self._station_nearness}m'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        pts   = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and self._min_range < r < self._max_range:
                pts.append((angle, r))
            angle += msg.angle_increment
        self._scan_points   = pts
        self._scan_received = True

    def _active_cb(self, msg: Bool):
        self._nav_active = msg.data
        if not self._nav_active and self._state in (AVOIDING, NUDGING):
            self.get_logger().info('Navigator stopped — exiting avoidance.')
            self._exit_avoidance()

    def _station_dist_cb(self, msg: Float32):
        self._target_station_dist = msg.data

    # ── Safety loop ───────────────────────────────────────────────────────────

    def _safety_loop(self):
        if not self._nav_active:
            if self._state != IDLE:
                self._exit_avoidance()
            return

        if not self._scan_received:
            return

        nearest_range, nearest_bearing = self._nearest_in_arc()

        # ── WATCHING ──────────────────────────────────────────────────────────
        if self._state in (IDLE, WATCHING):
            self._state = WATCHING

            if nearest_range < self._trigger_dist:

                # Suppression 1: station-proximity
                # The robot is close to the station and the detected obstacle
                # is farther than the station — it's a wall/corner beside the
                # station, not blocking the docking path. Let the robot through.
                if self._suppress_near_station(nearest_range):
                    self.get_logger().info(
                        f'Near-station suppression: obstacle at {nearest_range:.2f}m '
                        f'but station is only {self._target_station_dist:.2f}m away '
                        f'— ignoring obstacle.',
                        throttle_duration_sec=2.0,
                    )
                    return

                # Suppression 2: flanking passthrough
                if self._check_flanking_passthrough():
                    self.get_logger().info(
                        f'Flanking passthrough: obstacle at {nearest_range:.2f}m '
                        f'but corridor is wide enough — driving through.',
                        throttle_duration_sec=2.0,
                    )
                    return

                self.get_logger().warn(
                    f'Obstacle at {nearest_range:.2f}m '
                    f'(bearing {math.degrees(nearest_bearing):.1f}°) — '
                    f'interrupting navigation.'
                )
                self._state             = AVOIDING
                self._avoiding_ticks    = 0
                self._committed_direction = 0.0
                self._commit_ticks      = 0
                self._publish_paused(True)
                self._stop()

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
                # Re-check dead-ahead before each nudge step
                nudge_range, _ = self._nearest_in_arc(arc_override=math.radians(30.0))
                if nudge_range < self._trigger_dist:
                    self.get_logger().warn(
                        'Obstacle appeared during nudge — re-entering AVOIDING.'
                    )
                    self._state          = AVOIDING
                    self._avoiding_ticks = 0
                    self._nudge_steps    = 0
                    self._stop()
                    return
                twist = Twist()
                twist.linear.x = self._avoid_lin_spd
                self._cmd_pub.publish(twist)
                self._nudge_steps -= 1
            else:
                self.get_logger().info('Nudge complete — resuming navigation.')
                self._state = WATCHING
                self._stop()
                self._publish_paused(False)

    # ── Suppression checks ────────────────────────────────────────────────────

    def _suppress_near_station(self, nearest_obstacle_range: float) -> bool:
        """
        Return True (suppress avoidance) when:
          - The robot is within station_nearness of its target station, AND
          - The nearest obstacle is farther than the station itself.
        
        This handles the case where the charging station is in a corner or
        against a wall. As the robot approaches, the wall/corner enters the
        trigger distance. But since the station is closer than the wall, the
        wall is behind or beside the target — not blocking it. We can ignore it.
        """
        robot_near_station = self._target_station_dist <= self._station_nearness
        obstacle_beyond_station = nearest_obstacle_range > self._target_station_dist

        return robot_near_station and obstacle_beyond_station

    def _check_flanking_passthrough(self) -> bool:
        """
        Return True (suppress avoidance) when obstacles flank both sides of
        the forward arc but leave a gap wide enough for the robot to fit:
          - Both left and right sectors have an obstacle within trigger_distance
          - Dead-ahead (±15°) is clear beyond trigger_distance
          - Each side's nearest obstacle is at least min_side_clearance away
        """
        left_min, right_min = self._sector_clearance()
        dead_ahead, _       = self._nearest_in_arc(arc_override=math.radians(15.0))

        # Must have obstacles on both sides (flanking, not one-sided)
        if not (left_min < self._trigger_dist and right_min < self._trigger_dist):
            return False

        # Dead ahead must be open — this is a corridor, not a wall
        if dead_ahead < self._trigger_dist:
            return False

        # Each side must leave room for the robot body + wiggle
        if left_min >= self._min_side_clearance and right_min >= self._min_side_clearance:
            self.get_logger().info(
                f'Flanking: left={left_min:.2f}m  right={right_min:.2f}m  '
                f'need≥{self._min_side_clearance:.2f}m each side.',
                throttle_duration_sec=2.0,
            )
            return True

        return False

    # ── Corridor-based avoidance rotation ────────────────────────────────────

    def _do_avoidance_rotation(self):
        """
        Rotate toward the clearer corridor rather than away from the nearest
        point. Prevents oscillation when two obstacles are symmetric.
        After stuck_threshold ticks with no clearance, commit to one direction
        for commit_duration ticks to break symmetry deadlocks.
        """
        left_clear, right_clear = self._sector_clearance()

        # Stuck / deadlock handling
        if self._avoiding_ticks >= self._stuck_threshold:
            if self._committed_direction == 0.0:
                # First time hitting the threshold — pick a side and lock in
                self._committed_direction = (
                    +1.0 if left_clear > right_clear + 0.05 else -1.0
                )
                self._commit_ticks = self._commit_duration
                self.get_logger().warn(
                    f'Stuck {self._avoiding_ticks} ticks — committing '
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
                # Committed spin ended but still blocked — reset and retry
                self._committed_direction = 0.0
                self._avoiding_ticks      = 0

        # Normal: rotate toward the clearer side
        turn_sign = +1.0 if left_clear >= right_clear else -1.0
        twist = Twist()
        twist.angular.z = self._avoid_ang_spd * turn_sign
        self._cmd_pub.publish(twist)

        self.get_logger().warn(
            f'Avoiding: left={left_clear:.2f}m  right={right_clear:.2f}m  '
            f'→ {"LEFT" if turn_sign > 0 else "RIGHT"}  '
            f'(tick {self._avoiding_ticks}/{self._stuck_threshold})',
            throttle_duration_sec=0.5,
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _nearest_in_arc(self, arc_override: float | None = None):
        """Nearest obstacle range and bearing within the forward arc."""
        arc       = arc_override if arc_override is not None else self._forward_arc
        nearest_r = float('inf')
        nearest_b = 0.0
        for angle, r in self._scan_points:
            if abs(angle) <= arc and r < nearest_r:
                nearest_r = r
                nearest_b = angle
        return nearest_r, nearest_b

    def _sector_clearance(self):
        """
        Minimum range in the left half vs. right half of the forward arc.
        Left = positive bearing (CCW), right = negative bearing (CW).
        Uses the minimum so a single close point correctly marks a side blocked.
        """
        left_min  = float('inf')
        right_min = float('inf')
        for angle, r in self._scan_points:
            if abs(angle) <= self._forward_arc:
                if angle >= 0.0:
                    left_min  = min(left_min,  r)
                else:
                    right_min = min(right_min, r)
        if left_min  == float('inf'): left_min  = self._max_range
        if right_min == float('inf'): right_min = self._max_range
        return left_min, right_min

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_paused(self, paused: bool):
        self._paused_pub.publish(Bool(data=paused))

    def _exit_avoidance(self):
        self._state               = IDLE
        self._nudge_steps         = 0
        self._avoiding_ticks      = 0
        self._committed_direction = 0.0
        self._commit_ticks        = 0
        self._stop()
        self._publish_paused(False)


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