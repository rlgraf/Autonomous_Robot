#!/usr/bin/env python3
###############################################################################
# Soft Obstacle Avoidance Node
#
# Works alongside auto_recharge_node. Acts as a safety interrupt layer:
#
#   IDLE     – /recharge_active is False → silent, does nothing.
#   WATCHING – /recharge_active is True  → monitors lidar.
#   AVOIDING – Obstacle within trigger_distance → takes over /cmd_vel,
#              publishes /navigation_paused = True so navigator yields,
#              rotates away from obstacle, then nudges forward.
#   NUDGING  – Short forward drive after rotating clear, then hands back.
#
# All tuning parameters live in avoidance_parameters.yaml.
#
# Topics
# ──────
#   Sub  /scan               sensor_msgs/LaserScan
#   Sub  /recharge_active    std_msgs/Bool   (from auto_recharge_node)
#   Pub  /cmd_vel            geometry_msgs/Twist
#   Pub  /navigation_paused  std_msgs/Bool   (True = avoidance owns cmd_vel)
###############################################################################

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


IDLE     = 'IDLE'
WATCHING = 'WATCHING'
AVOIDING = 'AVOIDING'
NUDGING  = 'NUDGING'


class SoftObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('soft_obstacle_avoidance_node')

        # ── Declare parameters (values come from avoidance_parameters.yaml) ───
        self.declare_parameter('trigger_distance',   0.70)
        self.declare_parameter('clear_distance',     0.95)
        self.declare_parameter('forward_arc_deg',   70.0)
        self.declare_parameter('avoid_angular_spd',  0.45)
        self.declare_parameter('avoid_linear_spd',   0.15)
        self.declare_parameter('avoid_linear_dist',  0.40)
        self.declare_parameter('min_range',          0.10)
        self.declare_parameter('max_range',          8.0)
        self.declare_parameter('control_hz',        20.0)

        self._trigger_dist    = self.get_parameter('trigger_distance').value
        self._clear_dist      = self.get_parameter('clear_distance').value
        self._forward_arc     = math.radians(self.get_parameter('forward_arc_deg').value)
        self._avoid_ang_spd   = self.get_parameter('avoid_angular_spd').value
        self._avoid_lin_spd   = self.get_parameter('avoid_linear_spd').value
        self._avoid_lin_dist  = self.get_parameter('avoid_linear_dist').value
        self._min_range       = self.get_parameter('min_range').value
        self._max_range       = self.get_parameter('max_range').value
        self._control_hz      = self.get_parameter('control_hz').value

        # ── State ─────────────────────────────────────────────────────────────
        self._state         = IDLE
        self._scan_points   = []       # [(angle_rad, range_m), ...]
        self._scan_received = False
        self._nav_active    = False
        self._nudge_steps   = 0

        # ── QoS for lidar (sensor best-effort) ────────────────────────────────
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',            self._scan_cb,   sensor_qos)
        self.create_subscription(Bool,      '/recharge_active', self._active_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist, '/cmd_vel',            10)
        self._paused_pub = self.create_publisher(Bool,  '/navigation_paused',  10)

        self.create_timer(1.0 / self._control_hz, self._safety_loop)

        self.get_logger().info(
            f'SoftObstacleAvoidanceNode ready. '
            f'trigger={self._trigger_dist}m  clear={self._clear_dist}m  '
            f'arc=±{math.degrees(self._forward_arc):.0f}°'
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

    # ── Safety loop ───────────────────────────────────────────────────────────

    def _safety_loop(self):
        if not self._nav_active:
            if self._state != IDLE:
                self._exit_avoidance()
            return

        if not self._scan_received:
            return

        nearest_range, nearest_bearing = self._nearest_in_arc()

        # ── WATCHING: monitor for obstacles ───────────────────────────────────
        if self._state in (IDLE, WATCHING):
            self._state = WATCHING
            if nearest_range < self._trigger_dist:
                self.get_logger().warn(
                    f'Obstacle at {nearest_range:.2f} m '
                    f'(bearing {math.degrees(nearest_bearing):.1f}°) — '
                    f'interrupting navigation.'
                )
                self._state = AVOIDING
                self._publish_paused(True)
                self._stop()

        # ── AVOIDING: rotate away from nearest obstacle ────────────────────────
        elif self._state == AVOIDING:
            if nearest_range >= self._clear_dist:
                steps = int((self._avoid_lin_dist / self._avoid_lin_spd) * self._control_hz)
                self._nudge_steps = steps
                self._state = NUDGING
                self.get_logger().info('Obstacle cleared — nudging forward.')
            else:
                twist = Twist()
                # Turn away: if obstacle is on the left (+bearing), turn right (-)
                twist.angular.z = (
                    -self._avoid_ang_spd if nearest_bearing >= 0 else self._avoid_ang_spd
                )
                self._cmd_pub.publish(twist)
                self.get_logger().warn(
                    f'Avoiding: {nearest_range:.2f} m at '
                    f'{math.degrees(nearest_bearing):.1f}° — rotating.',
                    throttle_duration_sec=0.5,
                )

        # ── NUDGING: short forward drive before handing back to navigator ──────
        elif self._state == NUDGING:
            if self._nudge_steps > 0:
                twist = Twist()
                twist.linear.x = self._avoid_lin_spd
                self._cmd_pub.publish(twist)
                self._nudge_steps -= 1
            else:
                self.get_logger().info('Nudge complete — resuming navigation.')
                self._state = WATCHING
                self._stop()
                self._publish_paused(False)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _nearest_in_arc(self):
        nearest_r = float('inf')
        nearest_b = 0.0
        for angle, r in self._scan_points:
            if abs(angle) <= self._forward_arc and r < nearest_r:
                nearest_r = r
                nearest_b = angle
        return nearest_r, nearest_b

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_paused(self, paused: bool):
        self._paused_pub.publish(Bool(data=paused))

    def _exit_avoidance(self):
        self._state = IDLE
        self._nudge_steps = 0
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