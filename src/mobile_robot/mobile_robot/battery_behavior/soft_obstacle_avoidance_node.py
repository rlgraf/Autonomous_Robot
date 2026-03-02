#!/usr/bin/env python3
###############################################################################
# Soft Obstacle Avoidance Node (UPDATED FOR SUPERVISOR ARBITER)
#
# CHANGE SUMMARY
#   - NO LONGER publishes to /cmd_vel
#   - Publishes candidate avoidance command to:  /cmd_vel_avoid
#   - Keeps publishing /navigation_paused (Bool) as an interlock flag
#   - Optionally can publish /avoid_active (Bool) if you want later (not required)
#
# WHY:
#   Supervisor must be the only publisher to /cmd_vel. This node becomes a
#   "suggested command" source with higher priority.
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

        self._trigger_dist    = float(self.get_parameter('trigger_distance').value)
        self._clear_dist      = float(self.get_parameter('clear_distance').value)
        self._forward_arc     = math.radians(float(self.get_parameter('forward_arc_deg').value))
        self._avoid_ang_spd   = float(self.get_parameter('avoid_angular_spd').value)
        self._avoid_lin_spd   = float(self.get_parameter('avoid_linear_spd').value)
        self._avoid_lin_dist  = float(self.get_parameter('avoid_linear_dist').value)
        self._min_range       = float(self.get_parameter('min_range').value)
        self._max_range       = float(self.get_parameter('max_range').value)
        self._control_hz      = float(self.get_parameter('control_hz').value)

        # ── State ─────────────────────────────────────────────────────────────
        self._state         = IDLE
        self._scan_points   = []       # [(angle_rad, range_m), ...]
        self._scan_received = False
        self._recharge_active = False
        self._nudge_steps   = 0

        # Latch for paused publishing (avoid spamming)
        self._paused = False

        # ── QoS for lidar (sensor best-effort) ────────────────────────────────
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.create_subscription(Bool, '/recharge_active', self._active_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        # IMPORTANT: publish to candidate topic, supervisor owns /cmd_vel
        self._cmd_pub_avoid = self.create_publisher(Twist, '/cmd_vel_avoid', 10)
        self._paused_pub    = self.create_publisher(Bool,  '/navigation_paused', 10)

        self.create_timer(1.0 / self._control_hz, self._safety_loop)

        self.get_logger().info(
            f'SoftObstacleAvoidanceNode ready. '
            f'trigger={self._trigger_dist}m  clear={self._clear_dist}m  '
            f'arc=±{math.degrees(self._forward_arc):.0f}°'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and (self._min_range < r < self._max_range):
                pts.append((angle, r))
            angle += msg.angle_increment
        self._scan_points = pts
        self._scan_received = True

    def _active_cb(self, msg: Bool):
        self._recharge_active = bool(msg.data)
        if (not self._recharge_active) and self._state in (AVOIDING, NUDGING, WATCHING):
            self.get_logger().info('Recharge inactive — exiting avoidance.')
            self._exit_avoidance()

    # ── Safety loop ───────────────────────────────────────────────────────────

    def _safety_loop(self):
        # Only run avoidance when recharge is active
        if not self._recharge_active:
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
                    f'interrupting recharge navigation.'
                )
                self._state = AVOIDING
                self._publish_paused(True)
                self._publish_stop()  # to /cmd_vel_avoid

        # ── AVOIDING: rotate away from nearest obstacle ───────────────────────
        elif self._state == AVOIDING:
            if nearest_range >= self._clear_dist:
                steps = int((self._avoid_lin_dist / max(self._avoid_lin_spd, 1e-6)) * self._control_hz)
                self._nudge_steps = max(0, steps)
                self._state = NUDGING
                self.get_logger().info('Obstacle cleared — nudging forward.')
            else:
                twist = Twist()
                # Turn away: obstacle left (+bearing) => turn right (-)
                twist.angular.z = (-self._avoid_ang_spd if nearest_bearing >= 0 else self._avoid_ang_spd)
                self._cmd_pub_avoid.publish(twist)
                self.get_logger().warn(
                    f'Avoiding: {nearest_range:.2f} m at {math.degrees(nearest_bearing):.1f}° — rotating.',
                    throttle_duration_sec=0.5,
                )

        # ── NUDGING: short forward drive before handing back ──────────────────
        elif self._state == NUDGING:
            if self._nudge_steps > 0:
                twist = Twist()
                twist.linear.x = self._avoid_lin_spd
                self._cmd_pub_avoid.publish(twist)
                self._nudge_steps -= 1
            else:
                self.get_logger().info('Nudge complete — resuming recharge navigation.')
                self._state = WATCHING
                self._publish_stop()
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

    def _publish_stop(self):
        self._cmd_pub_avoid.publish(Twist())

    def _publish_paused(self, paused: bool):
        # Only publish on changes (reduces noise)
        if paused == self._paused:
            return
        self._paused = paused
        self._paused_pub.publish(Bool(data=paused))

    def _exit_avoidance(self):
        self._state = IDLE
        self._nudge_steps = 0
        self._publish_stop()
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