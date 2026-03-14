#!/usr/bin/env python3
"""
Node: avoid_while_interact.py (ROS2) — supervisor-safe version

Monitors the lidar continuously and acts as a safety interrupt layer over
object_navigator.py.

Behaviour
─────────
  IDLE      – No obstacle within TRIGGER_DISTANCE.
              Publishes /navigation_paused = False.
              The navigator drives normally via /cmd_vel_nav.

  AVOIDING  – An obstacle has entered TRIGGER_DISTANCE.
              Publishes /navigation_paused = True  (navigator goes silent).
              Publishes /avoid_active = True.
              Takes control via /cmd_vel_avoid:
                • Immediately stops the robot.
                • Rotates away from the nearest obstacle.
              Holds this state until the obstacle is beyond CLEAR_DISTANCE
              (hysteresis gap prevents flickering at the boundary).
              Then nudges forward and publishes /navigation_paused = False.

Topic layout
────────────
  This node   →  /cmd_vel_avoid        (geometry_msgs/Twist)   [candidate for supervisor]
  This node   →  /navigation_paused    (std_msgs/Bool)         [pause flag]
  This node   →  /avoid_active         (std_msgs/Bool)         [active flag]
  Navigator   →  /cmd_vel_nav          (geometry_msgs/Twist)   [only when not paused]
  Lidar       →  /scan                 (sensor_msgs/LaserScan)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ── Tunable parameters ────────────────────────────────────────────────────────
TRIGGER_DISTANCE  = 0.6    # m  – obstacle closer than this → interrupt navigation
CLEAR_DISTANCE    = 0.8    # m  – obstacle must reach this distance before resuming
AVOID_ANGULAR_SPD = 0.45   # rad/s – rotation speed while avoiding
MIN_RANGE         = 0.10   # m  – ignore sensor self-noise
MAX_RANGE         = 8.0    # m  – ignore far noise
FORWARD_ARC       = 1.047  # +/- 60 degrees - forward half only
AVOID_LINEAR_DIST = 0.8    # m - drive forward after rotating clear
AVOID_LINEAR_SPEED = 0.2   # m/s
# ─────────────────────────────────────────────────────────────────────────────

IDLE    = 'IDLE'
AVOIDING = 'AVOIDING'


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.scan_points: list[tuple[float, float]] = []  # (angle, range)
        self.scan_received = False
        self.state = IDLE
        self.forward_steps = 0

        self.recharge_active = False

        self._paused_latched = None
        self._active_latched = None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.create_subscription(Bool, '/recharge_active', self._cb_recharge_active, 10)

        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel_avoid', 10)
        self.paused_pub = self.create_publisher(Bool,  '/navigation_paused', 10)
        self.active_pub = self.create_publisher(Bool,  '/avoid_active', 10)

        self.create_timer(0.05, self._safety_loop)  # 20 Hz

        self.get_logger().info('Obstacle avoidance node running.')
        self.get_logger().info(
            f'  Trigger : {TRIGGER_DISTANCE:.2f} m  |  '
            f'Clear   : {CLEAR_DISTANCE:.2f} m'
        )

    # ── Scan callback ──────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.scan_points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE and math.isfinite(r):
                self.scan_points.append((angle, r))
            angle += msg.angle_increment
        self.scan_received = True

    def _cb_recharge_active(self, msg: Bool):
        new_val = bool(msg.data)
        if new_val != self.recharge_active:
            self.recharge_active = new_val
            if self.recharge_active:
                self.get_logger().info("Recharge active — disabling interaction avoidance.")
                self._force_idle()

    # ── Safety loop ────────────────────────────────────────────────────────────

    def _safety_loop(self):
        if not self.scan_received:
            return

        # This node must not compete during recharge
        if self.recharge_active:
            if self.state != IDLE or self._active_latched or self._paused_latched:
                self._force_idle()
            return

        nearest_range, nearest_bearing = self._nearest_obstacle()

        active = (self.state != IDLE)
        self._publish_paused(active)
        self._publish_active(active)

        if self.state == IDLE:
            if nearest_range < TRIGGER_DISTANCE:
                self.get_logger().warn(
                    f'Obstacle at {nearest_range:.2f} m — interrupting navigation.'
                )
                self.state = AVOIDING
                self._publish_paused(True)
                self._publish_active(True)
                self._stop()

        elif self.state == AVOIDING:
            if self.forward_steps > 0:
                # Phase 2: drive forward after rotating clear
                twist = Twist()
                twist.linear.x = AVOID_LINEAR_SPEED
                self.cmd_pub.publish(twist)
                self.forward_steps -= 1
                if self.forward_steps == 0:
                    self.get_logger().info('Forward nudge complete — resuming navigation.')
                    self.state = IDLE
                    self._stop()
                    self._publish_paused(False)
                    self._publish_active(False)

            elif nearest_range >= CLEAR_DISTANCE:
                # Phase 1 complete: obstacle is clear, now nudge forward
                # Calculate steps: distance / speed / dt
                dt = 0.05  # 20 Hz
                self.forward_steps = int((AVOID_LINEAR_DIST / AVOID_LINEAR_SPEED) / dt)
                self.get_logger().info('Obstacle cleared — nudging forward.')

            else:
                # Phase 1: rotate away from obstacle
                twist = Twist()
                twist.angular.z = AVOID_ANGULAR_SPD * (-1.0 if nearest_bearing >= 0 else 1.0)
                self.cmd_pub.publish(twist)
                self.get_logger().warn(
                    f'Avoiding: {nearest_range:.2f} m at '
                    f'{math.degrees(nearest_bearing):.1f}° — rotating.',
                    throttle_duration_sec=0.5,
                )
                

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _nearest_obstacle(self) -> tuple[float, float]:
        """Return (range, bearing) of the nearest lidar point."""
        nearest_range   = float('inf')
        nearest_bearing = 0.0
        for angle, r in self.scan_points:
            if abs(angle) > FORWARD_ARC:
                continue
            if r < nearest_range:
                nearest_range   = r
                nearest_bearing = angle
        return nearest_range, nearest_bearing

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_paused(self, paused: bool):
        if self._paused_latched == paused:
            return
        self._paused_latched = paused
        msg = Bool()
        msg.data = paused
        self.paused_pub.publish(msg)

    def _publish_active(self, active: bool):
        if self._active_latched == active:
            return
        self._active_latched = active
        msg = Bool()
        msg.data = active
        self.active_pub.publish(msg)

    def _force_idle(self):
        self._stop()
        self.state = IDLE
        self.forward_steps = 0
        self._publish_paused(False)
        self._publish_active(False)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
