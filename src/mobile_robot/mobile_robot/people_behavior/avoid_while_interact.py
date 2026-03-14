#!/usr/bin/env python3
"""
Node: avoid_while_interact.py (ROS2) — supervisor-safe version

What changed:
  1) Publishes candidate velocity only on /cmd_vel_avoid
  2) Publishes latched /navigation_paused and /avoid_active
  3) Disables itself completely during recharge mode
  4) Releases control cleanly on reverse timeout instead of rotating forever
  5) Adds a short cooldown after release to prevent instant re-trigger contention
  6) Publishes STOP on state entry / exit for cleaner handoff
  7) Uses safe shutdown guard
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ── Tunable parameters ────────────────────────────────────────────────────────
TRIGGER_DISTANCE = 0.60      # m
CLEAR_DISTANCE = 1.0         # m  # Increased to ensure better clearance before resuming navigation
MIN_RANGE = 0.10             # m
MAX_RANGE = 8.0              # m
FORWARD_ARC = 1.2            # rad  # Increased to detect obstacles at wider angles (better for cylinders)

# Escape behaviour
REVERSE_SPEED = 0.18         # m/s
REVERSE_TURN = 0.55          # rad/s
REVERSE_DIST = 0.60          # m  # Increased to better clear large obstacles like cylinders
REVERSE_TIME_MAX = 3.0       # s  # Increased timeout to allow more reverse distance

# Optional forward nudge once clear
AVOID_LINEAR_DIST = 0.50     # m  # Increased to better clear obstacles
AVOID_LINEAR_SPEED = 0.15    # m/s

# Handoff protection
RELEASE_COOLDOWN = 0.75      # s
# ─────────────────────────────────────────────────────────────────────────────

IDLE = "IDLE"
AVOID_REVERSE_ARC = "AVOID_REVERSE_ARC"
FORWARD_NUDGE = "FORWARD_NUDGE"


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__("avoid_while_interact")

        self.scan_points: list[tuple[float, float]] = []
        self.scan_received = False

        self.state = IDLE
        self.forward_steps = 0

        self.reverse_steps = 0
        self.reverse_started = False
        self.reverse_start_time = None

        self.recharge_active = False
        self.battery_charging = False

        self.release_cooldown_until = None

        self._paused_latched = None
        self._active_latched = None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)
        self.create_subscription(Bool, "/recharge_active", self._cb_recharge_active, 10)
        self.create_subscription(Bool, "/battery_charging", self._cb_battery_charging, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_avoid", 10)
        self.paused_pub = self.create_publisher(Bool, "/navigation_paused", 10)
        self.active_pub = self.create_publisher(Bool, "/avoid_active", 10)

        self.dt = 0.05
        self.create_timer(self.dt, self._safety_loop)

        self.get_logger().info("Obstacle avoidance node running.")
        self.get_logger().info(
            f"  Trigger: {TRIGGER_DISTANCE:.2f} m | Clear: {CLEAR_DISTANCE:.2f} m | "
            f"Arc: +/-{math.degrees(FORWARD_ARC):.1f}°"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE and math.isfinite(r):
                pts.append((angle, r))
            angle += msg.angle_increment
        self.scan_points = pts
        self.scan_received = True

    def _cb_recharge_active(self, msg: Bool):
        new_val = bool(msg.data)
        if new_val != self.recharge_active:
            self.recharge_active = new_val
            if self.recharge_active:
                self.get_logger().info("Recharge active — disabling interaction avoidance.")
                self._force_idle()
    def _cb_battery_charging(self, msg: Bool):
        self.battery_charging = bool(msg.data)

    # ── Main loop ─────────────────────────────────────────────────────────────
    def _safety_loop(self):
        if not self.scan_received:
            return

        # This node must not compete during recharge / charging.
        if self.recharge_active:
            if self.state != IDLE or self._active_latched or self._paused_latched:
                self._force_idle()
            return

        # Short cooldown after release to prevent instant re-trigger contention.
        now = self.get_clock().now()
        if self.release_cooldown_until is not None and now < self.release_cooldown_until:
            if self.state != IDLE:
                self._force_idle()
            return
        if self.release_cooldown_until is not None and now >= self.release_cooldown_until:
            self.release_cooldown_until = None

        nearest_range, nearest_bearing = self._nearest_obstacle()

        active = (self.state != IDLE)
        self._publish_paused(active)
        self._publish_active(active)

        # --- IDLE ---
        if self.state == IDLE:
            if nearest_range < TRIGGER_DISTANCE:
                self.get_logger().warn(
                    f"Obstacle at {nearest_range:.2f} m (bearing "
                    f"{math.degrees(nearest_bearing):.1f}°) — interrupting navigation."
                )
                self.state = AVOID_REVERSE_ARC
                self._publish_paused(True)
                self._publish_active(True)
                self._stop()
                self._start_reverse_phase()
            return

        # --- AVOID_REVERSE_ARC ---
        if self.state == AVOID_REVERSE_ARC:
            if nearest_range >= CLEAR_DISTANCE:
                self._start_forward_nudge()
                self.get_logger().info("Obstacle cleared — starting forward nudge.")
                self.state = FORWARD_NUDGE
                return

            if not self.reverse_started:
                self._start_reverse_phase()

            if self.reverse_start_time is not None:
                elapsed = (now - self.reverse_start_time).nanoseconds * 1e-9
                if elapsed >= REVERSE_TIME_MAX:
                    self.get_logger().warn(
                        "Reverse phase timed out — releasing avoidance.",
                        throttle_duration_sec=1.0,
                    )
                    self._release_with_cooldown()
                    return

            if self.reverse_steps > 0:
                twist = Twist()
                twist.linear.x = -REVERSE_SPEED
                twist.angular.z = REVERSE_TURN * (-1.0 if nearest_bearing >= 0.0 else 1.0)
                self.cmd_pub.publish(twist)
                self.reverse_steps -= 1
                self.get_logger().warn(
                    f"Escaping: {nearest_range:.2f} m at {math.degrees(nearest_bearing):.1f}° "
                    f"— reverse+turn.",
                    throttle_duration_sec=0.5,
                )
                return

            twist = Twist()
            twist.angular.z = 0.5 * (-1.0 if nearest_bearing >= 0.0 else 1.0)
            self.cmd_pub.publish(twist)
            self.get_logger().warn(
                f"Escaping: {nearest_range:.2f} m at {math.degrees(nearest_bearing):.1f}° "
                f"— rotating to clear.",
                throttle_duration_sec=0.5,
            )
            return

        # --- FORWARD_NUDGE ---
        if self.state == FORWARD_NUDGE:
            # Abort forward nudge if something gets too close again.
            if nearest_range < TRIGGER_DISTANCE:
                self.get_logger().warn(
                    f"Obstacle reappeared at {nearest_range:.2f} m — re-entering reverse arc."
                )
                self.state = AVOID_REVERSE_ARC
                self._stop()
                self._start_reverse_phase()
                return

            if self.forward_steps > 0:
                twist = Twist()
                twist.linear.x = AVOID_LINEAR_SPEED
                self.cmd_pub.publish(twist)
                self.forward_steps -= 1
                return

            self.get_logger().info("Forward nudge complete — resuming navigation.")
            self._release_with_cooldown()
            return

    # ── Phase helpers ─────────────────────────────────────────────────────────
    def _start_reverse_phase(self):
        steps = int((REVERSE_DIST / max(REVERSE_SPEED, 1e-6)) / self.dt)
        self.reverse_steps = max(1, steps)
        self.reverse_started = True
        self.reverse_start_time = self.get_clock().now()

    def _start_forward_nudge(self):
        steps = int((AVOID_LINEAR_DIST / max(AVOID_LINEAR_SPEED, 1e-6)) / self.dt)
        self.forward_steps = max(0, steps)

    def _reset_to_idle(self):
        self.state = IDLE
        self.forward_steps = 0
        self.reverse_steps = 0
        self.reverse_started = False
        self.reverse_start_time = None
        self._publish_paused(False)
        self._publish_active(False)

    def _release_with_cooldown(self):
        self._stop()
        self._reset_to_idle()
        self.release_cooldown_until = self.get_clock().now() + rclpy.duration.Duration(
            seconds=RELEASE_COOLDOWN
        )

    def _force_idle(self):
        self._stop()
        self._reset_to_idle()

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _nearest_obstacle(self) -> tuple[float, float]:
        nearest_range = float("inf")
        nearest_bearing = 0.0
        for angle, r in self.scan_points:
            if abs(angle) > FORWARD_ARC:
                continue
            if r < nearest_range:
                nearest_range = r
                nearest_bearing = angle
        return nearest_range, nearest_bearing

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_paused(self, paused: bool):
        if self._paused_latched == paused:
            return
        self._paused_latched = paused
        self.paused_pub.publish(Bool(data=paused))

    def _publish_active(self, active: bool):
        if self._active_latched == active:
            return
        self._active_latched = active
        self.active_pub.publish(Bool(data=active))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()