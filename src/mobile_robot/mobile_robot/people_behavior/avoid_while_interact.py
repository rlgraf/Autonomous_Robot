#!/usr/bin/env python3
"""
Node 3: obstacle_avoidance.py (ROS2) — UPDATED FOR SUPERVISOR ARBITER

Edits applied (minimal, supervisor-friendly):
  1) Publish velocity to /cmd_vel_avoid (NOT /cmd_vel).
     - Supervisor should be the only writer to /cmd_vel.
  2) Add /avoid_active publisher (Bool) as a cleaner “I am requesting control” signal.
     - You can keep /navigation_paused for legacy interlock with navigator.
  3) Latch /navigation_paused and /avoid_active to avoid spamming.
  4) Optional: publish a STOP on state entry / exit to prevent stale motion.

Topic layout (after edit):
  Pub  /cmd_vel_avoid        geometry_msgs/Twist   (candidate cmd)
  Pub  /navigation_paused    std_msgs/Bool         (True => nav should yield)
  Pub  /avoid_active         std_msgs/Bool         (True => avoidance wants priority)
  Sub  /scan                 sensor_msgs/LaserScan
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ── Tunable parameters ────────────────────────────────────────────────────────
TRIGGER_DISTANCE   = 0.6    # m
CLEAR_DISTANCE     = 0.7    # m
MIN_RANGE          = 0.10   # m
MAX_RANGE          = 8.0    # m
FORWARD_ARC        = 0.8    # rad

# Escape behaviour
REVERSE_SPEED      = 0.18   # m/s
REVERSE_TURN       = 0.55   # rad/s
REVERSE_DIST       = 0.35   # m
REVERSE_TIME_MAX   = 2.5    # s

# Optional forward nudge once clear
AVOID_LINEAR_DIST  = 0.5    # m
AVOID_LINEAR_SPEED = 0.15   # m/s
# ─────────────────────────────────────────────────────────────────────────────

IDLE = 'IDLE'
AVOID_REVERSE_ARC = 'AVOID_REVERSE_ARC'
FORWARD_NUDGE = 'FORWARD_NUDGE'


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.scan_points: list[tuple[float, float]] = []  # (angle, range)
        self.scan_received = False

        self.state = IDLE
        self.forward_steps = 0

        # Reverse-arc bookkeeping
        self.reverse_steps = 0
        self.reverse_started = False
        self.reverse_start_time = None

        # Latched pubs (avoid spamming)
        self._paused_latched = None  # type: bool | None
        self._active_latched = None  # type: bool | None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)

        # IMPORTANT: publish candidate cmd on /cmd_vel_avoid (not /cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_avoid', 10)

        # Legacy interlock (navigator listens to this and yields)
        self.paused_pub = self.create_publisher(Bool, '/navigation_paused', 10)

        # Cleaner “avoidance requests control” signal for supervisor
        self.active_pub = self.create_publisher(Bool, '/avoid_active', 10)

        self.dt = 0.05  # 20 Hz
        self.create_timer(self.dt, self._safety_loop)

        self.get_logger().info('Obstacle avoidance node running.')
        self.get_logger().info(
            f'  Trigger: {TRIGGER_DISTANCE:.2f} m | Clear: {CLEAR_DISTANCE:.2f} m | '
            f'Arc: +/-{math.degrees(FORWARD_ARC):.1f}°'
        )

    # ── Scan callback ──────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE and math.isfinite(r):
                pts.append((angle, r))
            angle += msg.angle_increment
        self.scan_points = pts
        self.scan_received = True

    # ── Safety loop ────────────────────────────────────────────────────────────
    def _safety_loop(self):
        if not self.scan_received:
            return

        nearest_range, nearest_bearing = self._nearest_obstacle()

        # Publish interlocks according to state (latched)
        paused = (self.state != IDLE)
        self._publish_paused(paused)
        self._publish_active(paused)

        # --- IDLE ---
        if self.state == IDLE:
            if nearest_range < TRIGGER_DISTANCE:
                self.get_logger().warn(
                    f'Obstacle at {nearest_range:.2f} m — interrupting navigation.'
                )
                self.state = AVOID_REVERSE_ARC
                self._publish_paused(True)
                self._publish_active(True)
                self._stop()
                self._start_reverse_phase()
            return

        # --- AVOID_REVERSE_ARC ---
        if self.state == AVOID_REVERSE_ARC:
            # If already clear, transition to forward nudge (optional) or release.
            if nearest_range >= CLEAR_DISTANCE:
                self._start_forward_nudge()
                self.get_logger().info('Obstacle cleared — starting forward nudge.')
                self.state = FORWARD_NUDGE
                return

            # Ensure reverse phase is initialized
            if not self.reverse_started:
                self._start_reverse_phase()

            # Timeout protection for reverse phase
            if self.reverse_start_time is not None:
                elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds * 1e-9
                if elapsed >= REVERSE_TIME_MAX:
                    twist = Twist()
                    twist.angular.z = 0.6 * (-1.0 if nearest_bearing >= 0 else 1.0)
                    self.cmd_pub.publish(twist)
                    self.get_logger().warn(
                        'Reverse phase timed out — rotating-only fallback.',
                        throttle_duration_sec=1.0,
                    )
                    return

            # Reverse arc for a minimum distance/time budget
            if self.reverse_steps > 0:
                twist = Twist()
                twist.linear.x = -REVERSE_SPEED
                twist.angular.z = REVERSE_TURN * (-1.0 if nearest_bearing >= 0 else 1.0)
                self.cmd_pub.publish(twist)
                self.reverse_steps -= 1
                self.get_logger().warn(
                    f'Escaping: {nearest_range:.2f} m at {math.degrees(nearest_bearing):.1f}° '
                    f'— reverse+turn.',
                    throttle_duration_sec=0.5,
                )
                return

            # After minimum reverse, continue with rotate-away until clear.
            twist = Twist()
            twist.angular.z = 0.5 * (-1.0 if nearest_bearing >= 0 else 1.0)
            self.cmd_pub.publish(twist)
            self.get_logger().warn(
                f'Escaping: {nearest_range:.2f} m at {math.degrees(nearest_bearing):.1f}° '
                f'— rotating to clear.',
                throttle_duration_sec=0.5,
            )
            return

        # --- FORWARD_NUDGE ---
        if self.state == FORWARD_NUDGE:
            if self.forward_steps > 0:
                twist = Twist()
                twist.linear.x = AVOID_LINEAR_SPEED
                self.cmd_pub.publish(twist)
                self.forward_steps -= 1
                return

            self.get_logger().info('Forward nudge complete — resuming navigation.')
            self._stop()
            self._reset_to_idle()
            return

    # ── Phase helpers ──────────────────────────────────────────────────────────
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

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _nearest_obstacle(self) -> tuple[float, float]:
        nearest_range = float('inf')
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
        if self._paused_latched is paused:
            return
        self._paused_latched = paused
        self.paused_pub.publish(Bool(data=paused))

    def _publish_active(self, active: bool):
        if self._active_latched is active:
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()