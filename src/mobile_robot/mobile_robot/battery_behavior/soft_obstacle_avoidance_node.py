#!/usr/bin/env python3
###############################################################################
# Soft Obstacle Avoidance Node (UPDATED FOR SUPERVISOR ARBITER)
#
# Key updates you requested:
#   - Add a bounded avoidance policy so it DOES NOT get stuck rotating forever.
#   - If no progress (range not increasing) for a while, RELEASE /navigation_paused
#     so recharge navigation can resume.
#   - Add a short cooldown after releasing to avoid immediate re-trigger loops.
#
# Publishes:
#   /cmd_vel_soft_avoid    (geometry_msgs/Twist)   candidate command during recharge
#   /navigation_paused     (std_msgs/Bool)         gate for recharge-only soft avoid
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

        # NEW: anti-stuck behavior
        self.declare_parameter('max_avoid_sec',          8.0)   # hard cap for AVOIDING time
        self.declare_parameter('no_progress_sec',        2.5)   # if range doesn't improve for this long -> release
        self.declare_parameter('progress_epsilon_m',     0.05)  # improvement threshold (meters)
        self.declare_parameter('release_cooldown_sec',   1.0)   # ignore triggers briefly after releasing

        self._trigger_dist    = float(self.get_parameter('trigger_distance').value)
        self._clear_dist      = float(self.get_parameter('clear_distance').value)
        self._forward_arc     = math.radians(float(self.get_parameter('forward_arc_deg').value))
        self._avoid_ang_spd   = float(self.get_parameter('avoid_angular_spd').value)
        self._avoid_lin_spd   = float(self.get_parameter('avoid_linear_spd').value)
        self._avoid_lin_dist  = float(self.get_parameter('avoid_linear_dist').value)
        self._min_range       = float(self.get_parameter('min_range').value)
        self._max_range       = float(self.get_parameter('max_range').value)
        self._control_hz      = float(self.get_parameter('control_hz').value)

        self._max_avoid_sec       = float(self.get_parameter('max_avoid_sec').value)
        self._no_progress_sec     = float(self.get_parameter('no_progress_sec').value)
        self._progress_eps        = float(self.get_parameter('progress_epsilon_m').value)
        self._release_cooldown_sec = float(self.get_parameter('release_cooldown_sec').value)

        # ── State ─────────────────────────────────────────────────────────────
        self._state           = IDLE
        self._scan_points     = []       # [(angle_rad, range_m), ...]
        self._scan_received   = False
        self._recharge_active = False
        self._nudge_steps     = 0

        # Latch for paused publishing (avoid spamming)
        self._paused = False

        # NEW: anti-stuck timers/progress
        self._avoid_start_time = None          # rclpy.time.Time
        self._last_progress_time = None        # rclpy.time.Time
        self._best_range = None                # float
        self._last_release_time = None         # rclpy.time.Time

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
        # Candidate topic; supervisor owns /cmd_vel
        self._cmd_pub_soft = self.create_publisher(Twist, '/cmd_vel_soft_avoid', 10)
        self._paused_pub   = self.create_publisher(Bool,  '/navigation_paused', 10)

        self.create_timer(1.0 / self._control_hz, self._safety_loop)

        self.get_logger().info(
            f'SoftObstacleAvoidanceNode ready. '
            f'trigger={self._trigger_dist}m  clear={self._clear_dist}m  '
            f'arc=±{math.degrees(self._forward_arc):.0f}°  '
            f'max_avoid={self._max_avoid_sec:.1f}s  no_progress={self._no_progress_sec:.1f}s  '
            f'cooldown={self._release_cooldown_sec:.1f}s'
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
        now = self.get_clock().now()

        # Only run avoidance when recharge is active
        if not self._recharge_active:
            if self._state != IDLE:
                self._exit_avoidance()
            return

        if not self._scan_received:
            return

        # Cooldown after release to avoid re-trigger loops
        if self._last_release_time is not None:
            since_rel = (now - self._last_release_time).nanoseconds * 1e-9
            if since_rel < self._release_cooldown_sec:
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
                self._enter_avoiding(now, nearest_range)
                self._publish_paused(True)
                self._publish_stop()

        # ── AVOIDING: rotate away from nearest obstacle ───────────────────────
        elif self._state == AVOIDING:
            # Success condition
            if nearest_range >= self._clear_dist:
                self._enter_nudging()
                return

            # Anti-stuck: progress tracking + timeouts
            if self._avoid_start_time is None:
                self._avoid_start_time = now
            if self._last_progress_time is None:
                self._last_progress_time = now
            if self._best_range is None:
                self._best_range = nearest_range

            total = (now - self._avoid_start_time).nanoseconds * 1e-9
            no_prog = (now - self._last_progress_time).nanoseconds * 1e-9

            # Update "progress" if range improves meaningfully
            if nearest_range > (self._best_range + self._progress_eps):
                self._best_range = nearest_range
                self._last_progress_time = now
                no_prog = 0.0

            # Hard cap or no-progress release
            if (total >= self._max_avoid_sec) or (no_prog >= self._no_progress_sec):
                self.get_logger().warn(
                    f'Avoidance release: total={total:.1f}s, no_progress={no_prog:.1f}s, '
                    f'nearest={nearest_range:.2f}m @ {math.degrees(nearest_bearing):.1f}°. '
                    f'Releasing recharge navigation.'
                )
                self._publish_stop()
                self._publish_paused(False)
                self._state = WATCHING
                self._reset_avoid_progress()
                self._last_release_time = now
                return

            # Continue avoidance rotation
            twist = Twist()
            # Turn away: obstacle left (+bearing) => turn right (-)
            twist.angular.z = (-self._avoid_ang_spd if nearest_bearing >= 0 else self._avoid_ang_spd)
            self._cmd_pub_soft.publish(twist)
            self.get_logger().warn(
                f'Avoiding: {nearest_range:.2f} m at {math.degrees(nearest_bearing):.1f}° — rotating.',
                throttle_duration_sec=0.5,
            )

        # ── NUDGING: short forward drive before handing back ──────────────────
        elif self._state == NUDGING:
            if self._nudge_steps > 0:
                twist = Twist()
                twist.linear.x = self._avoid_lin_spd
                self._cmd_pub_soft.publish(twist)
                self._nudge_steps -= 1
            else:
                self.get_logger().info('Nudge complete — resuming recharge navigation.')
                self._state = WATCHING
                self._publish_stop()
                self._publish_paused(False)
                self._reset_avoid_progress()
                self._last_release_time = now

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
        self._cmd_pub_soft.publish(Twist())

    def _publish_paused(self, paused: bool):
        # Only publish on changes (reduces noise)
        if paused == self._paused:
            return
        self._paused = paused
        self._paused_pub.publish(Bool(data=paused))

    def _enter_avoiding(self, now, nearest_range: float):
        self._state = AVOIDING
        self._avoid_start_time = now
        self._last_progress_time = now
        self._best_range = nearest_range

    def _enter_nudging(self):
        steps = int((self._avoid_lin_dist / max(self._avoid_lin_spd, 1e-6)) * self._control_hz)
        self._nudge_steps = max(0, steps)
        self._state = NUDGING
        self.get_logger().info('Obstacle cleared — nudging forward.')
        self._reset_avoid_progress()

    def _reset_avoid_progress(self):
        self._avoid_start_time = None
        self._last_progress_time = None
        self._best_range = None

    def _exit_avoidance(self):
        self._state = IDLE
        self._nudge_steps = 0
        self._publish_stop()
        self._publish_paused(False)
        self._reset_avoid_progress()
        self._last_release_time = self.get_clock().now()


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