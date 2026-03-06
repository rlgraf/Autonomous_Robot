#!/usr/bin/env python3
###############################################################################
# Auto Recharge Node
#
# Publishes navigation commands to /recharge/cmd_vel — NOT /cmd_vel directly.
# soft_obstacle_avoidance_node is the sole owner of /cmd_vel and either
# forwards these commands unchanged (no obstacle) or overrides with avoidance
# rotation (obstacle detected). No boolean handshake needed.
#
# Topics
# ──────
#   Sub  battery_status           sensor_msgs/BatteryState
#   Sub  /odom_gt                 nav_msgs/Odometry
#   Pub  /recharge/cmd_vel        geometry_msgs/Twist   → soft_avoid reads this
#   Pub  /recharge_active         std_msgs/Bool
#   Pub  /target_station_dist     std_msgs/Float32
###############################################################################

import math
import os

import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


IDLE       = 'IDLE'
NAVIGATING = 'NAVIGATING'
ARRIVED    = 'ARRIVED'


class AutoRechargeNode(Node):

    def __init__(self):
        super().__init__('auto_recharge_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('arrived_radius',             0.40)
        self.declare_parameter('max_linear',                 5.0)
        self.declare_parameter('max_angular',                5.0)
        self.declare_parameter('k_linear',                   0.4)
        self.declare_parameter('k_angular',                  1.6)
        self.declare_parameter('full_speed_angle',           0.20)
        self.declare_parameter('stop_drive_angle',           0.60)
        self.declare_parameter('control_hz',                 20.0)
        self.declare_parameter('return_mode',                'threshold')
        self.declare_parameter('low_battery_threshold',      0.25)
        self.declare_parameter('prediction_cruise_fraction', 0.6)
        self.declare_parameter('prediction_turn_fraction',   0.3)
        self.declare_parameter('safety_margin_ah',           0.1)

        self._arrived_radius   = self.get_parameter('arrived_radius').value
        self._max_linear       = self.get_parameter('max_linear').value
        self._max_angular      = self.get_parameter('max_angular').value
        self._k_linear         = self.get_parameter('k_linear').value
        self._k_angular        = self.get_parameter('k_angular').value
        self._full_speed_angle = self.get_parameter('full_speed_angle').value
        self._stop_drive_angle = self.get_parameter('stop_drive_angle').value
        self._control_hz       = self.get_parameter('control_hz').value
        self._return_mode      = self.get_parameter('return_mode').value
        self._low_thresh       = self.get_parameter('low_battery_threshold').value
        self._cruise_fraction  = self.get_parameter('prediction_cruise_fraction').value
        self._turn_fraction    = self.get_parameter('prediction_turn_fraction').value
        self._safety_margin    = self.get_parameter('safety_margin_ah').value

        # ── Load battery params + stations from YAML ──────────────────────────
        params_file = os.path.join(
            get_package_share_directory('mobile_robot'),
            'parameters', 'battery_tunable_parameters.yaml'
        )
        with open(params_file, 'r') as f:
            raw = yaml.safe_load(f)
        bat_params = raw['battery_node']['ros__parameters']

        self._stations        = [tuple(s) for s in bat_params['charging_stations']]
        self._capacity        = bat_params['battery_capacity_ah']
        self._charging_radius = bat_params['charging_radius']

        # Warn if arrived_radius would stop the robot outside charging range
        if self._arrived_radius > self._charging_radius:
            self.get_logger().warn(
                f'arrived_radius ({self._arrived_radius}m) > '
                f'charging_radius ({self._charging_radius}m) — '
                f'robot will stop before entering charging zone! '
                f'Set arrived_radius < {self._charging_radius}m.'
            )

        # ── Derive drain_per_metre ─────────────────────────────────────────────
        idle_drain   = bat_params['idle_drain_a']
        move_drain   = bat_params['move_drain_per_ms']
        turn_drain   = bat_params['turn_drain_per_rads']
        cruise_speed = self._max_linear  * self._cruise_fraction
        turn_speed   = self._max_angular * self._turn_fraction

        if cruise_speed <= 0.0:
            cruise_speed = 0.1
            self.get_logger().warn('cruise_speed=0 — defaulting to 0.1 m/s')

        total_current_a       = idle_drain + move_drain * cruise_speed + turn_drain * turn_speed
        self._drain_per_metre = total_current_a / (cruise_speed * 3600.0)

        self.get_logger().info(
            f'AutoRecharge: {len(self._stations)} stations | mode={self._return_mode} | '
            f'arrived_radius={self._arrived_radius}m (charging_radius={self._charging_radius}m)\n'
            f'  drain_per_metre={self._drain_per_metre:.5f} Ah/m  '
            f'safety_margin={self._safety_margin:.3f} Ah'
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._battery_pct    = 1.0
        self._battery_ah     = self._capacity
        self._pos_x          = 0.0
        self._pos_y          = 0.0
        self._yaw            = 0.0
        self._nav_state      = IDLE
        self._target_station = None

        # Progress watchdog
        self._last_dist_check   = float('inf')
        self._no_progress_ticks = 0
        self._watchdog_ticks    = int(self._control_hz * 5.0)
        self._progress_margin   = 0.10

        self.charge_needed      = self._low_thresh * self._capacity

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(BatteryState, 'battery_status', self._battery_cb, 10)
        self.create_subscription(Odometry,     '/odom_gt',       self._odom_cb,    10)

        # ── Publishers ────────────────────────────────────────────────────────
        # NOTE: publishes to /recharge/cmd_vel, NOT /cmd_vel
        # soft_obstacle_avoidance_node owns /cmd_vel and forwards this through
        self._cmd_pub          = self.create_publisher(Twist,   '/recharge/cmd_vel',    10)
        self._active_pub       = self.create_publisher(Bool,    '/recharge/recharge_active',     10)
        self._station_dist_pub = self.create_publisher(Float32, '/recharge/target_station_dist', 10)
        self._threshold_pub = self.create_publisher(Float32, '/recharge/low_battery_threshold', 10)     

        self.create_timer(1.0 / self._control_hz, self._control_loop)
        self.get_logger().info('AutoRechargeNode ready — publishing to /recharge/cmd_vel.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _battery_cb(self, msg: BatteryState):
        self._battery_pct = msg.percentage
        self._battery_ah  = msg.charge

    def _odom_cb(self, msg: Odometry):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    # ── Return-home trigger ───────────────────────────────────────────────────

    def _should_return_to_charge(self) -> bool:
        if self._return_mode == 'prediction':
            nearest = self._nearest_station()
            if nearest is None:
                return False
            dist          = math.hypot(self._pos_x - nearest[0], self._pos_y - nearest[1])
            self.charge_needed = dist * self._drain_per_metre + self._safety_margin
            trigger       = self._battery_ah <= self.charge_needed
            self.get_logger().info(
                f'[prediction] dist={dist:.1f}m  '
                f'need={self.charge_needed:.4f}Ah  have={self._battery_ah:.4f}Ah  '
                f'→ {"RETURN" if trigger else "ok"}',
                throttle_duration_sec=5.0,
            )
            return trigger
        else:
            return self._battery_pct < self._low_thresh

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _nearest_station(self):
        best, best_dist = None, float('inf')
        for sx, sy in self._stations:
            d = math.hypot(self._pos_x - sx, self._pos_y - sy)
            if d < best_dist:
                best_dist = d
                best = (sx, sy)
        return best

    def _dist_to_target(self) -> float:
        if self._target_station is None:
            return float('inf')
        return math.hypot(
            self._pos_x - self._target_station[0],
            self._pos_y - self._target_station[1],
        )

    def _angle_to_target(self) -> float:
        dx  = self._target_station[0] - self._pos_x
        dy  = self._target_station[1] - self._pos_y
        err = math.atan2(dy, dx) - self._yaw
        while err >  math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi
        return err

    def _linear_speed_blend(self, ang_err: float) -> float:
        abs_err = abs(ang_err)
        if abs_err <= self._full_speed_angle:
            return 1.0
        if abs_err >= self._stop_drive_angle:
            return 0.0
        span = self._stop_drive_angle - self._full_speed_angle
        return 1.0 - (abs_err - self._full_speed_angle) / span

    def _publish_zero(self):
        """Publish a zero twist — tells soft_avoid to hold position."""
        self._cmd_pub.publish(Twist())

    def _publish_active(self, active: bool):
        self._active_pub.publish(Bool(data=active))

    def _publish_station_dist(self, dist: float):
        self._station_dist_pub.publish(Float32(data=float(dist)))
    
    def _publish_threshold(self):
        """
        Publish the effective low-battery threshold as a fraction (0-1)
        so battery_node can use it for /low_battery_warning.
        In prediction mode, convert safety_margin_ah to an equivalent percentage
        so move6 gets warned at a meaningful level.
        """
        if self._return_mode == 'prediction':
            # Use safety_margin as the warning floor — at minimum this much
            # charge must remain, expressed as a fraction of capacity
            thresh = self.charge_needed / self._capacity
        else:
            thresh = self._low_thresh
        self._threshold_pub.publish(Float32(data=float(thresh)))

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        self._publish_threshold()   # ← add this as first line
        # ── IDLE ──────────────────────────────────────────────────────────────
        if self._nav_state == IDLE:
            if self._should_return_to_charge():
                self._target_station    = self._nearest_station()
                self._nav_state         = NAVIGATING
                self._last_dist_check   = self._dist_to_target()
                self._no_progress_ticks = 0
                self.get_logger().warn(
                    f'[{self._return_mode}] Battery low — heading to '
                    f'{self._target_station} | '
                    f'bat={self._battery_pct*100:.1f}% ({self._battery_ah:.4f} Ah)'
                )
                self._publish_active(True)
            else:
                self._publish_active(False)
                self._publish_station_dist(float('inf'))
                return

        dist = self._dist_to_target()
        self._publish_station_dist(dist)

        # ── ARRIVED: publish zero so soft_avoid forwards nothing ──────────────
        if self._nav_state == ARRIVED:
            self._publish_zero()
            if self._battery_pct >= 0.98:
                self.get_logger().info('Battery full — returning to IDLE.')
                self._nav_state      = IDLE
                self._target_station = None
                self._publish_active(False)
                self._publish_station_dist(float('inf'))
            return

        # ── Check arrival ─────────────────────────────────────────────────────
        if dist <= self._arrived_radius:
            self._nav_state = ARRIVED
            self._publish_zero()
            self.get_logger().info(
                f'Arrived at {self._target_station} (dist={dist:.2f}m) — charging.'
            )
            return

        # ── NAVIGATING ────────────────────────────────────────────────────────
        ang_err = self._angle_to_target()
        blend   = self._linear_speed_blend(ang_err)

        cmd = Twist()
        cmd.angular.z = max(-self._max_angular,
                            min(self._max_angular, self._k_angular * ang_err))
        cmd.linear.x  = blend * min(self._max_linear, self._k_linear * dist)
        self._cmd_pub.publish(cmd)

        # ── Progress watchdog ─────────────────────────────────────────────────
        self._no_progress_ticks += 1
        if self._no_progress_ticks >= self._watchdog_ticks:
            improvement = self._last_dist_check - dist
            if improvement < self._progress_margin:
                self.get_logger().warn(
                    f'Progress watchdog: only closed {improvement:.2f}m in '
                    f'{self._watchdog_ticks / self._control_hz:.0f}s'
                )
            self._last_dist_check   = dist
            self._no_progress_ticks = 0

        self.get_logger().info(
            f'[NAVIGATING] dist={dist:.2f}m  ang={math.degrees(ang_err):.1f}°  '
            f'blend={blend:.2f}  lin={cmd.linear.x:.2f}m/s  '
            f'bat={self._battery_pct*100:.1f}% ({self._battery_ah:.4f}Ah)',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoRechargeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()