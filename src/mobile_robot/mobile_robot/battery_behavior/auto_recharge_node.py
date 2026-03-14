#!/usr/bin/env python3
###############################################################################
# Auto Recharge Node  (UPDATED FOR SUPERVISOR ARBITER)
#
# Publishes motion candidate to: /cmd_vel_recharge
###############################################################################

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


IDLE = 'IDLE'
TURNING = 'TURNING'
DRIVING = 'DRIVING'
ARRIVED = 'ARRIVED'


class AutoRechargeNode(Node):

    def __init__(self):
        super().__init__('auto_recharge_node')

        # Shared scaling / motion parameters
        self.declare_parameter('gain_multiplier', 1.0)
        self.declare_parameter('base_linear_speed', 0.5)
        self.declare_parameter('base_angular_speed', 1.2)

        # Recharge-specific parameters
        self.declare_parameter('low_battery_threshold', 0.25)
        self.declare_parameter('arrived_radius', 0.50)
        self.declare_parameter('k_linear', 0.4)
        self.declare_parameter('k_angular', 1.6)
        self.declare_parameter('turn_only_angle', 0.25)
        self.declare_parameter('drive_angle', 0.40)
        self.declare_parameter('control_hz', 20.0)

        # Match the battery YAML format
        self.declare_parameter('charging_station_x', [-20.0, 20.0])
        self.declare_parameter('charging_station_y', [0.0, 0.0])

        # Read charger coordinates
        xs = list(self.get_parameter('charging_station_x').value)
        ys = list(self.get_parameter('charging_station_y').value)

        n = min(len(xs), len(ys))
        if len(xs) != len(ys):
            self.get_logger().warn(
                f"charging_station_x has length {len(xs)} but charging_station_y has length {len(ys)}. "
                f"Using first {n} station pairs."
            )

        self._stations = [(float(xs[i]), float(ys[i])) for i in range(n)]

        if not self._stations:
            self.get_logger().warn(
                "No charging stations provided. Falling back to default stations."
            )
            self._stations = [(-20.0, 0.0), (20.0, 0.0)]

        # Read other parameters
        self._gain = float(self.get_parameter('gain_multiplier').value)
        self._base_linear = float(self.get_parameter('base_linear_speed').value)
        self._base_angular = float(self.get_parameter('base_angular_speed').value)

        self._low_thresh = float(self.get_parameter('low_battery_threshold').value)
        self._arrived_radius = float(self.get_parameter('arrived_radius').value)
        self._k_linear = float(self.get_parameter('k_linear').value)
        self._k_angular = float(self.get_parameter('k_angular').value)
        self._turn_only_angle = float(self.get_parameter('turn_only_angle').value)
        self._drive_angle = float(self.get_parameter('drive_angle').value)
        self._control_hz = float(self.get_parameter('control_hz').value)

        # Derived limits from shared speed settings
        self._max_linear = self._base_linear * self._gain
        self._max_angular = self._base_angular * self._gain

        self.get_logger().info(
            f'AutoRecharge: loaded {len(self._stations)} stations: {self._stations}'
        )
        self.get_logger().info(
            f'low_battery={self._low_thresh*100:.0f}%  '
            f'arrived_radius={self._arrived_radius}m  '
            f'max_linear={self._max_linear}m/s  '
            f'max_angular={self._max_angular}rad/s'
        )

        # State
        self._battery_pct = 1.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._yaw = 0.0
        self._nav_state = IDLE
        self._target_station = None
        self._avoidance_paused = False

        # Latching
        self._active_latched = False

        # Subscriptions
        self.create_subscription(BatteryState, '/battery_status', self._battery_cb, 10)
        self.create_subscription(Odometry, '/odom_gt', self._odom_cb, 10)
        self.create_subscription(Bool, '/navigation_paused', self._paused_cb, 10)

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_recharge', 10)
        self._active_pub = self.create_publisher(Bool, '/recharge_active', 10)

        self.create_timer(1.0 / self._control_hz, self._control_loop)

        self.get_logger().info('AutoRechargeNode ready.')

    def _battery_cb(self, msg: BatteryState):
        self._battery_pct = float(msg.percentage)

    def _odom_cb(self, msg: Odometry):
        self._pos_x = float(msg.pose.pose.position.x)
        self._pos_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def _paused_cb(self, msg: Bool):
        was = self._avoidance_paused
        self._avoidance_paused = bool(msg.data)
        if self._avoidance_paused and not was:
            self._stop()

    def _nearest_station(self):
        best = None
        best_dist = float('inf')
        for sx, sy in self._stations:
            d = math.hypot(self._pos_x - sx, self._pos_y - sy)
            if d < best_dist:
                best_dist = d
                best = (sx, sy)
        return best

    def _dist_to_target(self):
        if self._target_station is None:
            return float('inf')
        return math.hypot(
            self._pos_x - self._target_station[0],
            self._pos_y - self._target_station[1]
        )

    def _angle_to_target(self):
        if self._target_station is None:
            return 0.0
        dx = self._target_station[0] - self._pos_x
        dy = self._target_station[1] - self._pos_y
        desired_yaw = math.atan2(dy, dx)
        err = desired_yaw - self._yaw
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi
        return err

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_active(self, active: bool):
        if active == self._active_latched:
            return
        self._active_latched = active
        self._active_pub.publish(Bool(data=active))

    def _control_loop(self):
        if self._nav_state == IDLE:
            if self._battery_pct < self._low_thresh:
                self._target_station = self._nearest_station()
                self._nav_state = TURNING
                self.get_logger().warn(
                    f'Battery low ({self._battery_pct*100:.1f}%) — '
                    f'heading to station {self._target_station}'
                )
                self._publish_active(True)
            else:
                self._publish_active(False)
                return

        if self._battery_pct >= 0.98:
            self.get_logger().info('Battery full — returning to IDLE.')
            self._stop()
            self._nav_state = IDLE
            self._target_station = None
            self._publish_active(False)
            return

        self._publish_active(True)

        if self._avoidance_paused:
            return

        if self._nav_state == ARRIVED:
            self._stop()
            return

        dist = self._dist_to_target()
        if dist <= self._arrived_radius:
            self._nav_state = ARRIVED
            self._stop()
            self.get_logger().info(
                f'Arrived at charging station {self._target_station} '
                f'(dist={dist:.2f}m) — holding position for charging.'
            )
            return

        ang_err = self._angle_to_target()
        cmd = Twist()

        if self._nav_state == TURNING:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -self._max_angular,
                min(self._max_angular, self._k_angular * ang_err)
            )
            if abs(ang_err) < self._turn_only_angle:
                self._nav_state = DRIVING
                self.get_logger().info('Aligned — switching to DRIVING.')

        elif self._nav_state == DRIVING:
            if abs(ang_err) > self._drive_angle:
                self._nav_state = TURNING
                cmd.linear.x = 0.0
                cmd.angular.z = max(
                    -self._max_angular,
                    min(self._max_angular, self._k_angular * ang_err)
                )
            else:
                cmd.angular.z = max(
                    -self._max_angular,
                    min(self._max_angular, self._k_angular * ang_err)
                )
                cmd.linear.x = max(
                    0.0,
                    min(self._max_linear, self._k_linear * dist)
                )

        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[{self._nav_state}] dist={dist:.2f}m  '
            f'ang_err={math.degrees(ang_err):.1f}°  '
            f'bat={self._battery_pct*100:.1f}%',
            throttle_duration_sec=1.0
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