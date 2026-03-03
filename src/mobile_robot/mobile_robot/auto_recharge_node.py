#!/usr/bin/env python3
###############################################################################
# Auto Recharge Node
#
# Monitors battery_status. When percentage drops below low_battery_threshold,
# computes the nearest charging station from battery_tunable_parameters.yaml
# and navigates there using a turn-first / drive state machine.
#
# All tuning parameters live in battery_tunable_parameters.yaml under
# auto_recharge_node/ros__parameters.
#
# Topics
# ──────
#   Sub  battery_status          sensor_msgs/BatteryState
#   Sub  /odom_gt                nav_msgs/Odometry        (ground-truth pose)
#   Sub  /navigation_paused      std_msgs/Bool            (from avoidance node)
#   Pub  /cmd_vel                geometry_msgs/Twist
#   Pub  /recharge_active        std_msgs/Bool            (True while homing)
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
from std_msgs.msg import Bool


IDLE    = 'IDLE'
TURNING = 'TURNING'
DRIVING = 'DRIVING'
ARRIVED = 'ARRIVED'


class AutoRechargeNode(Node):

    def __init__(self):
        super().__init__('auto_recharge_node')

        # ── Declare parameters (values come from battery_tunable_parameters.yaml) ──
        self.declare_parameter('low_battery_threshold', 0.25)
        self.declare_parameter('arrived_radius',        0.60)
        self.declare_parameter('max_linear',            0.15)
        self.declare_parameter('max_angular',           0.5)
        self.declare_parameter('k_linear',              0.4)
        self.declare_parameter('k_angular',             1.6)
        self.declare_parameter('turn_only_angle',       0.25)
        self.declare_parameter('drive_angle',           0.40)
        self.declare_parameter('control_hz',            20.0)

        self._low_thresh      = self.get_parameter('low_battery_threshold').value
        self._arrived_radius  = self.get_parameter('arrived_radius').value
        self._max_linear      = self.get_parameter('max_linear').value
        self._max_angular     = self.get_parameter('max_angular').value
        self._k_linear        = self.get_parameter('k_linear').value
        self._k_angular       = self.get_parameter('k_angular').value
        self._turn_only_angle = self.get_parameter('turn_only_angle').value
        self._drive_angle     = self.get_parameter('drive_angle').value
        self._control_hz      = self.get_parameter('control_hz').value

        # ── Load charging stations from YAML (single source of truth) ──────────
        # Stations use nested list format that doesn't map cleanly to ROS2 params,
        # so we read the file directly — same pattern as battery_node.py
        params_file = os.path.join(
            get_package_share_directory('mobile_robot'),
            'parameters', 'battery_tunable_parameters.yaml'
        )
        with open(params_file, 'r') as f:
            raw = yaml.safe_load(f)
        ros_params = raw['battery_node']['ros__parameters']
        self._stations = [tuple(s) for s in ros_params['charging_stations']]

        self.get_logger().info(
            f'AutoRecharge: loaded {len(self._stations)} stations: {self._stations}'
        )
        self.get_logger().info(
            f'  low_battery={self._low_thresh*100:.0f}%  arrived_radius={self._arrived_radius}m  '
            f'max_linear={self._max_linear}m/s  max_angular={self._max_angular}rad/s'
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._battery_pct      = 1.0
        self._pos_x            = 0.0
        self._pos_y            = 0.0
        self._yaw              = 0.0
        self._nav_state        = IDLE
        self._target_station   = None   # (x, y)
        self._avoidance_paused = False  # avoidance node is currently overriding cmd_vel

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(BatteryState, 'battery_status',     self._battery_cb, 10)
        self.create_subscription(Odometry,     '/odom_gt',           self._odom_cb,    10)
        self.create_subscription(Bool,         '/navigation_paused', self._paused_cb,  10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist, '/cmd_vel',         10)
        self._active_pub = self.create_publisher(Bool,  '/recharge_active', 10)

        self.create_timer(1.0 / self._control_hz, self._control_loop)

        self.get_logger().info('AutoRechargeNode ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _battery_cb(self, msg: BatteryState):
        self._battery_pct = msg.percentage

    def _odom_cb(self, msg: Odometry):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def _paused_cb(self, msg: Bool):
        self._avoidance_paused = msg.data

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _nearest_station(self):
        best, best_dist = None, float('inf')
        for sx, sy in self._stations:
            d = math.hypot(self._pos_x - sx, self._pos_y - sy)
            if d < best_dist:
                best_dist = d
                best = (sx, sy)
        return best

    def _dist_to_target(self):
        if self._target_station is None:
            return float('inf')
        return math.hypot(self._pos_x - self._target_station[0],
                          self._pos_y - self._target_station[1])

    def _angle_to_target(self):
        dx = self._target_station[0] - self._pos_x
        dy = self._target_station[1] - self._pos_y
        desired_yaw = math.atan2(dy, dx)
        err = desired_yaw - self._yaw
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_active(self, active: bool):
        self._active_pub.publish(Bool(data=active))

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):

        # ── Trigger homing when battery is low ────────────────────────────────
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

        # ── Yield to obstacle avoidance ───────────────────────────────────────
        if self._avoidance_paused:
            return

        # ── ARRIVED: hold position and wait for battery_node to charge ────────
        if self._nav_state == ARRIVED:
            # Keep publishing zero every tick so battery_node sees stationary=True
            self._stop()
            if self._battery_pct >= 0.98:
                self.get_logger().info('Battery full — returning to IDLE.')
                self._nav_state = IDLE
                self._target_station = None
            return

        # ── Check arrival ─────────────────────────────────────────────────────
        dist = self._dist_to_target()
        if dist <= self._arrived_radius:
            self._nav_state = ARRIVED
            self._stop()
            self._publish_active(False)
            self.get_logger().info(
                f'Arrived at charging station {self._target_station} '
                f'(dist={dist:.2f}m) — holding position for charging.'
            )
            return

        # ── Navigate: TURNING then DRIVING ────────────────────────────────────
        ang_err = self._angle_to_target()
        cmd     = Twist()

        if self._nav_state == TURNING:
            cmd.linear.x  = 0.0
            cmd.angular.z = max(-self._max_angular,
                                min(self._max_angular, self._k_angular * ang_err))
            if abs(ang_err) < self._turn_only_angle:
                self._nav_state = DRIVING
                self.get_logger().info('Aligned — switching to DRIVING.')

        elif self._nav_state == DRIVING:
            if abs(ang_err) > self._drive_angle:
                self._nav_state = TURNING
                cmd.linear.x  = 0.0
                cmd.angular.z = max(-self._max_angular,
                                    min(self._max_angular, self._k_angular * ang_err))
            else:
                cmd.angular.z = max(-self._max_angular,
                                    min(self._max_angular, self._k_angular * ang_err))
                cmd.linear.x  = max(0.0, min(self._max_linear,
                                             self._k_linear * dist))

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