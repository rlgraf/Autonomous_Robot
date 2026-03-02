#!/usr/bin/env python3
###############################################################################
# Battery State Node
#
# Tracks battery discharge/charge and publishes:
#   - /battery_status    (sensor_msgs/BatteryState)
#   - /battery_charging  (std_msgs/Bool)   True while charging is active
#   - /battery_depleted  (std_msgs/Bool)   True when percentage == 0
#
# IMPORTANT CHANGE:
#   - This node NO LONGER publishes /cmd_vel (supervisor is the sole owner).
###############################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import yaml
from ament_index_python.packages import get_package_share_directory
import subprocess
import os
import math

DEFAULTS = {
    'battery_capacity_ah' : 1.0,
    'idle_drain_a'        : 1.0,
    'move_drain_per_ms'   : 10.0,
    'turn_drain_per_rads' : 5.0,
    'battery_voltage'     : 12.0,
    'publish_rate_hz'     : 10.0,
    'charge_rate_a'       : 2.0,
    'charging_radius'     : 0.5,
    'stationary_thresh'   : 0.05,
    # 'charging_stations'   : [0.0, 0.0],
}


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        # Declare + load all parameters from YAML
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)

        self._capacity    = self.get_parameter('battery_capacity_ah').value
        self._idle_drain  = self.get_parameter('idle_drain_a').value
        self._move_drain  = self.get_parameter('move_drain_per_ms').value
        self._turn_drain  = self.get_parameter('turn_drain_per_rads').value
        self._voltage     = self.get_parameter('battery_voltage').value
        self._rate_hz     = self.get_parameter('publish_rate_hz').value
        self._charge_rate = self.get_parameter('charge_rate_a').value
        self._chg_radius  = self.get_parameter('charging_radius').value
        self._stat_thresh = self.get_parameter('stationary_thresh').value

        params_file = os.path.join(
            get_package_share_directory('mobile_robot'),
            'parameters', 'battery_tunable_parameters.yaml'
        )
        with open(params_file, 'r') as f:
            raw = yaml.safe_load(f)
        ros_params = raw['battery_node']['ros__parameters']

        self._stations = [tuple(s) for s in ros_params['charging_stations']]

        # Station colors from YAML
        self._colors = {
            'idle':     ros_params['color_idle'],
            'charging': ros_params['color_charging'],
            'full':     ros_params['color_full'],
        }

        self._station_states = [None] * len(self._stations)

        # Look up visual/parent IDs from Gazebo scene at startup
        self._station_ids = {}   # idx -> (visual_id, parent_id)
        self._init_timer = self.create_timer(0.0, self._lookup_station_ids)

        # State
        self._charge_ah  = self._capacity  # current charge in Ah
        self._linear_vel = 0.0
        self._angular_vel = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._last_time = self.get_clock().now()
        self._charging = False
        self._gz_voltage = self._voltage

        # Latches (published as Bool topics)
        self._battery_depleted = False

        # Subscribe to odometry (ground truth) to get current velocity and position
        self.create_subscription(Odometry, 'odom_gt', self._odom_callback, 10)

        # Subscribe to Gazebo battery topic (bridged) for voltage reference
        self.create_subscription(BatteryState, 'battery_state', self._gz_battery_callback, 10)

        # Publishers
        self._pub = self.create_publisher(BatteryState, 'battery_status', 10)
        self._charging_pub = self.create_publisher(Bool, 'battery_charging', 10)
        self._depleted_pub = self.create_publisher(Bool, 'battery_depleted', 10)

        # Timer to update and publish at fixed rate
        self.create_timer(1.0 / self._rate_hz, self._update_battery)

        self.get_logger().info('Battery node started (no /cmd_vel publishing).')
        self.get_logger().info(
            f'Capacity: {self._capacity} Ah | Idle drain: {self._idle_drain:.3f} A | '
            f'Movement drain: {self._move_drain} A/(m/s) linear, {self._turn_drain} A/(rad/s) angular'
        )
        self.get_logger().info(
            f'Charging stations: ' + ', '.join(f'({x},{y})' for x, y in self._stations)
        )

    def _odom_callback(self, msg: Odometry):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y
        self._linear_vel = abs(msg.twist.twist.linear.x)
        self._angular_vel = abs(msg.twist.twist.angular.z)

    def _gz_battery_callback(self, msg: BatteryState):
        self._gz_voltage = msg.voltage

    # -- Helpers -----#
    def _at_charging_station(self) -> bool:
        for sx, sy in self._stations:
            dist = math.sqrt((self._pos_x - sx) ** 2 + (self._pos_y - sy) ** 2)
            if dist <= self._chg_radius:
                return True
        return False

    def _is_stationary(self) -> bool:
        speed = math.sqrt(self._linear_vel ** 2 + self._angular_vel ** 2)
        return speed < self._stat_thresh

    def _lookup_station_ids(self):
        """Query Gazebo scene once to get visual and parent IDs for each station."""
        try:
            result = subprocess.run(
                [
                    'gz', 'service',
                    '-s', '/world/arena2/scene/info',
                    '--reqtype', 'gz.msgs.Empty',
                    '--reptype', 'gz.msgs.Scene',
                    '--timeout', '2000',
                    '--req', ''
                ],
                capture_output=True, text=True, timeout=5.0
            )
            scene_text = result.stdout

            for i in range(len(self._stations)):
                name = f'charging_station_{i}'
                start = scene_text.find(f'name: "{name}"')
                if start == -1:
                    self.get_logger().warn(f'Could not find {name} in scene')
                    continue
                block = scene_text[start:start + 500]

                visual_start = block.find('visual {')
                if visual_start == -1:
                    continue
                visual_block = block[visual_start:visual_start + 200]

                visual_id = self._parse_id(visual_block, 'id:')
                parent_id = self._parse_id(visual_block, 'parent_id:')

                if visual_id and parent_id:
                    self._station_ids[i] = (visual_id, parent_id)
                    self.get_logger().info(
                        f'Station {i}: visual_id={visual_id} parent_id={parent_id}'
                    )

        except Exception as e:
            self.get_logger().warn(f'Failed to look up station IDs: {e}')

        # One-shot — cancel after first run
        self._init_timer.cancel()

        # Initialize idle colors
        for i in range(len(self._stations)):
            self._set_station_color(i, 'idle')

    def _parse_id(self, text: str, field: str) -> int | None:
        idx = text.find(field)
        if idx == -1:
            return None
        after = text[idx + len(field):].strip()
        token = after.split()[0]
        try:
            return int(token)
        except ValueError:
            return None

    # ── Charging station color management ──────────────────────────────── #
    def _update_station_colors(self):
        for i, (sx, sy) in enumerate(self._stations):
            dist = math.sqrt((self._pos_x - sx) ** 2 + (self._pos_y - sy) ** 2)
            at_this = dist <= self._chg_radius

            if at_this and self._charging:
                new_state = 'charging'
            elif at_this and self._charge_ah >= self._capacity * 0.95:
                new_state = 'full'
            else:
                new_state = 'idle'

            if new_state != self._station_states[i]:
                self._station_states[i] = new_state
                self._set_station_color(i, new_state)

    def _set_station_color(self, idx: int, state: str):
        if idx not in self._station_ids:
            self.get_logger().warn(f'No IDs for station {idx} yet — skipping color update', once=True)
            return

        visual_id, parent_id = self._station_ids[idx]
        col = self._colors[state]
        am = col['ambient']
        di = col['diffuse']
        sp = col['specular']

        req = (
            f'id: {visual_id} '
            f'parent_id: {parent_id} '
            f'material {{ '
            f'  ambient  {{ r: {am[0]} g: {am[1]} b: {am[2]} a: {am[3]} }} '
            f'  diffuse  {{ r: {di[0]} g: {di[1]} b: {di[2]} a: {di[3]} }} '
            f'  specular {{ r: {sp[0]} g: {sp[1]} b: {sp[2]} a: {sp[3]} }} '
            f'}}'
        )
        try:
            subprocess.run(
                [
                    'gz', 'service',
                    '-s', '/world/arena2/visual_config',
                    '--reqtype', 'gz.msgs.Visual',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000',
                    '--req', req
                ],
                timeout=3.0, capture_output=True
            )
        except Exception as e:
            self.get_logger().warn(f'visual_config call failed: {e}', throttle_duration_sec=10.0)

    # --- Main Loop --#
    def _update_battery(self):
        now = self.get_clock().now()
        dt_sec = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        total_current_a = 0.0

        at_station = self._at_charging_station()
        stationary = self._is_stationary()
        full = self._charge_ah >= self._capacity

        # Charging state machine (same logic as your current version)
        if at_station and stationary and not full:
            self._charging = True

        if self._charging and (not at_station or not stationary or full):
            self._charging = False
            if full:
                self.get_logger().info('Charging complete — battery at 100%.')

        if self._charging:
            self._charge_ah += self._charge_rate * (dt_sec / 3600.0)
            self._charge_ah = min(self._capacity, self._charge_ah)
            percentage = self._charge_ah / self._capacity
            self.get_logger().info(f'Charging: {percentage * 100:.1f}%', throttle_duration_sec=5.0)
        else:
            movement_drain = (self._linear_vel * self._move_drain + self._angular_vel * self._turn_drain)
            total_current_a = self._idle_drain + movement_drain

            self._charge_ah -= total_current_a * (dt_sec / 3600.0)
            self._charge_ah = max(0.0, self._charge_ah)
            percentage = self._charge_ah / self._capacity

            if percentage <= 0.0:
                if not self._battery_depleted:
                    self.get_logger().error('Battery depleted — publishing /battery_depleted=True.')
                self._battery_depleted = True
            else:
                self._battery_depleted = False

            if 0.0 < percentage < 0.2:
                self.get_logger().warn(
                    f'Low battery: {percentage * 100:.1f}% | {self._charge_ah:.3f} Ah remaining',
                    throttle_duration_sec=10.0
                )

        # Publish BatteryState
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_footprint'
        # msg.voltage = self._gz_voltage  # optional
        msg.current = self._charge_rate if self._charging else -(total_current_a)
        msg.charge = self._charge_ah
        msg.capacity = self._capacity
        msg.percentage = percentage
        msg.present = True

        self._update_station_colors()
        self._pub.publish(msg)

        # Publish simple status flags (for supervisor / higher-level logic)
        chg = Bool()
        chg.data = self._charging
        self._charging_pub.publish(chg)

        dep = Bool()
        dep.data = self._battery_depleted
        self._depleted_pub.publish(dep)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()