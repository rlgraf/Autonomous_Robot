#!/usr/bin/env python3
###############################################################################
# Battery State Node
#
# Tracks battery discharge/charge and publishes:
#   - /battery_status    (sensor_msgs/BatteryState)
#   - /battery_charging  (std_msgs/Bool)
#   - /battery_depleted  (std_msgs/Bool)
###############################################################################

import math
import os
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


DEFAULTS = {
    'battery_capacity_ah': 0.5,
    'gain_multiplier': 1.0,
    'base_idle_drain_a': 0.4,
    'base_move_drain_per_ms': 2.0,
    'base_turn_drain_per_rads': 1.0,
    'battery_voltage': 12.0,
    'publish_rate_hz': 10.0,
    'charge_rate_a': 4.0,
    'charging_radius': 0.5,
    'stationary_thresh': 0.05,
    'charging_station_x': [-20.0, 20.0],
    'charging_station_y': [0.0, 0.0],

    # vacant = yellow
    'color_idle.ambient': [1.0, 1.0, 0.0, 1.0],
    'color_idle.diffuse': [1.0, 1.0, 0.0, 1.0],
    'color_idle.specular': [0.2, 0.2, 0.0, 1.0],

    # occupied and charging = blue
    'color_charging.ambient': [0.0, 0.0, 1.0, 1.0],
    'color_charging.diffuse': [0.0, 0.0, 1.0, 1.0],
    'color_charging.specular': [0.2, 0.2, 0.2, 1.0],

    # occupied and full = green
    'color_full.ambient': [0.0, 1.0, 0.0, 1.0],
    'color_full.diffuse': [0.0, 1.0, 0.0, 1.0],
    'color_full.specular': [0.2, 0.2, 0.2, 1.0],
}


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)

        self._capacity = float(self.get_parameter('battery_capacity_ah').value)
        self._gain = float(self.get_parameter('gain_multiplier').value)

        self._idle_drain = float(self.get_parameter('base_idle_drain_a').value)
        self._move_drain = float(self.get_parameter('base_move_drain_per_ms').value) * self._gain
        self._turn_drain = float(self.get_parameter('base_turn_drain_per_rads').value) * self._gain

        self._voltage = float(self.get_parameter('battery_voltage').value)
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._charge_rate = float(self.get_parameter('charge_rate_a').value)
        self._chg_radius = float(self.get_parameter('charging_radius').value)
        self._stat_thresh = float(self.get_parameter('stationary_thresh').value)

        xs = list(self.get_parameter('charging_station_x').value)
        ys = list(self.get_parameter('charging_station_y').value)

        if len(xs) != len(ys):
            raise ValueError('charging_station_x and charging_station_y must have the same length')

        self._stations = [(float(x), float(y)) for x, y in zip(xs, ys)]

        self._colors = {
            'idle': {
                'ambient': list(self.get_parameter('color_idle.ambient').value),
                'diffuse': list(self.get_parameter('color_idle.diffuse').value),
                'specular': list(self.get_parameter('color_idle.specular').value),
            },
            'charging': {
                'ambient': list(self.get_parameter('color_charging.ambient').value),
                'diffuse': list(self.get_parameter('color_charging.diffuse').value),
                'specular': list(self.get_parameter('color_charging.specular').value),
            },
            'full': {
                'ambient': list(self.get_parameter('color_full.ambient').value),
                'diffuse': list(self.get_parameter('color_full.diffuse').value),
                'specular': list(self.get_parameter('color_full.specular').value),
            }
        }

        self._station_states = [None] * len(self._stations)
        self._station_ids = {}

        self._charge_ah = self._capacity
        self._linear_vel = 0.0
        self._angular_vel = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._last_time = self.get_clock().now()
        self._charging = False
        self._battery_depleted = False

        self.create_subscription(Odometry, '/odom_gt', self._odom_callback, 10)

        self._pub = self.create_publisher(BatteryState, '/battery_status', 10)
        self._charging_pub = self.create_publisher(Bool, '/battery_charging', 10)
        self._depleted_pub = self.create_publisher(Bool, '/battery_depleted', 10)

        self._init_timer = self.create_timer(1.0, self._lookup_station_ids)
        self.create_timer(1.0 / self._rate_hz, self._update_battery)

        self.get_logger().info('Battery node started.')
        self.get_logger().info(
            f'Capacity={self._capacity:.3f} Ah | gain={self._gain:.2f} | '
            f'idle={self._idle_drain:.3f} A | '
            f'move={self._move_drain:.3f} A/(m/s) | '
            f'turn={self._turn_drain:.3f} A/(rad/s)'
        )
        self.get_logger().info(
            'Charging stations: ' + ', '.join(f'({x}, {y})' for x, y in self._stations)
        )
        self.get_logger().warn(f'battery_node pid={os.getpid()}')
        self.get_logger().warn(
            f'BATTERY NODE INIT | charge_ah={self._charge_ah:.3f} | '
            f'percent={100.0 * self._charge_ah / self._capacity:.1f}%'
        )

    def _odom_callback(self, msg: Odometry):
        self._pos_x = float(msg.pose.pose.position.x)
        self._pos_y = float(msg.pose.pose.position.y)
        self._linear_vel = abs(float(msg.twist.twist.linear.x))
        self._angular_vel = abs(float(msg.twist.twist.angular.z))

    def _at_charging_station(self) -> bool:
        for sx, sy in self._stations:
            if math.hypot(self._pos_x - sx, self._pos_y - sy) <= self._chg_radius:
                return True
        return False

    def _is_stationary(self) -> bool:
        speed = math.hypot(self._linear_vel, self._angular_vel)
        return speed < self._stat_thresh

    def _scene_info_text(self):
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
        return result.stdout

    def _extract_station_block(self, scene_text: str, station_name: str):
        marker = f'name: "{station_name}"'
        start = scene_text.find(marker)
        if start == -1:
            return None

        next_model = scene_text.find('\nmodel {', start + len(marker))
        if next_model == -1:
            return scene_text[start:]
        return scene_text[start:next_model]

    def _parse_first_visual_ids(self, text: str):
        visual_idx = text.find('visual {')
        if visual_idx == -1:
            return None, None

        visual_text = text[visual_idx:visual_idx + 500]
        visual_id = self._parse_id(visual_text, 'id:')
        parent_id = self._parse_id(visual_text, 'parent_id:')
        return visual_id, parent_id

    def _lookup_station_ids(self):
        try:
            scene_text = self._scene_info_text()
            all_found = True

            for i in range(len(self._stations)):
                station_name = f'charging_station_{i}'
                block = self._extract_station_block(scene_text, station_name)
                if block is None:
                    all_found = False
                    continue

                visual_id, parent_id = self._parse_first_visual_ids(block)
                if visual_id is None or parent_id is None:
                    all_found = False
                    continue

                old = self._station_ids.get(i)
                new = (visual_id, parent_id)

                if old != new:
                    self._station_ids[i] = new
                    self.get_logger().info(
                        f'Station {i}: visual_id={visual_id} parent_id={parent_id}'
                    )

            if len(self._station_ids) != len(self._stations):
                all_found = False

            if all_found:
                self.get_logger().info('All charging station visual IDs resolved.')
                try:
                    self._init_timer.cancel()
                except Exception:
                    pass

                for i in range(len(self._stations)):
                    self._station_states[i] = None
                self._update_station_colors()

        except Exception as e:
            self.get_logger().warn(f'Failed to look up station IDs: {e}')

    def _parse_id(self, text: str, field: str):
        idx = text.find(field)
        if idx == -1:
            return None
        after = text[idx + len(field):].strip()
        token = after.split()[0]
        try:
            return int(token)
        except ValueError:
            return None

    def _update_station_colors(self):
        full_thresh = 0.95 * self._capacity

        for i, (sx, sy) in enumerate(self._stations):
            dist = math.hypot(self._pos_x - sx, self._pos_y - sy)
            occupied = dist <= self._chg_radius

            if occupied and self._charging:
                new_state = 'charging'
            elif occupied and self._charge_ah >= full_thresh:
                new_state = 'full'
            else:
                new_state = 'idle'

            if new_state != self._station_states[i]:
                ok = self._set_station_color(i, new_state)
                if ok:
                    self._station_states[i] = new_state

    def _set_station_color(self, idx: int, state: str) -> bool:
        if idx not in self._station_ids:
            return False

        visual_id, parent_id = self._station_ids[idx]
        col = self._colors[state]
        am = col['ambient']
        di = col['diffuse']
        sp = col['specular']

        req = (
            f'id: {visual_id} '
            f'parent_id: {parent_id} '
            f'material {{ '
            f'ambient {{ r: {am[0]} g: {am[1]} b: {am[2]} a: {am[3]} }} '
            f'diffuse {{ r: {di[0]} g: {di[1]} b: {di[2]} a: {di[3]} }} '
            f'specular {{ r: {sp[0]} g: {sp[1]} b: {sp[2]} a: {sp[3]} }} '
            f'}}'
        )

        try:
            result = subprocess.run(
                [
                    'gz', 'service',
                    '-s', '/world/arena2/visual_config',
                    '--reqtype', 'gz.msgs.Visual',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000',
                    '--req', req
                ],
                timeout=3.0,
                capture_output=True,
                text=True
            )

            stdout = result.stdout or ''
            stderr = result.stderr or ''

            if result.returncode != 0 or 'data: true' not in stdout:
                self.get_logger().warn(
                    f'visual_config failed for station {idx} state={state}; refreshing IDs'
                )
                self._station_ids.pop(idx, None)
                self._lookup_station_ids()
                return False

            return True

        except Exception as e:
            self.get_logger().warn(f'visual_config call failed: {e}')
            self._station_ids.pop(idx, None)
            return False

    def _update_battery(self):
        now = self.get_clock().now()
        dt_sec = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt_sec < 0.0 or dt_sec > 2.0:
            self.get_logger().warn(
                f'Abnormal dt_sec={dt_sec:.3f}; clamping integration step.'
            )
            dt_sec = min(max(dt_sec, 0.0), 2.0)

        total_current_a = 0.0

        at_station = self._at_charging_station()
        stationary = self._is_stationary()
        full = self._charge_ah >= self._capacity

        if at_station and stationary and not full and not self._charging:
            self._charging = True
            self.get_logger().warn(
                f'CHARGING START | charge_ah={self._charge_ah:.3f} | '
                f'percent={100.0 * self._charge_ah / self._capacity:.1f}%'
            )

        if self._charging and (not at_station or not stationary or full):
            self._charging = False
            self.get_logger().warn(
                f'CHARGING STOP | charge_ah={self._charge_ah:.3f} | '
                f'percent={100.0 * self._charge_ah / self._capacity:.1f}%'
            )
            if full:
                self.get_logger().info('Charging complete - battery at 100%.')

        if self._charging:
            self._charge_ah += self._charge_rate * (dt_sec / 3600.0)
            self._charge_ah = min(self._capacity, self._charge_ah)
            percentage = self._charge_ah / self._capacity

            self.get_logger().info(
                f'Charging: {percentage * 100:.1f}%',
                throttle_duration_sec=5.0
            )
        else:
            movement_drain = (
                self._linear_vel * self._move_drain +
                self._angular_vel * self._turn_drain
            )
            total_current_a = self._idle_drain + movement_drain

            self._charge_ah -= total_current_a * (dt_sec / 3600.0)
            self._charge_ah = max(0.0, self._charge_ah)
            percentage = self._charge_ah / self._capacity

            if percentage <= 0.0:
                if not self._battery_depleted:
                    self.get_logger().error('Battery depleted - publishing /battery_depleted=True.')
                self._battery_depleted = True
            else:
                self._battery_depleted = False

            if 0.0 < percentage < 0.2:
                self.get_logger().warn(
                    f'Low battery: {percentage * 100:.1f}% | {self._charge_ah:.3f} Ah remaining',
                    throttle_duration_sec=10.0
                )

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.voltage = self._voltage
        msg.current = self._charge_rate if self._charging else -total_current_a
        msg.charge = self._charge_ah
        msg.capacity = self._capacity
        msg.percentage = percentage
        msg.present = True

        self._update_station_colors()
        self._pub.publish(msg)
        self._charging_pub.publish(Bool(data=self._charging))
        self._depleted_pub.publish(Bool(data=self._battery_depleted))


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