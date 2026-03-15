#!/usr/bin/env python3
###############################################################################
# Battery State Node
# Tracks battery discharge:
#   - Idle drain: constant, handled by Gazebo LinearBattery plugin
#   - Movement drain: proportional to linear + angular velocity
###############################################################################

import math
import os
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool

DEFAULTS = {
    'battery_capacity_ah': 1.0,
    'idle_drain_a': 2.0,
    'move_drain_per_ms': 20.0,
    'turn_drain_per_rads': 10.0,
    'battery_voltage': 12.0,
    'publish_rate_hz': 10.0,
    'charge_rate_a': 40.0,
    'charging_radius': 0.75,
    'stationary_thresh': 0.1,
    'initial_battery_fraction': 1.0,  # Start at 100% (1.0) by default, can set to 0.5 for 50%, etc.
}


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        # Declare + load all parameters from DEFAULTS
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)

        self._capacity = float(self.get_parameter('battery_capacity_ah').value)
        self._idle_drain = float(self.get_parameter('idle_drain_a').value)
        self._move_drain = float(self.get_parameter('move_drain_per_ms').value)
        self._turn_drain = float(self.get_parameter('turn_drain_per_rads').value)
        self._voltage = float(self.get_parameter('battery_voltage').value)
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._charge_rate = float(self.get_parameter('charge_rate_a').value)
        self._chg_radius = float(self.get_parameter('charging_radius').value)
        self._stat_thresh = float(self.get_parameter('stationary_thresh').value)
        self._initial_battery_fraction = float(self.get_parameter('initial_battery_fraction').value)

        # Load stations and colors from YAML file
        params_file = os.path.join(
            get_package_share_directory('mobile_robot'),
            'parameters', 'battery_tunable_parameters.yaml'
        )
        with open(params_file, 'r') as f:
            raw = yaml.safe_load(f)
        ros_params = raw['battery_node']['ros__parameters']

        # Handle charging_stations - convert from x/y arrays to list of tuples
        xs = ros_params.get('charging_station_x', [0.0])
        ys = ros_params.get('charging_station_y', [0.0])
        if isinstance(xs, list) and isinstance(ys, list):
            self._stations = [(float(x), float(y)) for x, y in zip(xs, ys)]
        else:
            # Fallback: try charging_stations as list of lists
            stations_raw = ros_params.get('charging_stations', [[0.0, 0.0]])
            self._stations = [tuple(s) for s in stations_raw]

        # Load station colors from YAML
        # Each color is a dict with ambient, diffuse, specular — each a [r,g,b,a] list
        self._colors = {
            'idle': {
                'ambient': ros_params.get('color_idle.ambient', [1.0, 1.0, 0.0, 1.0]),
                'diffuse': ros_params.get('color_idle.diffuse', [1.0, 1.0, 0.0, 1.0]),
                'specular': ros_params.get('color_idle.specular', [0.2, 0.2, 0.0, 1.0]),
            },
            'charging': {
                'ambient': ros_params.get('color_charging.ambient', [0.0, 0.0, 1.0, 1.0]),
                'diffuse': ros_params.get('color_charging.diffuse', [0.0, 0.0, 1.0, 1.0]),
                'specular': ros_params.get('color_charging.specular', [0.2, 0.2, 0.2, 1.0]),
            },
            'full': {
                'ambient': ros_params.get('color_full.ambient', [0.0, 1.0, 0.0, 1.0]),
                'diffuse': ros_params.get('color_full.diffuse', [0.0, 1.0, 0.0, 1.0]),
                'specular': ros_params.get('color_full.specular', [0.2, 0.2, 0.2, 1.0]),
            }
        }

        # Track current visual state per station to avoid redundant marker publishes
        self._station_states = [None] * len(self._stations)

        # Look up visual/parent IDs from Gazebo scene at startup
        # Small delay to ensure stations are spawned before we query
        self._station_ids = {}   # idx -> (visual_id, parent_id)
        self._init_timer = self.create_timer(0.5, self._lookup_station_ids)

        # State
        # Initialize battery at specified fraction (default 1.0 = 100%)
        self._charge_ah = self._capacity * self._initial_battery_fraction
        self._linear_vel = 0.0
        self._angular_vel = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._last_time = self.get_clock().now()
        self._charging = False
        self._gz_voltage = self._voltage

        # extra test states
        self._pos_x_odom = 0.0
        self._pos_y_odom = 0.0
        self._world_name = None

        # Subscribe to odometry to get current velocity
        self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10
        )

        # Subscribe to ground truth odometry from Gazebo for accurate position/velocity
        self.create_subscription(
            Odometry,
            '/odom_gt',
            self._gt_odom_callback,
            10
        )

        # Subscribe to Gazebo battery topic (bridged) for voltage reference
        self.create_subscription(
            BatteryState,
            'battery_state',
            self._gz_battery_callback,
            10
        )

        # Publish our computed battery state
        self._pub = self.create_publisher(BatteryState, 'battery_status', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        # Publish low battery warning for guest interaction node
        self._low_battery_pub = self.create_publisher(Bool, '/low_battery_warning', 10)
        
        # Publish charging status for supervisor (supervisor integration)
        self._charging_pub = self.create_publisher(Bool, '/battery_charging', 10)

        # Timer to update and publish at fixed rate
        self.create_timer(1.0 / self._rate_hz, self._update_battery)

        self.get_logger().info('Battery node started (using ground truth odometry).')
        initial_pct = (self._charge_ah / self._capacity) * 100.0
        self.get_logger().info(
            f'Initial battery: {initial_pct:.1f}% ({self._charge_ah:.3f} Ah / {self._capacity:.3f} Ah)'
        )
        self.get_logger().info(
            f'Capacity: {self._capacity} Ah | Idle drain: {self._idle_drain:.3f} A | '
            f'Movement drain: {self._move_drain} A/(m/s) linear, {self._turn_drain} A/(rad/s) angular'
        )
        self.get_logger().info(
            f'Charging stations: ' + ', '.join(f'({x},{y})' for x, y in self._stations)
        )

    def _odom_callback(self, msg: Odometry):
        self._pos_x_odom = msg.pose.pose.position.x
        self._pos_y_odom = msg.pose.pose.position.y
        self._linear_vel = abs(msg.twist.twist.linear.x)
        self._angular_vel = abs(msg.twist.twist.angular.z)

    def _gt_odom_callback(self, msg: Odometry):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y

    def _gz_battery_callback(self, msg: BatteryState):
        # Use Gazebo's voltage calculation (more accurate)
        if math.isfinite(msg.voltage):
            self._gz_voltage = msg.voltage

    # -- Helpers -----#
    def _at_charging_station(self) -> bool:
        """True if robot is within CHARGING_RADIUS of any station."""
        for sx, sy in self._stations:
            dist = math.sqrt((self._pos_x - sx) ** 2 + (self._pos_y - sy) ** 2)
            if dist <= self._chg_radius:
                return True
        return False

    def _is_stationary(self) -> bool:
        """True if the robot is not meaningfully moving."""
        speed = math.sqrt(self._linear_vel ** 2 + self._angular_vel ** 2)
        return speed < self._stat_thresh

    def _get_world_name(self) -> str:
        """Manually set Gazebo world name."""
        world = "arena2"   # <-- change this to match your SDF world name
        self.get_logger().info(f'Using manual world name: "{world}"')
        return world

    def _lookup_station_ids(self):
        """Query Gazebo scene repeatedly until all station IDs are resolved."""
        self._world_name = self._get_world_name()
        if self._world_name is None:
            self.get_logger().warn('World name unknown — retrying...')
            return  # timer will retry
        try:
            result = subprocess.run(
                ['gz', 'service',
                 '-s', f'/world/{self._world_name}/scene/info',  # ← dynamic world name
                 '--reqtype', 'gz.msgs.Empty',
                 '--reptype', 'gz.msgs.Scene',
                 '--timeout', '2000',
                 '--req', ''],
                capture_output=True, text=True, timeout=5.0
            )
            scene_text = result.stdout
            for i in range(len(self._stations)):
                if i in self._station_ids:
                    continue  # already resolved, skip
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

        # Only cancel + initialize colors once ALL stations are resolved
        if len(self._station_ids) == len(self._stations):
            self._init_timer.cancel()
            self.get_logger().info('All charging station IDs resolved.')
            for i in range(len(self._stations)):
                self._set_station_color(i, 'idle')
        else:
            missing = [i for i in range(len(self._stations)) if i not in self._station_ids]
            self.get_logger().warn(f'Stations not yet found: {missing} — retrying...')

    def _parse_id(self, text: str, field: str) -> int | None:
        """Extract an integer ID field from a protobuf text block."""
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
        """Determine correct color state for each station and publish markers."""
        for i, (sx, sy) in enumerate(self._stations):
            dist = math.sqrt((self._pos_x - sx)**2 + (self._pos_y - sy)**2)
            at_this = dist <= self._chg_radius
            full = self._charge_ah >= self._capacity

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
        """Call /world/default/visual_config to change the pad color in-place."""
        if idx not in self._station_ids:
            self.get_logger().warn(
                f'No IDs for station {idx} yet — skipping color update', once=True
            )
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
                ['gz', 'service',
                 '-s', f'/world/{self._world_name}/visual_config',  # ← dynamic world name
                 '--reqtype', 'gz.msgs.Visual',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', req],
                timeout=3.0, capture_output=True
            )
        except Exception as e:
            self.get_logger().warn(
                f'visual_config call failed: {e}', throttle_duration_sec=10.0
            )

    # --- Main Loop --#

    def _update_battery(self):
        now = self.get_clock().now()
        dt_sec = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        total_current_a = 0.0    # ← add this default

        at_station = self._at_charging_station()
        stationary = self._is_stationary()
        full = self._charge_ah >= self._capacity

        # ── Charging state machine ────────────────────────────────────────────
        # Entry: must be at station AND stationary to begin
        # Exit:  ONLY when battery reaches 100% — position/motion noise ignored
        if not self._charging:
            if at_station and stationary and not full:
                self._charging = True
                self.get_logger().info('Charging started — locked until 100%.')
        else:
            # Locked in charging — only exit condition is full battery
            if full:
                self._charging = False
                self.get_logger().info('Charging complete — battery at 100%.')

        if self._charging:
            # Charge up and block movement
            self._charge_ah += self._charge_rate * (dt_sec / 3600.0)
            self._charge_ah = min(self._capacity, self._charge_ah)
            self._cmd_vel_pub.publish(Twist())    # zero velocity — can't move while charging
            percentage = self._charge_ah / self._capacity
            self.get_logger().info(
                f'Charging: {percentage * 100:.1f}%',
                throttle_duration_sec=5.0
            )

        else:
            # discharge normally

            # Total current draw:
            # idle (constant) + linear movement + angular movement
            movement_drain = (
                self._linear_vel * self._move_drain +
                self._angular_vel * self._turn_drain
            )
            total_current_a = self._idle_drain + movement_drain

            # Discharge: I * dt in Ah
            self._charge_ah -= total_current_a * (dt_sec / 3600.0)
            self._charge_ah = max(0.0, self._charge_ah)

            # Percentage
            percentage = self._charge_ah / self._capacity

            # Publish low battery warning
            low_battery_msg = Bool()
            low_battery_msg.data = percentage < 0.25  # 25% threshold
            self._low_battery_pub.publish(low_battery_msg)

            # Stop robot when battery is depleted
            if percentage <= 0.0:
                self._cmd_vel_pub.publish(Twist())   # all zeros = full stop
                self.get_logger().error(
                    'Battery depleted — robot stopped!',
                    throttle_duration_sec=5.0
                )

            # Warn at low battery
            elif percentage < 0.2:
                self.get_logger().warn(
                    f'Low battery: {percentage*100:.1f}% | '
                    f'{self._charge_ah:.3f} Ah remaining',
                    throttle_duration_sec=10.0
                )

        # Build and publish BatteryState message
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_footprint'
        # msg.voltage = self._gz_voltage
        # msg.charging = self._charging
        msg.current = self._charge_rate if self._charging else -(total_current_a)
        msg.charge = self._charge_ah
        msg.capacity = self._capacity
        # msg.design_capacity = BATTERY_CAPACITY_AH
        msg.percentage = percentage
        # msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        # msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        # msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        # verification of robot odometry drift from true
        # self.get_logger().info(f'Odometry robot x: {self._pos_x_odom}, y: {self._pos_y_odom}')
        # self.get_logger().info(f'Odometry true  x: {self._pos_x}, y: {self._pos_y}')

        # Update charging station visual colors
        self._update_station_colors()

        self._pub.publish(msg)
        
        # Publish charging status for supervisor
        charging_msg = Bool()
        charging_msg.data = self._charging
        self._charging_pub.publish(charging_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
