###############################################################################
# Battery State Node
# Tracks battery discharge:
#   - Idle drain: constant, handled by Gazebo LinearBattery plugin
#   - Movement drain: proportional to linear + angular velocity
###############################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml
from ament_index_python.packages import get_package_share_directory
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
        # raw_stations      = self.get_parameter('charging_stations').value
        params_file = os.path.join(
            get_package_share_directory('mobile_robot'),
            'parameters', 'battery_tunable_parameters.yaml'
        )
        with open(params_file, 'r') as f:
            raw = yaml.safe_load(f)
        self._stations = [
            tuple(s) for s in raw['battery_node']['ros__parameters']['charging_stations']
        ]

        # State
        self._charge_ah       = self._capacity  # current charge in Ah
        self._linear_vel      = 0.0
        self._angular_vel     = 0.0
        self._pos_x           = 0.0
        self._pos_y           = 0.0
        self._last_time       = self.get_clock().now()
        self._charging        = False
        # in __init__, add with the other state variables:
        self._gz_voltage  = self._voltage       

        # Subscribe to odometry to get current velocity
        self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
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

        # Timer to update and publish at fixed rate
        self.create_timer(1.0 / self._rate_hz, self._update_battery)



        self.get_logger().info('Battery node started.')
        self.get_logger().info(
            f'Capacity: {self._capacity} Ah | Idle drain: {self._idle_drain:.3f} A | '
            f'Movement drain: {self._move_drain} A/(m/s) linear, {self._turn_drain} A/(rad/s) angular'
        )
        self.get_logger().info(
            f'Charging stations: ' + ', '.join(f'({x},{y})' for x, y in self._stations)
        )

    def _odom_callback(self, msg: Odometry):
        self._pos_x       = msg.pose.pose.position.x
        self._pos_y       = msg.pose.pose.position.y
        self._linear_vel  = abs(msg.twist.twist.linear.x)
        self._angular_vel = abs(msg.twist.twist.angular.z)

    def _gz_battery_callback(self, msg: BatteryState):
        # Use Gazebo's voltage calculation (more accurate)
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
    

    # --- Main Loop --#


    def _update_battery(self):
        now = self.get_clock().now()
        dt_sec = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        total_current_a = 0.0    # ← add this default

        at_station = self._at_charging_station()
        stationary = self._is_stationary()
        full       = self._charge_ah >= self._capacity


        # Enter charging: at a station, not moving, battery not full
        if at_station and stationary and not full:
            self._charging = True

        # Leave charging: moved away, started moving, or now full
        if self._charging and (not at_station or not stationary or full):
            self._charging = False
            if full:
                self.get_logger().info('Charging complete — battery at 100%.')

        if self._charging:
            # Charge up and block movement
            self._charge_ah += self._charge_rate * (dt_sec / 3600.0)
            self._charge_ah  = min(self._capacity, self._charge_ah)
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
                self._linear_vel  * self._move_drain +
                self._angular_vel * self._turn_drain
            )
            total_current_a = self._idle_drain + movement_drain

            # Discharge: I * dt in Ah
            self._charge_ah -= total_current_a * (dt_sec / 3600.0)
            self._charge_ah  = max(0.0, self._charge_ah)

            # Percentage
            percentage = self._charge_ah / self._capacity

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
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = 'base_footprint'
        # msg.voltage         = self._gz_voltage
        # msg.charging        = self._charging
        msg.current         = self._charge_rate if self._charging else -(total_current_a)
        msg.charge          = self._charge_ah
        msg.capacity        = self._capacity
        # msg.design_capacity = BATTERY_CAPACITY_AH
        msg.percentage      = percentage
        # msg.power_supply_status   = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        # msg.power_supply_health   = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        # msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present         = True

        self._pub.publish(msg)

        

        

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()