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
import math

# --- Tunable parameters ---
BATTERY_CAPACITY_AH   = 1.0   # Total capacity in Ah
IDLE_DRAIN_A          = 0.25 # Idle current draw in A (2W / 12V)
MOVE_DRAIN_PER_MS     = 10    # Extra current per m/s of linear speed (A)
TURN_DRAIN_PER_RADS   = 5    # Extra current per rad/s of angular speed (A)
BATTERY_VOLTAGE       = 12.0   # Nominal voltage in V
PUBLISH_RATE_HZ       = 10.0   # How often to publish battery state


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        # State
        self._charge_ah       = BATTERY_CAPACITY_AH  # current charge in Ah
        self._linear_vel      = 0.0
        self._angular_vel     = 0.0
        self._last_time       = self.get_clock().now()

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

        # Timer to update and publish at fixed rate
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._update_battery)

        # Voltage from Gazebo (fallback to nominal)
        self._gz_voltage = BATTERY_VOLTAGE

        self.get_logger().info('Battery node started.')
        self.get_logger().info(
            f'Idle drain: {IDLE_DRAIN_A:.3f} A | '
            f'Movement drain: {MOVE_DRAIN_PER_MS} A/(m/s) linear, '
            f'{TURN_DRAIN_PER_RADS} A/(rad/s) angular'
        )

    def _odom_callback(self, msg: Odometry):
        self._linear_vel  = abs(msg.twist.twist.linear.x)
        self._angular_vel = abs(msg.twist.twist.angular.z)

    def _gz_battery_callback(self, msg: BatteryState):
        # Use Gazebo's voltage calculation (more accurate)
        self._gz_voltage = msg.voltage

    def _update_battery(self):
        now = self.get_clock().now()
        dt_sec = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        # Total current draw:
        # idle (constant) + linear movement + angular movement
        movement_drain = (
            self._linear_vel  * MOVE_DRAIN_PER_MS +
            self._angular_vel * TURN_DRAIN_PER_RADS
        )
        total_current_a = IDLE_DRAIN_A + movement_drain

        # Discharge: I * dt in Ah
        self._charge_ah -= total_current_a * (dt_sec / 3600.0)
        self._charge_ah  = max(0.0, self._charge_ah)

        # Percentage
        percentage = self._charge_ah / BATTERY_CAPACITY_AH

        # Build and publish BatteryState message
        msg = BatteryState()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = 'base_footprint'
        # msg.voltage         = self._gz_voltage
        msg.current         = -total_current_a   # negative = discharging
        msg.charge          = self._charge_ah
        msg.capacity        = BATTERY_CAPACITY_AH
        # msg.design_capacity = BATTERY_CAPACITY_AH
        msg.percentage      = percentage
        # msg.linear_vel      = self._linear_vel
        # msg.power_supply_status   = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        # msg.power_supply_health   = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        # msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present         = True

        self._pub.publish(msg)

        # Warn at low battery
        if percentage < 0.2:
            self.get_logger().warn(
                f'Low battery: {percentage*100:.1f}% | '
                f'{self._charge_ah:.3f} Ah remaining',
                throttle_duration_sec=10.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()