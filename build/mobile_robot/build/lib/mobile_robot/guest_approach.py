#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class StationaryTargetApproach(Node):
    def __init__(self):
        super().__init__("stationary_target_approach")

        # -------- Params --------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("stationary_topic", "/object/stationary")
        self.declare_parameter("range_topic", "/object/range")
        self.declare_parameter("bearing_topic", "/object/bearing")

        self.declare_parameter("stop_distance_m", 1.2)
        self.declare_parameter("forward_speed_mps", 1.0)

        # Turning control
        self.declare_parameter("turn_gain", 1.2)
        self.declare_parameter("max_turn_radps", 1.5)

        # NEW: Aim-before-move behavior
        self.declare_parameter("aim_tolerance_rad", 0.12)          # ~6.9 deg
        self.declare_parameter("aim_turn_speed_min_radps", 0.10)   # minimum turn speed magnitude
        self.declare_parameter("allow_small_steering_while_moving", False)

        # Confirm + latch behavior
        self.declare_parameter("stationary_confirm_count", 5)
        self.declare_parameter("lost_release_count", 15)     # control ticks with no target before unlatch
        self.declare_parameter("use_live_updates_while_latched", True)
        self.declare_parameter("bearing_ema_alpha", 0.25)
        self.declare_parameter("range_ema_alpha", 0.25)

        # -------- Read params --------
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.stationary_topic = self.get_parameter("stationary_topic").value
        self.range_topic = self.get_parameter("range_topic").value
        self.bearing_topic = self.get_parameter("bearing_topic").value

        self.stop_distance_m = float(self.get_parameter("stop_distance_m").value)
        self.forward_speed_mps = float(self.get_parameter("forward_speed_mps").value)

        self.turn_gain = float(self.get_parameter("turn_gain").value)
        self.max_turn_radps = float(self.get_parameter("max_turn_radps").value)

        self.aim_tolerance_rad = float(self.get_parameter("aim_tolerance_rad").value)
        self.aim_turn_speed_min_radps = float(self.get_parameter("aim_turn_speed_min_radps").value)
        self.allow_small_steering_while_moving = bool(
            self.get_parameter("allow_small_steering_while_moving").value
        )

        self.stationary_confirm_count = max(1, int(self.get_parameter("stationary_confirm_count").value))
        self.lost_release_count = max(1, int(self.get_parameter("lost_release_count").value))
        self.use_live_updates = bool(self.get_parameter("use_live_updates_while_latched").value)
        self.bearing_alpha = float(self.get_parameter("bearing_ema_alpha").value)
        self.range_alpha = float(self.get_parameter("range_ema_alpha").value)

        # -------- State --------
        self._stationary_streak = 0
        self._stationary_confirmed = False

        self.range_m = float("nan")
        self.bearing_rad = float("nan")

        # Latch state
        self._latched = False
        self._latched_range = float("nan")
        self._latched_bearing = float("nan")
        self._lost_streak = 0

        # -------- Pub/Sub --------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Bool, self.stationary_topic, self.on_stationary, qos_profile_sensor_data)
        self.create_subscription(Float32, self.range_topic, self.on_range, qos_profile_sensor_data)
        self.create_subscription(Float32, self.bearing_topic, self.on_bearing, qos_profile_sensor_data)

        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def on_stationary(self, msg: Bool):
        if bool(msg.data):
            self._stationary_streak += 1
        else:
            self._stationary_streak = 0

        self._stationary_confirmed = (self._stationary_streak >= self.stationary_confirm_count)

        # Latch as soon as stationary is confirmed AND we have good measurements
        if (not self._latched) and self._stationary_confirmed and self._good_measurements():
            self._latched = True
            self._latched_range = self.range_m
            self._latched_bearing = self.bearing_rad
            self._lost_streak = 0
            self.get_logger().info("Latched target.")

    def on_range(self, msg: Float32):
        self.range_m = float(msg.data)

    def on_bearing(self, msg: Float32):
        self.bearing_rad = float(msg.data)

    def _good_measurements(self) -> bool:
        return math.isfinite(self.range_m) and math.isfinite(self.bearing_rad)

    def _update_latched_estimate(self):
        """Optional: keep target estimate current while latched (smoothed)."""
        if not self.use_live_updates:
            return
        if not self._good_measurements():
            return

        a_r = clamp(self.range_alpha, 0.0, 1.0)
        a_b = clamp(self.bearing_alpha, 0.0, 1.0)

        self._latched_range = (1.0 - a_r) * self._latched_range + a_r * self.range_m
        self._latched_bearing = (1.0 - a_b) * self._latched_bearing + a_b * self.bearing_rad

    def _unlatch(self, reason: str):
        if self._latched:
            self.get_logger().info(f"Unlatched target ({reason}).")
        self._latched = False
        self._latched_range = float("nan")
        self._latched_bearing = float("nan")
        self._lost_streak = 0
        self._stationary_streak = 0
        self._stationary_confirmed = False

    def stop(self):
        self.cmd_pub.publish(Twist())

    def _turn_in_place_cmd(self, bearing: float) -> Twist:
        """Create a Twist that only rotates to reduce bearing."""
        cmd = Twist()

        # Basic P controller
        w = clamp(self.turn_gain * bearing, -self.max_turn_radps, self.max_turn_radps)

        # Ensure enough torque to actually rotate (optional but helpful on real robots)
        if abs(w) > 1e-6:
            w = math.copysign(max(abs(w), self.aim_turn_speed_min_radps), w)

        cmd.linear.x = 0.0
        cmd.angular.z = float(w)
        return cmd

    def control_loop(self):
        # Default: stop
        cmd = Twist()

        if not self._latched:
            self.cmd_pub.publish(cmd)
            return

        # Latched: check whether data is still usable
        if self._good_measurements():
            self._lost_streak = 0
            self._update_latched_estimate()
        else:
            self._lost_streak += 1
            if self._lost_streak >= self.lost_release_count:
                self._unlatch("lost target")
                self.cmd_pub.publish(Twist())
                return

        # Use latched values for control
        r = self._latched_range
        b = self._latched_bearing

        # If close enough, stop + unlatch
        if (not math.isfinite(r)) or (not math.isfinite(b)) or (r <= self.stop_distance_m):
            self._unlatch("reached stop distance")
            self.cmd_pub.publish(Twist())
            return

        # ---------- NEW BEHAVIOR ----------
        # Aim first: if not aligned, rotate in place only
        if abs(b) > self.aim_tolerance_rad:
            cmd = self._turn_in_place_cmd(b)
            self.cmd_pub.publish(cmd)
            return

        # Aligned: move straight toward target
        cmd.linear.x = float(self.forward_speed_mps)

        if self.allow_small_steering_while_moving:
            # Optional: small steering correction while driving
            w = clamp(self.turn_gain * b, -self.max_turn_radps, self.max_turn_radps)
            cmd.angular.z = float(w)
        else:
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = StationaryTargetApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
