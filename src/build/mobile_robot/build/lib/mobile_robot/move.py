#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class OrientToStationary(Node):
    def __init__(self):
        super().__init__('move')

        # Inputs
        self.create_subscription(Bool, '/object_stationary', self.stationary_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Output
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.stationary = False
        self.prev_stationary = False
        self.have_scan = False
        self.last_min_angle = None
        self.last_min_range = None
        self.aligned = False

        # Hysteresis latch: once aligned, stay aligned until error exceeds resume_tol
        self.aligned = False

        # Params
        self.angle_tol_deg = 1.0
        self.angle_tol = math.radians(self.angle_tol_deg)

        self.resume_tol_deg = 3.0
        self.resume_tol = math.radians(self.resume_tol_deg)

        self.max_detect_range = 10.0      # only consider objects closer than this
        self.turn_speed = 0.5             # rad/s (constant turn speed)
        self.min_valid_range = 0.02       # meters; ignore crazy tiny values

        # Control loop timer
        self.control_hz = 20.0
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f"OrientToStationary started. Stop@±{self.angle_tol_deg:.1f}°, resume@±{self.resume_tol_deg:.1f}°."
        )

    def stationary_cb(self, msg: Bool):
        self.prev_stationary = self.stationary
        self.stationary = bool(msg.data)

        # Rising edge: False -> True
        if self.stationary and not self.prev_stationary:
            self.aligned = False
            self.get_logger().info("Stationary became True -> orienting.")
        if (not self.stationary) and self.prev_stationary:
            self.aligned = False
            self.stop_robot()
            self.get_logger().info("Stationary became False -> stopping.")

    def scan_cb(self, msg: LaserScan):
        current_min_r = float('inf')
        current_min_i = -1

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < self.min_valid_range:
                continue
            if r < current_min_r:
                current_min_r = r
                current_min_i = i

        if current_min_i < 0 or current_min_r == float('inf') or current_min_r > self.max_detect_range:
            self.have_scan = False
            self.last_min_angle = None
            self.last_min_range = None
            return

        angle = msg.angle_min + current_min_i * msg.angle_increment
        self.have_scan = True
        self.last_min_angle = wrap_to_pi(angle)
        self.last_min_range = current_min_r

    def control_loop(self):
        if not self.stationary:
            return
        if not self.have_scan or self.last_min_angle is None:
            return

        err = wrap_to_pi(self.last_min_angle)  # want 0

        # Latch: if aligned, only re-engage if we drift outside resume_tol
        if self.aligned:
            if abs(err) < self.resume_tol:
                return
            self.aligned = False  # drifted too far, re-engage

        #Enter align latch
        if abs(err) <= self.angle_tol:
            self.stop_robot()
            self.aligned = True
            self.get_logger().info(
                f"Aligned within ±{self.angle_tol_deg:.1f}° (err={math.degrees(err):.2f}°)."
            )
            return

        # Rotate toward reducing error 
        twist = Twist()
        twist.angular.z = self.turn_speed if err > 0.0 else -self.turn_speed
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = OrientToStationary()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()