#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class SimpleMovementDetector(Node):

    def __init__(self):
        super().__init__('identify')

        # LiDAR subscriber
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Stationary object publisher (Bool)
        self.stationary_pub = self.create_publisher(
            Bool,
            '/object_stationary',
            10
        )

        # Stop searching subscriber
        self.create_subscription(
            Bool,
            '/stop_search',
            self.stop_search_callback,
            10
        )

        self.stop_search = False
        self.previous_min_r = None
        self.move_threshold = 0.05 # meters change to count as movement
        self.threshold = 10.0 # meters (object closer than this detected)

        self.get_logger().info("Stationary object detector started.")

    def publish_stationary(self, value: bool):
        msg = Bool()
        msg.data = value
        self.stationary_pub.publish(msg)

    def stop_search_callback(self, msg: Bool):
        self.stop_search = msg.data
        if self.stop_search:
            # avoids large delta when resuming
            self.previous_min_r = None

    def scan_callback(self, msg: LaserScan):
        # Find closest valid range
        current_min_r = float('inf')
        current_min_i = -1

        # find minimum range and lidar index
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < current_min_r:
                current_min_r = r
                current_min_i = i

        # No valid return
        if current_min_r == float('inf'):
            self.get_logger().info(
                "No object detected (no valid LiDAR returns)"
            )
            if not self.stop_search:
                self.publish_stationary(False)
                self.previous_min_r = None
            return

        angle = msg.angle_min + current_min_i * msg.angle_increment
        
        self.get_logger().info(
            f"Closest return:  r={current_min_r:.2f} m, angle={math.degrees(angle):.1f}° (stop_search={self.stop_search})"
        )
        # Object too far
        if current_min_r > self.threshold:
            self.get_logger().info(f"No object detected (within 10 m)")
            if not self.stop_search:
                self.publish_stationary(False)
                self.previous_min_r = current_min_r
            return

        # when object is found to be stationary below, exit callback:
        if self.stop_search:
            return

        # Object detected
        if self.previous_min_r is None:
            self.get_logger().info(
                f"Object detected at {current_min_r:.2f} m (first scan)."
                )
            self.publish_stationary(False)
        else:
            delta = abs(current_min_r - self.previous_min_r)
            if delta > self.move_threshold:
                self.get_logger().info(
                    f"Movement detected. Change = {delta:.3f} m (range={current_min_r:.2f} m)"
                    )
                self.publish_stationary(False)
            else:
                self.get_logger().info(
                    f"Stationary object detected! (range={current_min_r:.2f} m, angle={math.degrees(angle):.1f} degrees)"
                )
                self.publish_stationary(True)
        self.previous_min_r = current_min_r

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMovementDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()