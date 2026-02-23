#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SimpleObjectDetector360(Node):

    def __init__(self):
        super().__init__('simple_object_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.threshold = 10.0 # meters (object closer than this detected)
        self.get_logger().info("Simple object detector started.")

    def scan_callback(self, msg: LaserScan):

        best_r = float('inf')
        best_i = -1

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < best_r:
                best_r = r
                best_i = i

        if best_i == -1:
            self.get_logger().info(
                f"No objects detected within {self.threshold} m"
            )
            return # no valid ranges

        angle = msg.angle_min + best_i * msg.angle_increment

        if best_r < self.threshold:
            self.get_logger().info(
                f"Object detected: r={best_r:.2f} m, angle={math.degrees(angle):.1f} deg"
            )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObjectDetector360()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()