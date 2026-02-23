#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SimpleMovementDetector(Node):

    def __init__(self):
        super().__init__('simple_movement_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.previous_min_r = None
        self.move_threshold = 0.05 # meters change to count as movement
        self.threshold = 10.0 # meters (object closer than this detected)
        self.get_logger().info("Movement detector started.")

    def scan_callback(self, msg: LaserScan):

        # Find closest valid range
        current_min_r = float('inf')
        current_min_i = -1


        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < current_min_r:
                current_min_r = r
                current_min_i = i

        if current_min_r == float('inf'):
            self.get_logger().info("No object detected (no valid LiDAR returns)")
            self.previous_min_r = None
            return
        
        if current_min_r > self.threshold:
            self.get_logger().info(f"No object detected (within 10 m)")
            self.previous_min_r = current_min_r
            return

        # Object is detected
        angle = msg.angle_min + current_min_i * msg.angle_increment
        if self.previous_min_r is None:
            self.get_logger().info(f"Object detected at {current_min_r:.2f} m (first scan).")
        else:
            delta = abs(current_min_r - self.previous_min_r)
            if delta > self.move_threshold:
                self.get_logger().info(
                    f"Movement detected! Change = {delta:.3f} m (range={current_min_r:.2f} m)"
                    )
            else:
                self.get_logger().info(
                    f"Object detected but no movement (range={current_min_r:.2f} m, angle={math.degrees(angle):.1f} degrees)"
                )
        self.previous_min_r = current_min_r

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMovementDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()