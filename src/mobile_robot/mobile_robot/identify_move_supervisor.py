#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MovementSupervisor(Node):
    
    def __init__(self):
        super().__init__('movement_supervisor')

        # Subscribe to detector output
        self.create_subscription(
            Bool,
            '/object_stationary',
            self.stationary_callback,
            10
        )

        # Publish command to search node
        self.stop_search_pub = self.create_publisher(
            Bool,
            '/stop_search',
            10
        )

        self.stop_search_state = False
        
        self.get_logger().info("Movement Supervisor started.")

    def publish_stop_search(self, value: bool):
        stop_search_msg = Bool()
        stop_search_msg.data = value
        self.stop_search_pub.publish(stop_search_msg)

    def stationary_callback(self, msg: Bool):

        if msg.data: #Object is stationary
            if not self.stop_search_state:
                self.get_logger().info(
                    "Stationary object detected -> STOP SEARCH"
                )
                self.publish_stop_search(True)
                self.stop_search_state = True

        else: # Object moving or not detected
            if self.stop_search_state:
                self.get_logger().info(
                    "Object moving or lost -> RESUME SEARCH"
                )
                self.publish_stop_search(False)
                self.stop_search_state = False

def main(args=None):
    rclpy.init(args=args)
    node = MovementSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

