#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q):
    """
    Extract yaw from quaternion without tf_transformations
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)



class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver_with_logging")

        # Parameters
        self.declare_parameter("side_length", 3.0)
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("pos_tol", 0.03)
        self.declare_parameter("yaw_tol_deg", 1.0)
        self.declare_parameter("log_rate_hz", 5.0)

        self.declare_parameter("odom_topic", "/odom")
        odom_topic = self.get_parameter("odom_topic").value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)
        self.get_logger().info(f"Using odometry topic: {odom_topic}")


        self.side_length = self.get_parameter("side_length").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.pos_tol = self.get_parameter("pos_tol").value
        self.yaw_tol = math.radians(self.get_parameter("yaw_tol_deg").value)
        self.log_rate = self.get_parameter("log_rate_hz").value

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 50)

        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)
        self.log_timer = self.create_timer(1.0 / self.log_rate, self.log_position)

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        # Square state machine
        self.state = "WAIT_ODOM"
        self.edge_count = 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_yaw = 0.0

        self.get_logger().info("Square driver started.")

    # ---------------- ODOM CALLBACK ----------------
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y
        self.z = p.z
        self.yaw = yaw_from_quat(q)

        self.odom_ready = True

    # ---------------- LOGGING ----------------
    def log_position(self):
        if self.odom_ready:
            self.get_logger().info(
                f"Odometry Position -> "
                f"x: {self.x:.3f} m | "
                f"y: {self.y:.3f} m | "
                f"z: {self.z:.3f} m"
            )

    # ---------------- CONTROL ----------------
    def distance_from_start(self):
        return math.hypot(self.x - self.start_x, self.y - self.start_y)

    def yaw_error(self):
        return wrap_to_pi(self.target_yaw - self.yaw)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):

        if not self.odom_ready:
            return

        if self.state == "WAIT_ODOM":
            self.start_x = self.x
            self.start_y = self.y
            self.state = "DRIVE"
            self.get_logger().info("Starting square motion.")
            return

        if self.state == "DRIVE":

            if self.distance_from_start() >= self.side_length - self.pos_tol:
                self.stop()
                self.target_yaw = wrap_to_pi(self.yaw + math.pi / 2.0)
                self.state = "TURN"
                return

            cmd = Twist()
            cmd.linear.x = self.linear_speed
            self.cmd_pub.publish(cmd)
            return

        if self.state == "TURN":

            if abs(self.yaw_error()) <= self.yaw_tol:
                self.stop()
                self.edge_count += 1

                if self.edge_count >= 4:
                    self.state = "DONE"
                    self.get_logger().info("Finished 3m x 3m square.")
                    return

                self.start_x = self.x
                self.start_y = self.y
                self.state = "DRIVE"
                return

            cmd = Twist()
            cmd.angular.z = self.angular_speed if self.yaw_error() > 0 else -self.angular_speed
            self.cmd_pub.publish(cmd)
            return

        if self.state == "DONE":
            self.stop()


def main():
    rclpy.init()
    node = SquareDriver()
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
