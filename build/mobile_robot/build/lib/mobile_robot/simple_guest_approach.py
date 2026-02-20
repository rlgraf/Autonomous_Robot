#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, Twist
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point


class StationaryTargetApproach(Node):
    def __init__(self):
        super().__init__("stationary_target_approach")

        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_lin", 0.2)
        self.declare_parameter("max_ang", 0.2)
        self.declare_parameter("stop_dist", 1.0)
        self.declare_parameter("k_ang", 1.8)
        self.declare_parameter("k_lin", 0.6)
        self.declare_parameter("turn_only_angle", 0.25)   # rad (~14 deg)
        self.declare_parameter("drive_angle", 0.35)       # rad (~20 deg) hysteresis

        self.base_frame = self.get_parameter("base_frame").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)
        self.stop_dist = float(self.get_parameter("stop_dist").value)
        self.k_ang = float(self.get_parameter("k_ang").value)
        self.k_lin = float(self.get_parameter("k_lin").value)
        self.turn_only_angle = float(self.get_parameter("turn_only_angle").value)
        self.drive_angle = float(self.get_parameter("drive_angle").value)

        self.turning_in_place = True

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(PointStamped, "/object/target_point", self.on_target, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.last_target = None

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info(f"Approach up. cmd_vel={self.cmd_vel_topic}, base_frame={self.base_frame}")

    def on_target(self, msg: PointStamped):
        self.last_target = msg

    def control_loop(self):
        if self.last_target is None:
            self.publish_stop()
            return

        # Transform target point into base frame using latest TF
        try:
            stamp = Time.from_msg(self.last_target.header.stamp)

            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.last_target.header.frame_id,
                stamp,   # <-- use target timestamp
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException:
            self.publish_stop()
            return

        pt_base = do_transform_point(self.last_target, tf).point

        x = pt_base.x
        y = pt_base.y
        dist = math.hypot(x, y)
        ang = math.atan2(y, x)

        cmd = Twist()

        if dist < self.stop_dist:
            self.publish_stop()
            return

        cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * ang))

        # --- Turn-first state machine (with hysteresis) ---
        # If we're not aligned, rotate in place.
        if self.turning_in_place:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * ang))

            # Exit turning mode once we're well-aligned
            if abs(ang) < self.turn_only_angle:
                self.turning_in_place = False

        else:
            # If we drift out of alignment, go back to turning mode
            if abs(ang) > self.drive_angle:
                self.turning_in_place = True
                cmd.linear.x = 0.0
                cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * ang))
            else:
                # Drive while making small heading corrections
                cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * ang))
                cmd.linear.x = max(0.0, min(self.max_lin, self.k_lin * (dist - self.stop_dist)))


        self.pub.publish(cmd)

    def publish_stop(self):
        self.turning_in_place = True
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = StationaryTargetApproach()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
