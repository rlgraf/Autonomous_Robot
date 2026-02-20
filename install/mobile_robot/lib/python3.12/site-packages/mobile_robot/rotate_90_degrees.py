#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Twist
import tf2_ros


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q) -> float:
    """Yaw from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RotateAngle(Node):
    def __init__(self):
        super().__init__("rotate_angle")

        # ---- Config (edit these if needed) ----
        self.world_frame = "odom"
        self.base_frame = "base_footprint"   # try "base_link" if yaw looks wrong
        self.cmd_topic = "/cmd_vel"

        self.target_deg = 90.0               # +CCW, use -90 for CW
        self.tol_deg = 0.25                  # tighter than before
        self.hold_time_sec = 0.25            # must be within tol for this long

        self.kp = 2.0                        # proportional gain
        self.max_w = 0.20                    # rad/s (slower -> more accurate)
        self.min_w = 0.12                    # rad/s (fight deadband/stiction)
        self.rate_hz = 50.0
        # --------------------------------------

        self.target = math.radians(self.target_deg)
        self.tol = math.radians(self.tol_deg)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_yaw = None
        self.start_yaw = None
        self.accum = 0.0

        self.good_time = 0.0
        self.dt = 1.0 / self.rate_hz

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info(
            f"RotateAngle: target={self.target_deg:.2f}°, tol={self.tol_deg:.2f}°, "
            f"frames=({self.world_frame} -> {self.base_frame}), cmd={self.cmd_topic}"
        )

    def get_yaw(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                Time(),  # latest
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return None
        return yaw_from_quaternion(tf.transform.rotation)

    def stop_robot(self):
        self.pub.publish(Twist())

    def loop(self):
        yaw = self.get_yaw()
        if yaw is None:
            return

        # Initialize
        if self.last_yaw is None:
            self.last_yaw = yaw
            self.start_yaw = yaw
            self.get_logger().info(f"Start yaw: {math.degrees(yaw):.2f}°")
            return

        # Accumulate yaw change robustly (handles wrap)
        dyaw = normalize_angle(yaw - self.last_yaw)
        self.last_yaw = yaw
        self.accum += dyaw

        err = self.target - self.accum

        # Debounced stop condition
        if abs(err) <= self.tol:
            self.good_time += self.dt
        else:
            self.good_time = 0.0

        if self.good_time >= self.hold_time_sec:
            self.stop_robot()
            final_yaw = yaw
            rotated_deg = math.degrees(self.accum)
            start_deg = math.degrees(self.start_yaw)
            stop_deg = math.degrees(final_yaw)
            self.get_logger().info(
                f"Done. rotated={rotated_deg:.2f}° (target {self.target_deg:.2f}°). "
                f"yaw start={start_deg:.2f}°, yaw stop={stop_deg:.2f}°"
            )
            rclpy.shutdown()
            return

        # Proportional control with clamps and minimum speed
        w = self.kp * err
        w = max(-self.max_w, min(self.max_w, w))

        # Avoid deadband: if we still have meaningful error, force min speed
        if abs(err) > self.tol and abs(w) < self.min_w:
            w = math.copysign(self.min_w, w)

        cmd = Twist()
        cmd.angular.z = w
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = RotateAngle()
    try:
        rclpy.spin(node)
    finally:
        # Ensure stop on exit
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
