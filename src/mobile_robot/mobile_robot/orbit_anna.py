#!/usr/bin/env python3
import math
from dataclasses import dataclass

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


def yaw_from_quat(q) -> float:
    # geometry_msgs/Quaternion -> yaw
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Params:
    center_x: float = 0.0     # Hannah/Anna x
    center_y: float = 0.0     # Hannah/Anna y
    radius: float = 2.0       # circle radius [m]
    orbit_rate: float = 0.25  # rad/s (how fast we move around the circle)

    k_lin: float = 0.8        # position gain
    k_ang: float = 2.5        # heading gain

    v_max: float = 0.6        # m/s
    w_max: float = 1.5        # rad/s

    odom_topic: str = "/odom"
    cmd_vel_topic: str = "/cmd_vel"


class OrbitController(Node):
    def __init__(self):
        super().__init__("orbit_controller")

        # Declare parameters
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("radius", 2.0)
        self.declare_parameter("orbit_rate", 0.25)
        self.declare_parameter("k_lin", 0.8)
        self.declare_parameter("k_ang", 2.5)
        self.declare_parameter("v_max", 0.6)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.p = Params(
            center_x=float(self.get_parameter("center_x").value),
            center_y=float(self.get_parameter("center_y").value),
            radius=float(self.get_parameter("radius").value),
            orbit_rate=float(self.get_parameter("orbit_rate").value),
            k_lin=float(self.get_parameter("k_lin").value),
            k_ang=float(self.get_parameter("k_ang").value),
            v_max=float(self.get_parameter("v_max").value),
            w_max=float(self.get_parameter("w_max").value),
            odom_topic=str(self.get_parameter("odom_topic").value),
            cmd_vel_topic=str(self.get_parameter("cmd_vel_topic").value),
        )

        self.cmd_pub = self.create_publisher(Twist, self.p.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.p.odom_topic, self.on_odom, 10)

        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz

        self.get_logger().info(
            f"OrbitController running. Center=({self.p.center_x:.3f},{self.p.center_y:.3f}), "
            f"R={self.p.radius:.3f}, orbit_rate={self.p.orbit_rate:.3f} rad/s, "
            f"odom={self.p.odom_topic}, cmd_vel={self.p.cmd_vel_topic}"
        )

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    def stop(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def on_timer(self):
        if not self.have_odom:
            return

        # Desired point on circle around Hannah/Anna
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds * 1e-9
        phi = self.p.orbit_rate * dt

        xd = self.p.center_x + self.p.radius * math.cos(phi)
        yd = self.p.center_y + self.p.radius * math.sin(phi)

        dx = xd - self.x
        dy = yd - self.y
        dist = math.hypot(dx, dy)

        heading_to_goal = math.atan2(dy, dx)
        e_yaw = wrap_to_pi(heading_to_goal - self.yaw)

        # Simple unicycle control to track the moving point
        v = self.p.k_lin * dist
        w = self.p.k_ang * e_yaw

        # Clamp
        v = max(-self.p.v_max, min(self.p.v_max, v))
        w = max(-self.p.w_max, min(self.p.w_max, w))

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = OrbitController()
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