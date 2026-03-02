#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CylinderBouncer:
    """Holds per-cylinder state and publisher."""
    def __init__(self, node: Node, name: str):
        self.node = node
        self.name = name

        # Parameter names are namespaced per cylinder: c1.*, c2.*
        ns = f"{name}."

        node.declare_parameter(ns + "cmd_vel_topic", f"/model/{name}/cmd_vel")
        node.declare_parameter(ns + "speed", 1.0)
        node.declare_parameter(ns + "hz", 20.0)
        node.declare_parameter(ns + "min_x", -32.0)
        node.declare_parameter(ns + "max_x", 32.0)
        node.declare_parameter(ns + "margin", 0.15)

        self.cmd_vel_topic = node.get_parameter(ns + "cmd_vel_topic").value
        self.speed = float(node.get_parameter(ns + "speed").value)
        self.hz = float(node.get_parameter(ns + "hz").value)
        self.min_x = float(node.get_parameter(ns + "min_x").value)
        self.max_x = float(node.get_parameter(ns + "max_x").value)
        self.margin = float(node.get_parameter(ns + "margin").value)

        if self.hz <= 0.0:
            self.hz = 20.0
        if self.speed <= 0.0:
            self.speed = 0.5

        span = (self.max_x - self.min_x) - 2.0 * self.margin
        if span <= 0.0:
            self.margin = 0.0
            span = (self.max_x - self.min_x)

        self.travel_time = span / self.speed
        self.direction = 1.0


        self.phase_start = node.get_clock().now()

        self.pub = node.create_publisher(Twist, self.cmd_vel_topic, 10)

        node.get_logger().info(
            f"[{self.name}] Publishing to {self.cmd_vel_topic} | speed={self.speed} hz={self.hz} "
            f"| min_x={self.min_x} max_x={self.max_x} margin={self.margin} "
            f"| travel_time={self.travel_time:.2f}s"
        )

    def step(self, now):
        elapsed = (now - self.phase_start).nanoseconds * 1e-9
        if elapsed >= self.travel_time:
            self.direction *= -1.0
            self.phase_start = now
            self.node.get_logger().info(f"[{self.name}] Reversing -> {'+' if self.direction > 0 else '-'}x")

        msg = Twist()
        msg.linear.x = self.direction * self.speed
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())


class BounceCylinders(Node):
    """
    Publishes geometry_msgs/Twist to multiple Gazebo model cmd_vel topics.
    """

    def __init__(self):
        super().__init__("bounce_cylinders")

        # Pick your two model names here (these should match the Gazebo model names).
        self.declare_parameter("cylinder_1_name", "cylinder_moving_1")
        self.declare_parameter("cylinder_2_name", "cylinder_moving_2")

        c1_name = self.get_parameter("cylinder_1_name").value
        c2_name = self.get_parameter("cylinder_2_name").value

        # Each gets its own bouncer + publisher + state
        self.c1 = CylinderBouncer(self, c1_name)
        self.c2 = CylinderBouncer(self, c2_name)

        self.c2.direction = -1.0

        # Run one timer at the fastest requested rate, update both each tick
        hz = max(self.c1.hz, self.c2.hz)
        self.timer = self.create_timer(1.0 / hz, self.on_timer)

    def on_timer(self):
        now = self.get_clock().now()
        self.c1.step(now)
        self.c2.step(now)

    def destroy_node(self):
        # stop both on shutdown
        try:
            self.c1.stop()
            self.c2.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BounceCylinders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
