import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class HeadCircle(Node):
    def __init__(self):
        super().__init__("head_circle")

        # ---- parameters you can tweak
        self.declare_parameter("topic", "/head_yaw_cmd")
        self.declare_parameter("speed_rad_s", 0.6)   # rotation speed
        self.declare_parameter("hz", 30.0)

        topic = self.get_parameter("topic").value
        self.speed = float(self.get_parameter("speed_rad_s").value)
        hz = float(self.get_parameter("hz").value)

        self.pub = self.create_publisher(Float64, topic, 10)

        self.angle = 0.0
        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"Spinning head on {topic} at {self.speed:.2f} rad/s"
        )

    def on_timer(self):
        # advance angle
        self.angle += self.speed * self.dt

        # wrap to [-pi, pi]
        if self.angle > math.pi:
            self.angle -= 2 * math.pi

        msg = Float64()
        msg.data = self.angle
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HeadCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
