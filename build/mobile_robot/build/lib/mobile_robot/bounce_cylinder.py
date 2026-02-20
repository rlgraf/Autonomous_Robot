import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class BounceCylinder(Node):
    """
    Publishes geometry_msgs/Twist to a cmd_vel topic bridged into Gazebo.
    Bounces by flipping direction after the time needed to traverse the range.
    """

    def __init__(self):
        super().__init__('bounce_cylinder')

        self.declare_parameter('cmd_vel_topic', '/model/cylinder_moving/cmd_vel')
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('hz', 20.0)
        self.declare_parameter('min_x', -16.0)
        self.declare_parameter('max_x', 16.0)
        self.declare_parameter('margin', 0.15)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.speed = float(self.get_parameter('speed').value)
        self.hz = float(self.get_parameter('hz').value)
        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.margin = float(self.get_parameter('margin').value)

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
        self.phase_start = self.get_clock().now()

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.hz, self.on_timer)

        self.get_logger().info(
            f'Publishing to {self.cmd_vel_topic} | speed={self.speed} hz={self.hz} '
            f'| min_x={self.min_x} max_x={self.max_x} margin={self.margin} '
            f'| travel_time={self.travel_time:.2f}s'
        )

    def on_timer(self):
        now = self.get_clock().now()
        elapsed = (now - self.phase_start).nanoseconds * 1e-9

        if elapsed >= self.travel_time:
            self.direction *= -1.0
            self.phase_start = now
            self.get_logger().info(f'Reversing -> {"+" if self.direction > 0 else "-"}x')

        msg = Twist()
        msg.linear.x = self.direction * self.speed
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BounceCylinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop motion on shutdown
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
