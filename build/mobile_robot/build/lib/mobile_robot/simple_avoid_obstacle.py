import math 
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def finite_range(r: float) -> bool:
    return (not math.isinf(r)) and (not math.isnan(r))

class SimpleLidarAvoider(Node):
    def __init__(self):
        super().__init__('simple_lidar_avoider')

        # --- Parameters 

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('forward_speed', 0.8) # m/s
        self.declare_parameter('turn_speed', 0.8) # rad/s
        self.declare_parameter('stop_distance', 1.0) # meters
        self.declare_parameter('front_angle_deg', 45.0) # +/- degrees to consider "in front"
        self.declare_parameter('control_hz', 50.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.control_hz = float(self.get_parameter('control_hz').value)

        # Latest "front min distance"

        self.front_min = float('inf')
        self.have_scan = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        period = 1.0 / max(self.control_hz, 1e-6)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Running. Sub: {self.scan_topic} Pub: {self.cmd_vel_topic} "
            f"stop_distance={self.stop_distance}m front angle = +/- {self.front_angle_deg} degrees"
        )

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)

        best_r = float('inf')
        best_i = None
        for i, r in enumerate(msg.ranges):
            if finite_range(r) and msg.range_min <= r <= msg.range_max and r < best_r:
                best_r, best_i = r, i

        if best_i is not None:
            center = n // 2
            best_angle = msg.angle_min + best_i * msg.angle_increment
            self.get_logger().info(
                f"GLOBAL min={best_r:.3f}m at idx={best_i} (center={center}), angle={best_angle:.3f} rad",
                throttle_duration_sec=1.0
    )


        if n == 0:
            return

        # For a full 360 scan, "front" is usually at the middle index
        center = n // 2

        # Convert degrees to number of samples on each side
        # total span in degrees:
        span_deg = (msg.angle_max - msg.angle_min) * 180.0 / math.pi
        samples_per_deg = n / span_deg if span_deg > 1e-6 else 0.0
        half_width = int(self.front_angle_deg * samples_per_deg)

        i0 = max(0, center - half_width)
        i1 = min(n - 1, center + half_width)

        m = float('inf')
        for r in msg.ranges[i0:i1 + 1]:
            if finite_range(r) and msg.range_min <= r <= msg.range_max:
                if r < m:
                    m = r

        self.front_min = m
        self.have_scan = True

    def on_timer(self):
        twist = Twist()

        # If we haven't received scan data yet, do nothing (safer than driving blind)
        if not self.have_scan:
            self.cmd_pub.publish(twist)
            return

        self.get_logger().info(f"front_min={self.front_min}")

        if self.front_min < self.stop_distance:
            # Obstacle close in front: turn in place (left or right)
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
        else:
            # Clear: drive forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = SimpleLidarAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        # Stop robot on exit
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
