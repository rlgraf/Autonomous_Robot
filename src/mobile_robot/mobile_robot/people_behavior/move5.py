#!/usr/bin/env python3
"""
Node 2: object_navigator.py  (ROS2)

Behaviour:
  SEARCHING   – rotate slowly until an unvisited object is detected
  ROTATING    – spin in place to face nearest unvisited object
  APPROACHING – drive toward it, stopping STOP_DISTANCE metres away
  DWELLING    – hold position for DWELL_TIME seconds
  (DEPART via Supervisor) – request supervisor to move robot away safely
  Then return to SEARCHING
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool


class ObjectNavigator(Node):

    SEARCHING   = 'SEARCHING'
    ROTATING    = 'ROTATING'
    APPROACHING = 'APPROACHING'
    DWELLING    = 'DWELLING'

    def __init__(self):
        super().__init__('object_navigator')

        # ── Declare shared / tunable parameters ─────────────────────
        self.declare_parameter('gain_multiplier', 1.0)

        self.declare_parameter('base_linear_speed', 0.5)
        self.declare_parameter('base_angular_speed', 0.4)

        self.declare_parameter('stop_distance', 1.0)
        self.declare_parameter('angle_tol', 0.05)
        self.declare_parameter('dist_tol', 0.20)
        self.declare_parameter('dwell_time', 5.0)
        self.declare_parameter('visited_radius', 0.8)
        self.declare_parameter('detection_timeout', 1.5)

        # ── Read parameters ─────────────────────────────────────────
        self.gain_multiplier = float(self.get_parameter('gain_multiplier').value)

        self.base_linear_speed = float(self.get_parameter('base_linear_speed').value)
        self.base_angular_speed = float(self.get_parameter('base_angular_speed').value)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.angle_tol = float(self.get_parameter('angle_tol').value)
        self.dist_tol = float(self.get_parameter('dist_tol').value)
        self.dwell_time = float(self.get_parameter('dwell_time').value)
        self.visited_radius = float(self.get_parameter('visited_radius').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)

        # Derived speeds
        self.linear_speed = self.base_linear_speed * self.gain_multiplier
        self.angular_speed = self.base_angular_speed * self.gain_multiplier

        self.get_logger().info(
            f'Navigator params: gain={self.gain_multiplier:.2f}, '
            f'linear_speed={self.linear_speed:.2f}, '
            f'angular_speed={self.angular_speed:.2f}'
        )

        # Supervisor interlock (supervisor owns /cmd_vel when active)
        self.supervisor_active = False
        self.create_subscription(Bool, '/supervisor_active', self.supervisor_cb, 10)

        # Request supervisor to execute a safe depart after dwelling
        self.depart_pub = self.create_publisher(Bool, '/depart_request', 10)

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Detections
        self.detections = []
        self.last_det_time = self.get_clock().now()

        # Visited world positions
        self.visited = []

        # Current target
        self.target_wx = None
        self.target_wy = None

        # State machine
        self.state = self.SEARCHING
        self.dwell_start = None

        # Obstacle avoidance interlock
        self.navigation_paused = False

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        self.visited_pub = self.create_publisher(
            Float32MultiArray,
            '/visited_columns',
            10
        )
            
        self.create_subscription(
            Float32MultiArray, '/detected_objects',
            self.detection_callback, 10)

        self.create_subscription(
            Odometry, '/odom_gt',
            self.odom_callback, 10)

        self.create_subscription(
            Bool, '/navigation_paused',
            self.paused_callback, 10)

        # 20 Hz loop
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Object navigator running.')

    # ── Callbacks ───────────────────────────────────────────────────
    def supervisor_cb(self, msg: Bool):
        self.supervisor_active = msg.data

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def detection_callback(self, msg: Float32MultiArray):
        data = msg.data
        self.detections = []
        for i in range(0, len(data) - 3, 4):
            self.detections.append(
                (float(data[i]),
                 float(data[i + 1]),
                 float(data[i + 2]),
                 float(data[i + 3]))
            )
        self.last_det_time = self.get_clock().now()

    def paused_callback(self, msg: Bool):
        was_paused = self.navigation_paused
        self.navigation_paused = msg.data

        if msg.data and not was_paused:
            self.get_logger().info('Navigation paused - avoidance has control')
        elif not msg.data and was_paused:
            self.get_logger().info(f'Navigation resumed - continuing from [{self.state}]')

            # Re-align if we were approaching
            if self.state == self.APPROACHING and self.target_wx is not None:
                self.state = self.ROTATING

    # ── Control Loop ────────────────────────────────────────────────
    def control_loop(self):
        if not self.odom_received:
            return

        # If avoidance has control, do not publish /cmd_vel
        if self.navigation_paused:
            return

        # If supervisor has control, do not publish /cmd_vel
        if self.supervisor_active:
            return

        if self.state == self.SEARCHING:
            self._handle_searching()
        elif self.state == self.ROTATING:
            self._handle_rotating()
        elif self.state == self.APPROACHING:
            self._handle_approaching()
        elif self.state == self.DWELLING:
            self._handle_dwelling()

    # ── State Handlers ──────────────────────────────────────────────
    def _handle_searching(self):
        target = self._pick_nearest_unvisited()

        if target is not None:
            self._stop()
            self.target_wx, self.target_wy = target
            self.state = self.ROTATING
            self.get_logger().info(f'New target: ({self.target_wx:.2f}, {self.target_wy:.2f})')
        else:
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_pub.publish(twist)

    def _handle_rotating(self):
        if self.target_wx is None:
            self.state = self.SEARCHING
            return

        angle_to_target = math.atan2(
            self.target_wy - self.robot_y,
            self.target_wx - self.robot_x
        )
        error = self._normalise_angle(angle_to_target - self.robot_yaw)

        if abs(error) < self.angle_tol:
            self._stop()
            self.state = self.APPROACHING
        else:
            twist = Twist()
            twist.angular.z = self.angular_speed * math.copysign(1.0, error)
            self.cmd_pub.publish(twist)

    def _handle_approaching(self):
        if self.target_wx is None:
            self.state = self.SEARCHING
            return

        dist = math.hypot(
            self.target_wx - self.robot_x,
            self.target_wy - self.robot_y
        )
        remaining = dist - self.stop_distance

        if remaining <= self.dist_tol:
            self._stop()
            self.dwell_start = self.get_clock().now()
            self.state = self.DWELLING
            self.get_logger().info(f'Arrived near object. Dwelling for {self.dwell_time}s.')
            return

        angle_to_target = math.atan2(
            self.target_wy - self.robot_y,
            self.target_wx - self.robot_x
        )
        heading_error = self._normalise_angle(angle_to_target - self.robot_yaw)

        twist = Twist()
        twist.linear.x = min(self.linear_speed, self.linear_speed * remaining)
        twist.angular.z = 1.5 * heading_error
        self.cmd_pub.publish(twist)

    def _handle_dwelling(self):
        self._stop()

        if self.dwell_start is None:
            self.dwell_start = self.get_clock().now()
            return

        elapsed = (self.get_clock().now() - self.dwell_start).nanoseconds * 1e-9
        if elapsed < self.dwell_time:
            return

        # Mark visited once
        # Mark visited once
        if self.target_wx is not None and self.target_wy is not None:
            self.visited.append((self.target_wx, self.target_wy))

            msg = Float32MultiArray()
            msg.data = [self.target_wx, self.target_wy]
            self.visited_pub.publish(msg)

        self.get_logger().info('Dwell complete. Requesting supervisor depart.')

        # Request supervisor to perform a safe depart maneuver
        req = Bool()
        req.data = True
        self.depart_pub.publish(req)

        # Clear current target and restart search after supervisor moves us away
        self.target_wx = None
        self.target_wy = None
        self.dwell_start = None
        self.state = self.SEARCHING
        return

    # ── Helper Functions ────────────────────────────────────────────
    def _fresh_detections(self):
        age = (self.get_clock().now() - self.last_det_time).nanoseconds * 1e-9
        return self.detections if age <= self.detection_timeout else []

    def _pick_nearest_unvisited(self):
        best = None
        best_dist = float('inf')

        for wx, wy, _r, _theta in self._fresh_detections():
            if self._is_visited(wx, wy):
                continue

            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist < best_dist:
                best_dist = dist
                best = (wx, wy)

        return best

    def _is_visited(self, wx, wy):
        return any(
            math.hypot(wx - vx, wy - vy) < self.visited_radius
            for vx, vy in self.visited
        )

    def _stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalise_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _yaw_from_quaternion(qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()