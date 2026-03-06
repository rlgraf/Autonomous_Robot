#!/usr/bin/env python3
"""
Node 2: object_navigator.py  (ROS2)
- Subscribes to /detected_objects (std_msgs/Float32MultiArray)
- Subscribes to /gt_odom (nav_msgs/Odometry) for robot pose
- Publishes /cmd_vel (geometry_msgs/Twist)

Behaviour loop:
  1. SEARCHING  – rotate slowly until an unvisited object is detected
  2. ROTATING   – spin in place to face the nearest unvisited object
  3. APPROACHING – drive toward it, stopping STOP_DISTANCE metres away
  4. DWELLING   – hold position for DWELL_TIME seconds
  5. Mark position as visited, never return → go back to step 1
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool


# ── Tunable parameters ────────────────────────────────────────────────────────
STOP_DISTANCE     = 1.0    # m    – stop this far from the object centroid
ANGULAR_SPEED     = 0.8    # rad/s – rotation speed
LINEAR_SPEED      = 0.4   # m/s  – forward travel speed
ANGLE_TOL         = 0.05   # rad  – heading error considered "aligned"
DIST_TOL          = 0.20   # m    – remaining distance considered "arrived"
DWELL_TIME        = 10.0    # s    – time to wait at the goal
VISITED_RADIUS    = 0.8    # m    – positions closer than this are "visited"
DETECTION_TIMEOUT = 1.5    # s    – detections older than this are stale
# ─────────────────────────────────────────────────────────────────────────────


class ObjectNavigator(Node):

    SEARCHING   = 'SEARCHING'
    ROTATING    = 'ROTATING'
    APPROACHING = 'APPROACHING'
    DWELLING    = 'DWELLING'

    def __init__(self):
        super().__init__('object_navigator')

        # Robot pose
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Latest detections: list of (world_x, world_y, r, theta)
        self.detections    = []
        self.last_det_time = self.get_clock().now()

        # Visited positions
        self.visited: list[tuple[float, float]] = []

        # Current target world position
        self.target_wx: float | None = None
        self.target_wy: float | None = None

        # State machine
        self.state       = self.SEARCHING
        self.dwell_start: Time | None = None

        # Obstacle Avoidance interlock
        # When True, avoid_while_interact.py has taken over /cmd_vel.
        # We must not publish anything until it hands control back

        self.navigation_paused = False

        # Low battery shutdown flag
        self.low_battery_shutdown = False

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.visited_pub = self.create_publisher(Float32MultiArray, '/visited_cylinders', 10)
        self.create_subscription(Float32MultiArray, '/detected_objects',
                                 self.detection_callback, 10)
        self.create_subscription(Odometry, '/odom_gt',
                                 self.odom_callback, 10)
        self.create_subscription(Bool, '/navigation_paused',
                                 self.paused_callback, 10)
        self.create_subscription(Bool, '/low_battery_warning',
                         self.low_battery_callback, 10)

        # 20 Hz control loop via timer
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Object navigator running.')

    # ── Callbacks ──────────────────────────────────────────────────────────────
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
                (float(data[i]), float(data[i + 1]),
                 float(data[i + 2]), float(data[i + 3]))
            )
        self.last_det_time = self.get_clock().now()

    def paused_callback(self, msg: Bool):
        """Receive the interlock flag from avoid_while_interact.py."""
        was_paused = self.navigation_paused
        self.navigation_paused = msg.data

        if msg.data and not was_paused:
            self.get_logger().info(
                'Navigation paused - obstacle avoidance has control'
            )
        elif not msg.data and was_paused:
            # Force re-alignment after avoidance manoeuvre — the robot may have
            # rotated significantly and the target is no longer straight ahead.
            if self.state == self.APPROACHING and self.target_wx is not None:
                self.state = self.ROTATING
                self.get_logger().info(
                    'Navigation resumed - re-aligning with target before approaching.'
                )
            else:
                self.get_logger().info(
                    f'Navigation resumed - continuing from [{self.state}].'
                )

    def low_battery_callback(self, msg: Bool):
        """Yield control when battery is low for recharging."""
        was_shutdown = self.low_battery_shutdown
        self.low_battery_shutdown = msg.data
    
        if msg.data and not was_shutdown:
            self.get_logger().warn(
                'Low battery detected - yielding control to auto_recharge node'
            )
        elif not msg.data and was_shutdown:
            self.get_logger().info(
                'Battery recharged - resuming guest interaction'
            )
            self.visited.clear()
            self.get_logger().info(
                'Cleared visited locations - starting fresh exploration'
            )
            self.state = self.SEARCHING
            self.target_wx = None 
            self.target_wy = None
            self.get_logger().info('Reset navigation state - will find nearest object from current position')

    # ── Control loop (timer) ───────────────────────────────────────────────────
    def control_loop(self):
        if not self.odom_received:
            return

        # Yield control entirely while obstacle avoidance is active.
        # State and target are preserved so we resume exactly where we left off.
        if self.navigation_paused:
            return

        # Stop all guest interaction when battery is low
        if self.low_battery_shutdown:
            return

        if   self.state == self.SEARCHING:   self._handle_searching()
        elif self.state == self.ROTATING:    self._handle_rotating()
        elif self.state == self.APPROACHING: self._handle_approaching()
        elif self.state == self.DWELLING:    self._handle_dwelling()

    # ── State handlers ─────────────────────────────────────────────────────────
    def _handle_searching(self):
        target = self._pick_nearest_unvisited()
        if target is not None:
            self._stop()
            self.target_wx, self.target_wy = target
            self.get_logger().info(
                f'New target acquired: ({self.target_wx:.2f}, {self.target_wy:.2f})'
            )
            self.state = self.ROTATING
        else:
            # Rotate slowly to sweep the lidar until something is found
            twist = Twist()
            twist.angular.z = ANGULAR_SPEED
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

        if abs(error) < ANGLE_TOL:
            self._stop()
            self.get_logger().info('Aligned with target. Approaching.')
            self.state = self.APPROACHING
        else:
            twist = Twist()
            twist.angular.z = ANGULAR_SPEED * math.copysign(1.0, error)
            self.cmd_pub.publish(twist)

    def _handle_approaching(self):
        if self.target_wx is None:
            self.state = self.SEARCHING
            return

        dist_to_centroid = math.hypot(
            self.target_wx - self.robot_x,
            self.target_wy - self.robot_y
        )
        remaining = dist_to_centroid - STOP_DISTANCE

        if remaining <= DIST_TOL:
            self._stop()
            self.get_logger().info(
                f'Arrived 1 m from target. Dwelling for {DWELL_TIME:.0f} s.'
            )
            self.dwell_start = self.get_clock().now()
            self.state = self.DWELLING
            return

        # Proportional heading correction while driving forward
        angle_to_target = math.atan2(
            self.target_wy - self.robot_y,
            self.target_wx - self.robot_x
        )
        heading_error = self._normalise_angle(angle_to_target - self.robot_yaw)

        twist = Twist()
        # Slow down when close
        twist.linear.x  = min(LINEAR_SPEED, LINEAR_SPEED * remaining)
        twist.angular.z = 1.5 * heading_error
        self.cmd_pub.publish(twist)

    def _handle_dwelling(self):
        self._stop()
        elapsed = (self.get_clock().now() - self.dwell_start).nanoseconds * 1e-9

        if elapsed >= DWELL_TIME:
            self.get_logger().info(
                f'Dwell complete. Marking ({self.target_wx:.2f}, '
                f'{self.target_wy:.2f}) as visited — will not return.'
            )
            self.visited.append((self.target_wx, self.target_wy))
            
            visited_msg = Float32MultiArray()
            visited_msg.data = [self.target_wx, self.target_wy]
            self.visited_pub.publish(visited_msg)

            self.target_wx = None
            self.target_wy = None
            # Go to SEARCHING so the robot rotates and re-acquires
            # a fresh view before selecting the next target
            self.state = self.SEARCHING

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _fresh_detections(self):
        """Return detections only if the last message arrived recently."""
        age = (self.get_clock().now() - self.last_det_time).nanoseconds * 1e-9
        return self.detections if age <= DETECTION_TIMEOUT else []

    def _pick_nearest_unvisited(self):
        """Return (world_x, world_y) of the nearest unvisited detection, or None."""
        best      = None
        best_dist = float('inf')

        for wx, wy, _r, _theta in self._fresh_detections():
            if self._is_visited(wx, wy):
                continue
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist < best_dist:
                best_dist = dist
                best = (wx, wy)

        return best

    def _is_visited(self, wx, wy) -> bool:
        return any(
            math.hypot(wx - vx, wy - vy) < VISITED_RADIUS
            for vx, vy in self.visited
        )

    def _stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _yaw_from_quaternion(qx, qy, qz, qw) -> float:
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