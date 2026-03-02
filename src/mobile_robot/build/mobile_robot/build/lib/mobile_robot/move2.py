#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Odom2D:
    x: float
    y: float
    yaw: float


class DebounceBool:
    def __init__(self, on_delay: float, off_delay: float):
        self.on_delay = float(on_delay)
        self.off_delay = float(off_delay)
        self.state = False
        self._candidate = False
        self._t0: Optional[float] = None

    def update(self, raw: bool, t: float) -> bool:
        if raw == self.state:
            self._candidate = raw
            self._t0 = None
            return self.state

        if self._t0 is None or self._candidate != raw:
            self._candidate = raw
            self._t0 = t
            return self.state

        dt = t - self._t0
        if raw and (dt >= self.on_delay):
            self.state = True
            self._t0 = None
        elif (not raw) and (dt >= self.off_delay):
            self.state = False
            self._t0 = None
        return self.state


class OrientToStationaryTarget(Node):
    def __init__(self):
        super().__init__("move")

        # ---------------- Topic config ----------------
        # If your robot listens to /cmd_vel_nav, keep this.
        # If it listens to /cmd_vel, change here or remap in launch.
        self.cmd_topic = "/cmd_vel_nav"

        # ---------------- Control params ----------------
        self.angle_tol_deg = 1.0
        self.resume_tol_deg = 3.0
        self.turn_speed = 0.5            # rad/s
        self.max_turn_speed = 1.0        # safety clamp
        self.kp = 1.5                    # proportional steering (set 0 for constant-speed logic)
        self.use_p = True                # True: P controller, False: constant sign

        # Debounce stationary input (prevents stop/start from detector flicker)
        self.stationary_on_delay = 0.30
        self.stationary_off_delay = 0.50
        self._stationary_deb = DebounceBool(self.stationary_on_delay, self.stationary_off_delay)

        # ---------------- State ----------------
        self.raw_stationary = False
        self.stationary = False

        self.have_odom = False
        self.odom: Optional[Odom2D] = None

        self.have_target = False
        self.target_xy = (0.0, 0.0)
        self.target_stamp_sec = 0.0

        self.aligned = False

        self.angle_tol = math.radians(self.angle_tol_deg)
        self.resume_tol = math.radians(self.resume_tol_deg)

        # ---------------- ROS I/O ----------------
        self.create_subscription(Bool, "/object_stationary", self.stationary_cb, 10)
        self.create_subscription(PointStamped, "/stationary_target", self.target_cb, 10)
        self.create_subscription(Odometry, "/odom_gt", self.odom_cb, 50)

        # Optional: track id for debugging
        self.create_subscription(Int32, "/stationary_track_id", self.track_id_cb, 10)
        self._last_tid: Optional[int] = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # Control loop
        self.control_hz = 20.0
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f"OrientToStationaryTarget started. Publishing {self.cmd_topic}. "
            f"Stop@±{self.angle_tol_deg:.1f}°, resume@±{self.resume_tol_deg:.1f}°."
        )

    def track_id_cb(self, msg: Int32):
        tid = int(msg.data)
        if self._last_tid != tid:
            self._last_tid = tid
            self.get_logger().info(f"Tracking stationary tid={tid}")

    def odom_cb(self, msg: Odometry):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.odom = Odom2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(yaw),
        )
        self.have_odom = True

    def target_cb(self, msg: PointStamped):
        # assuming identify publishes in "odom" coordinates consistent with /odom_gt
        self.target_xy = (float(msg.point.x), float(msg.point.y))
        self.have_target = True
        self.target_stamp_sec = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    def stationary_cb(self, msg: Bool):
        self.raw_stationary = bool(msg.data)

    def control_loop(self):
        # Use node time for debouncing transitions
        t = self.get_clock().now().nanoseconds * 1e-9
        self.stationary = self._stationary_deb.update(self.raw_stationary, t)

        if not self.stationary:
            # If we just fell out, stop once
            if self.aligned is False:
                # already not aligned; still ensure we are not commanding motion
                self.stop_robot()
            self.aligned = False
            return

        if not (self.have_odom and self.have_target):
            return

        # Compute bearing error to target in world frame
        ox, oy, oyaw = self.odom.x, self.odom.y, self.odom.yaw
        tx, ty = self.target_xy
        dx = tx - ox
        dy = ty - oy

        # If target coincident, stop
        if (dx * dx + dy * dy) < 1e-6:
            self.stop_robot()
            self.aligned = True
            return

        desired = math.atan2(dy, dx)
        err = wrap_to_pi(desired - oyaw)

        # Hysteresis latch
        if self.aligned:
            if abs(err) < self.resume_tol:
                return
            self.aligned = False

        if abs(err) <= self.angle_tol:
            self.stop_robot()
            self.aligned = True
            self.get_logger().info(f"Aligned (err={math.degrees(err):.2f}°).")
            return

        twist = Twist()
        if self.use_p:
            wz = self.kp * err
            # clamp
            wz = max(-self.max_turn_speed, min(self.max_turn_speed, wz))
            # enforce minimum turn magnitude so it actually moves
            if abs(wz) < self.turn_speed:
                wz = self.turn_speed if wz > 0 else -self.turn_speed
            twist.angular.z = float(wz)
        else:
            twist.angular.z = float(self.turn_speed if err > 0.0 else -self.turn_speed)

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = OrientToStationaryTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()