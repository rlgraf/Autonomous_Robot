#!/usr/bin/env python3
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Deque, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

def yaw_from_quat(q) -> float:
    # yaw from geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

@dataclass
class Odom2D:
    x: float
    y: float
    yaw: float
    stamp_sec: float

class WorldFrameStationaryDetector(Node):
    def __init__(self):
        super().__init__("identify2")

        # ---- Parameters / tunables ----
        self.range_threshold_m = 5.0
        self.stationary_window_sec = 3.0
        self.stationary_tol_m = 0.08 # world-frame position jitter allowed 
        
        # Static extrinsic: base -> Lidar
        self.lidar_in_base_x = 0.0
        self.lidar_in_base_y = 0.0
        self.lidar_in_base_yaw = 0.0

        # ---- ROS I/O ----
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom_gt", self.odom_callback, 50)

        self.stationary_pub = self.create_publisher(Bool, "/object_stationary", 10)

        # ---- State ----
        self.latest_odom: Optional[Odom2D] = None
        # history of (t, xW, yW) for lst window
        self.hist: Deque[tuple[float, float, float]] = deque(maxlen = 1000)

        self.get_logger().info("World-frame stationary detector running")

    def publish_stationary(self, value: bool):
        msg = Bool()
        msg.data = value
        self.stationary_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        t = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.latest_odom = Odom2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(yaw),
            stamp_sec=t,
        )

    def lidar_point_to_world(self, r: float, theta_lidar: float, odom: Odom2D) -> Tuple[float, float]:
        # ---- Transform a LiDAR polar point into world frame using custom transforms ----
        
        # LiDAR polar -> LiDAR Cartesian
        xL = r * math.cos(theta_lidar)
        yL = r * math.sin(theta_lidar)

        # LiDAR -> Base (static extrinsic: base_T_lidar applied to point in lidar frame)
        cy = math.cos(self.lidar_in_base_yaw)
        sy = math.sin(self.lidar_in_base_yaw)
        xB = cy * xL - sy * yL + self.lidar_in_base_x
        yB = sy * xL + cy * yL + self.lidar_in_base_y

        # Base -> World (from odom_gt)

        c = math.cos(odom.yaw)
        s = math.sin(odom.yaw)
        xW = c * xB - s* yB + odom.x
        yW = s * xB + c * yB + odom.y
        return xW, yW

    def scan_callback(self, msg: LaserScan):
        if self.latest_odom is None: 
            self.get_logger().warn("No /odom_gt recievec yet; publishing False.")
            self.publish_stationary(False)
            return

        # Find closest valid range
        min_r = float("inf")
        min_i = -1
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < min_r:
                min_r = r
                min_i = i

        # No valid return 
        if min_r == float("inf"):
            self.get_logger().info("No valid LiDAR returns.")
            self.publish_stationary(False)
            self.hist.clear()
            return

        # Too far
        if min_r > self.range_threshold_m:
            self.get_logger().info(f"No object within {self.range_thresholdf_m:.1f} m (closest={min_r:.2f} m).")
            self.publish_stationary(False)
            self.hist.clear()
            return

        # Compute angle and world point
        theta = msg.angle_min + min_i * msg.angle_increment
        odom = self.latest_odom
        xW, yW = self.lidar_point_to_world(min_r, theta, odom)

        t_scan = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

        self.get_logger().info(
            f"Closest: r={min_r:.2f}m, theta={math.degrees(theta):.1f}deg -> world=({xW:.2f},{yW:.2f})"
        )

        # Update history
        self.hist.append((t_scan, xW, yW))

        # Keep only points in last stationary_window_sec seconds
        while self.hist and (t_scan - self.hist[0][0]) > self.stationary_window_sec:
            self.hist.popleft()

        # Need enough time span to decide
        if not self.hist or (self.hist[-1][0] - self.hist[0][0]) < self.stationary_window_sec:
            self.publish_stationary(False)
            return

        xs = [p[1] for p in self.hist]
        ys = [p[2] for p in self.hist]
        mx = sum(xs) / len(xs)
        my = sum(ys) /len(ys)

        max_dev = 0.0
        for x, y in zip(xs, ys):
            d = math.hypot(x - mx, y - my)
            if d > max_dev:
                max_dev = d

        stationary = (max_dev <= self.stationary_tol_m)
        if stationary: 
            self.get_logger().info(
                f"STATIONARY: max_dev={max_dev:.3f}m over {self.stationary_window_sec:.1f}s, mean({mx:.2f}, {my:.2f})"
            )
        else:
            self.get_logger().info(f"MOVING: max_dev={max_dev:.3f}m")

        self.publish_stationary(stationary)

def main(args=None):
    rclpy.init(args=args)
    node = WorldFrameStationaryDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
