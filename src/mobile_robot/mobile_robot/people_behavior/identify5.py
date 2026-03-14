#!/usr/bin/env python3
"""
Node 1: lidar_cluster_detector.py  (ROS2)
- Subscribes to /scan (sensor_msgs/LaserScan)
- Subscribes to /gt_odom (nav_msgs/Odometry)  -- ground truth, no tf used
- Clusters lidar points, rejects walls, finds object centroids
- Publishes /detected_objects (std_msgs/Float32MultiArray)
  Format: flat list [world_x, world_y, robot_r, robot_theta,  ...repeated per object...]
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ── Tunable parameters ────────────────────────────────────────────────────────
CLUSTER_DISTANCE_THRESH = 0.15   # m  – max gap between consecutive points in same cluster
MIN_CLUSTER_POINTS = 4      # fewer → noise, discard
MAX_CLUSTER_POINTS = 80     # more  → wall / large surface, discard
MIN_CLUSTER_WIDTH = 0.05   # m  – minimum bounding width of a valid object
MAX_CLUSTER_WIDTH = 1.2    # m  – wider → wall segment, discard
MIN_RANGE = 0.15   # m  – ignore very close artefacts
MAX_RANGE = 12.0    # m  – ignore far noise
WALL_RESIDUAL_THRESH = 0.01   # m  – line-fit residual below this → wall
MAX_RANGE_VARIANCE = 1.0   # m – max allowed difference between min and max range in a cluster
# ─────────────────────────────────────────────────────────────────────────────


class LidarClusterDetector(Node):

    def __init__(self):
        super().__init__('lidar_cluster_detector')

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Use BEST_EFFORT for laser scan (common in simulation)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Float32MultiArray, '/detected_objects', 10)

        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)

        self.get_logger().info('Lidar cluster detector running.')

    # ── Odometry callback ──────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    # ── Scan callback ──────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        if not self.odom_received:
            return

        # 1. Convert valid ranges to Cartesian points in the ROBOT frame
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE and math.isfinite(r):
                points.append((r * math.cos(angle), r * math.sin(angle), r))
            angle += msg.angle_increment

        if len(points) < MIN_CLUSTER_POINTS:
            return

        # 2. Consecutive-distance clustering
        clusters = self._cluster_points(points)

        # 3. Reject walls / noise
        object_clusters = [c for c in clusters if self._is_object(c)]

        # 4. Compute centroids → transform to world frame (no tf, uses gt_odom)
        output_data = []
        for cluster in object_clusters:
            cx_r, cy_r = self._centroid(cluster)

            r_dist = math.hypot(cx_r, cy_r)
            theta = math.atan2(cy_r, cx_r)   # bearing relative to robot heading

            world_x = self.robot_x + r_dist * math.cos(self.robot_yaw + theta)
            world_y = self.robot_y + r_dist * math.sin(self.robot_yaw + theta)

            output_data.extend([world_x, world_y, r_dist, theta])

        if output_data:
            out_msg = Float32MultiArray()
            out_msg.data = output_data
            self.pub.publish(out_msg)

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _cluster_points(self, points):
        clusters = []
        current = [points[0]]

        for i in range(1, len(points)):
            px, py, _ = points[i]
            lx, ly, _ = points[i - 1]
            if math.hypot(px - lx, py - ly) < CLUSTER_DISTANCE_THRESH:
                current.append(points[i])
            else:
                if len(current) >= MIN_CLUSTER_POINTS:
                    clusters.append(current)
                current = [points[i]]

        if len(current) >= MIN_CLUSTER_POINTS:
            clusters.append(current)

        return clusters

    def _is_object(self, cluster):
        if not (MIN_CLUSTER_POINTS <= len(cluster) <= MAX_CLUSTER_POINTS):
            return False

        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))

        if not (MIN_CLUSTER_WIDTH <= width <= MAX_CLUSTER_WIDTH):
            return False

        # ── Check: angular span ────────────────────────────────────────────────────
        # Walls seen at a glancing angle form long thin clusters with a tiny
        # angular width relative to their distance. A 0.3 m radius cylinder
        # always subtends a meaningful angle; a parallel wall sliver does not.
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        dist = math.hypot(cx, cy)
        angles = [math.atan2(p[1], p[0]) for p in cluster]
        angular_span = abs(angles[-1] - angles[0])
        if dist > 0 and angular_span < 2 * math.atan2(MIN_CLUSTER_WIDTH * 1.5, dist):
            return False

        # ── Check: range variance ─────────────────────────────────────────────────
        # A wall seen near-parallel has wildly different ranges across the cluster
        # (one end close, other end far). A cylinder has similar ranges throughout.
        ranges = [p[2] for p in cluster]
        if max(ranges) - min(ranges) > MAX_RANGE_VARIANCE:
            return False

        # Straightness check – collinear points indicate a wall segment
        # Straightness check – collinear points indicate a wall segment
        if len(cluster) >= 6:
            xs_np = np.array(xs)
            ys_np = np.array(ys)
            coeffs = np.polyfit(xs_np, ys_np, 1)
            residual = float(np.mean(np.abs(ys_np - np.polyval(coeffs, xs_np))))
            if residual < WALL_RESIDUAL_THRESH:
                return False

        return True

    def _centroid(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        return sum(xs) / len(xs), sum(ys) / len(ys)

    @staticmethod
    def _yaw_from_quaternion(qx, qy, qz, qw):
        """Extract yaw (rotation about Z) from a quaternion — no external libraries needed."""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
