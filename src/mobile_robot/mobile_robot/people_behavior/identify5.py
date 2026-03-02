#!/usr/bin/env python3
"""
Node 1: lidar_cluster_detector.py  (ROS2) — SUPERVISOR-COMPATIBLE

This node does NOT publish /cmd_vel, so it does not need the same “cmd_vel_*”
split as avoidance/recharge. The only supervisor-related change that is
typically worth doing here is to publish an explicit heartbeat/health Bool
(optional) so the supervisor can detect if perception is alive.

Edits applied (safe + optional):
  1) Keep topics the same (/scan, /odom_gt, /detected_objects).
  2) Add optional /perception_alive publisher (Bool) at ~1 Hz.
     - Lets supervisor know the node is running and producing scan callbacks.
  3) Add a “no detections” publish option (optional) if you want downstream
     nodes to time out cleanly. By default, this code remains “publish only if any”.

If you do NOT want the heartbeat topic, remove:
  - self.alive_pub, self._alive_timer, and self._publish_alive()
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ── Tunable parameters ────────────────────────────────────────────────────────
CLUSTER_DISTANCE_THRESH = 0.15
MIN_CLUSTER_POINTS      = 4
MAX_CLUSTER_POINTS      = 80
MIN_CLUSTER_WIDTH       = 0.05
MAX_CLUSTER_WIDTH       = 1.2
MIN_RANGE               = 0.15
MAX_RANGE               = 8.0
WALL_RESIDUAL_THRESH    = 0.01
MAX_RANGE_VARIANCE      = 1.0
# ─────────────────────────────────────────────────────────────────────────────


class LidarClusterDetector(Node):

    def __init__(self):
        super().__init__('lidar_cluster_detector')

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Heartbeat (optional)
        self._last_scan_time = None
        self.alive_pub = self.create_publisher(Bool, '/perception_alive', 1)
        self._alive_timer = self.create_timer(1.0, self._publish_alive)

        # Use BEST_EFFORT for laser scan
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Float32MultiArray, '/detected_objects', 10)

        self.create_subscription(Odometry,  '/odom_gt', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan',    self.scan_callback, sensor_qos)

        self.get_logger().info('Lidar cluster detector running.')

    # ── Optional heartbeat ────────────────────────────────────────────────────
    def _publish_alive(self):
        msg = Bool()
        if self._last_scan_time is None:
            msg.data = False
        else:
            age = (self.get_clock().now() - self._last_scan_time).nanoseconds * 1e-9
            msg.data = (age < 1.5)  # scan seen within last 1.5 s
        self.alive_pub.publish(msg)

    # ── Odometry callback ──────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    # ── Scan callback ──────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now()

        if not self.odom_received:
            return

        # 1) Convert valid ranges to Cartesian points in ROBOT frame
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE and math.isfinite(r):
                points.append((r * math.cos(angle), r * math.sin(angle), r))
            angle += msg.angle_increment

        if len(points) < MIN_CLUSTER_POINTS:
            return

        # 2) Consecutive-distance clustering
        clusters = self._cluster_points(points)

        # 3) Reject walls / noise
        object_clusters = [c for c in clusters if self._is_object(c)]

        # 4) Centroids -> world frame (using /odom_gt)
        output_data = []
        for cluster in object_clusters:
            cx_r, cy_r = self._centroid(cluster)

            r_dist = math.hypot(cx_r, cy_r)
            theta  = math.atan2(cy_r, cx_r)

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
        current  = [points[0]]

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

        # Angular span check
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        dist = math.hypot(cx, cy)
        angles = [math.atan2(p[1], p[0]) for p in cluster]
        angular_span = abs(angles[-1] - angles[0])
        if dist > 0 and angular_span < 2 * math.atan2(MIN_CLUSTER_WIDTH * 1.5, dist):
            return False

        # Range variance check
        ranges = [p[2] for p in cluster]
        if max(ranges) - min(ranges) > MAX_RANGE_VARIANCE:
            return False

        # Straightness check
        if len(cluster) >= 6:
            xs_np = np.array(xs)
            ys_np = np.array(ys)
            coeffs  = np.polyfit(xs_np, ys_np, 1)
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