#!/usr/bin/env python3
"""
ROS 2 (rclpy) Stationary Object Detector (NOT cylinder-specific)

Detects a nearby object from 2D LiDAR by:
- clustering contiguous valid scan points
- rejecting wall-like clusters (too wide)
- selecting the nearest remaining cluster
- declaring it stationary if its closest-point position drifts less than a tolerance for N seconds

Publishes:
- /object/stationary (std_msgs/Bool)
- /object/range      (std_msgs/Float32)  nearest point range
- /object/bearing    (std_msgs/Float32)  nearest point bearing
- /object/point      (geometry_msgs/PointStamped) nearest point in laser frame

Tune:
- wall_width_max_m (main "not a wall" knob)
- stationary_pos_tol_m, stationary_required_sec
"""

import math
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped


def is_valid_range(r: float) -> bool:
    return math.isfinite(r) and r > 0.0


def polar_to_xy(r: float, theta: float) -> Tuple[float, float]:
    return (r * math.cos(theta), r * math.sin(theta))


class StationaryObjectDetector(Node):
    def __init__(self):
        super().__init__("stationary_object_detector")

        # -------- Params --------
        self.declare_parameter("scan_topic", "/scan")

        # Range gating
        self.declare_parameter("range_min_m", 0.15)
        self.declare_parameter("range_max_m", 4.0)

        # Clustering
        self.declare_parameter("cluster_jump_m", 0.10)
        self.declare_parameter("min_cluster_points", 6)

        # "Not a wall" heuristics
        self.declare_parameter("wall_width_max_m", 1.80)     # clusters wider than this are treated as wall-like
        self.declare_parameter("wall_angle_span_max_deg", 55.0)  # clusters spanning too much angle are wall-like

        # Stationary check
        self.declare_parameter("stationary_required_sec", 1.0)
        self.declare_parameter("stationary_pos_tol_m", 0.04)

        # Output topics
        self.declare_parameter("stationary_topic", "/object/stationary")
        self.declare_parameter("range_topic", "/object/range")
        self.declare_parameter("bearing_topic", "/object/bearing")
        self.declare_parameter("point_topic", "/object/point")

        # Read params
        self.scan_topic = self.get_parameter("scan_topic").value
        self.rmin = float(self.get_parameter("range_min_m").value)
        self.rmax = float(self.get_parameter("range_max_m").value)

        self.cluster_jump_m = float(self.get_parameter("cluster_jump_m").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)

        self.wall_width_max_m = float(self.get_parameter("wall_width_max_m").value)
        self.wall_angle_span_max = math.radians(float(self.get_parameter("wall_angle_span_max_deg").value))

        self.stationary_required_sec = float(self.get_parameter("stationary_required_sec").value)
        self.stationary_pos_tol_m = float(self.get_parameter("stationary_pos_tol_m").value)

        self.stationary_topic = self.get_parameter("stationary_topic").value
        self.range_topic = self.get_parameter("range_topic").value
        self.bearing_topic = self.get_parameter("bearing_topic").value
        self.point_topic = self.get_parameter("point_topic").value

        # Pub/Sub
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.pub_stationary = self.create_publisher(Bool, self.stationary_topic, 10)
        self.pub_range = self.create_publisher(Float32, self.range_topic, 10)
        self.pub_bearing = self.create_publisher(Float32, self.bearing_topic, 10)
        self.pub_point = self.create_publisher(PointStamped, self.point_topic, 10)

        # Track nearest point position in laser frame: (t, x, y)
        self.track = deque(maxlen=60)

        self.get_logger().info(
            f"Started. scan={self.scan_topic} -> {self.stationary_topic}, ignoring wall-like clusters "
            f"(width>{self.wall_width_max_m}m or angle_span>{math.degrees(self.wall_angle_span_max):.1f}deg)"
        )

    def publish_no_target(self):
        self.track.clear()
        self.pub_stationary.publish(Bool(data=False))

    def is_stationary(self) -> bool:
        if len(self.track) < 6:
            return False
        t0, x0, y0 = self.track[0]
        t1, x1, y1 = self.track[-1]
        if (t1 - t0) < self.stationary_required_sec:
            return False
        drift = math.hypot(x1 - x0, y1 - y0)
        return drift <= self.stationary_pos_tol_m

    def on_scan(self, scan: LaserScan):
        # Build list of valid points (x,y,theta,r) or None to break clusters
        pts: List[Optional[Tuple[float, float, float, float]]] = []
        theta = scan.angle_min
        rmax = min(self.rmax, float(scan.range_max))

        for r in scan.ranges:
            if is_valid_range(r) and self.rmin <= r <= rmax:
                x, y = polar_to_xy(r, theta)
                pts.append((x, y, theta, r))
            else:
                pts.append(None)
            theta += scan.angle_increment

        # Cluster contiguous points by jump distance
        clusters: List[List[Tuple[float, float, float, float]]] = []
        cur: List[Tuple[float, float, float, float]] = []
        prev_xy: Optional[Tuple[float, float]] = None

        for p in pts:
            if p is None:
                if len(cur) >= self.min_cluster_points:
                    clusters.append(cur)
                cur = []
                prev_xy = None
                continue

            x, y, th, r = p
            if prev_xy is None:
                cur.append(p)
            else:
                if math.hypot(x - prev_xy[0], y - prev_xy[1]) <= self.cluster_jump_m:
                    cur.append(p)
                else:
                    if len(cur) >= self.min_cluster_points:
                        clusters.append(cur)
                    cur = [p]
            prev_xy = (x, y)

        if len(cur) >= self.min_cluster_points:
            clusters.append(cur)

        if not clusters:
            self.publish_no_target()
            return

        # For each cluster, compute "wall-likeness" + nearest point
        best = None  # (nearest_range, nearest_x, nearest_y, nearest_theta)
        for c in clusters:
            # width: distance between endpoints
            x0, y0, th0, r0 = c[0]
            x1, y1, th1, r1 = c[-1]
            width = math.hypot(x1 - x0, y1 - y0)
            angle_span = abs(th1 - th0)

            # Reject wall-like clusters
            if width > self.wall_width_max_m:
                continue
            if angle_span > self.wall_angle_span_max:
                continue

            # Find nearest point in this cluster
            nearest = min(c, key=lambda p: p[3])  # by r
            nx, ny, nth, nr = nearest

            if best is None or nr < best[0]:
                best = (nr, nx, ny, nth)

        if best is None:
            self.publish_no_target()
            return

        nr, nx, ny, nth = best

        # Update stationary track using nearest point position in laser frame
        now = self.get_clock().now().nanoseconds * 1e-9
        self.track.append((now, nx, ny))
        stationary = self.is_stationary()

        # Publish
        self.pub_stationary.publish(Bool(data=bool(stationary)))
        self.pub_range.publish(Float32(data=float(nr)))
        self.pub_bearing.publish(Float32(data=float(nth)))

        pt = PointStamped()
        pt.header = scan.header
        pt.point.x = float(nx)
        pt.point.y = float(ny)
        pt.point.z = 0.0
        self.pub_point.publish(pt)


def main():
    rclpy.init()
    node = StationaryObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
