#!/usr/bin/env python3
"""
ROS 2 (rclpy) Stationary Object Detector

What this version fixes:
- No tf2_ros MessageFilter dependency (not available in your Jazzy python tf2_ros)
- Uses the LaserScan header stamp for TF lookup (correct time alignment)
- Avoids TF "extrapolation into the future" warn-spam by queueing scans until TF is ready
- Keeps your clustering + line rejection + stationarity test in world frame (odom/map)
- Keeps your sticky association gate (tracks same object instead of jumping targets)
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Point

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point


@dataclass
class Cluster:
    pts: List[Point]          # points in world frame (odom/map)
    centroid: Point
    size: int
    span_x: float
    span_y: float
    is_line_like: bool


def polar_to_xy(r: float, ang: float) -> Tuple[float, float]:
    return r * math.cos(ang), r * math.sin(ang)


class StationaryObjectDetector(Node):
    def __init__(self):
        super().__init__("stationary_object_detector")

        # ---------------- Parameters ----------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("world_frame", "odom")        # or "map"
        self.declare_parameter("max_range", 10.0)

        # clustering
        self.declare_parameter("cluster_dist", 0.20)         # meters between consecutive points
        self.declare_parameter("min_cluster_size", 6)
        self.declare_parameter("max_cluster_size", 400)

        # wall/line rejection
        self.declare_parameter("line_like_ratio", 6.0)       # larger => more likely a line
        self.declare_parameter("min_object_span", 0.05)      # ignore tiny noise

        # stationarity (in world frame)
        self.declare_parameter("stationary_window_sec", 1.5)
        self.declare_parameter("stationary_pos_tol_m", 0.10)

        # initial target selection
        self.declare_parameter("prefer_closest", True)

        # sticky association
        self.declare_parameter("association_gate_m", 0.4)
        self.declare_parameter("max_missed", 8)

        # TF gating / buffering behavior
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("pending_queue_size", 5)
        self.declare_parameter("retry_hz", 50.0)

        # Read params
        self.scan_topic = self.get_parameter("scan_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.max_range = float(self.get_parameter("max_range").value)

        self.cluster_dist = float(self.get_parameter("cluster_dist").value)
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.max_cluster_size = int(self.get_parameter("max_cluster_size").value)

        self.line_like_ratio = float(self.get_parameter("line_like_ratio").value)
        self.min_object_span = float(self.get_parameter("min_object_span").value)

        self.stationary_window_sec = float(self.get_parameter("stationary_window_sec").value)
        self.stationary_pos_tol_m = float(self.get_parameter("stationary_pos_tol_m").value)

        self.prefer_closest = bool(self.get_parameter("prefer_closest").value)

        self.association_gate_m = float(self.get_parameter("association_gate_m").value)
        self.max_missed = int(self.get_parameter("max_missed").value)

        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.pending_queue_size = int(self.get_parameter("pending_queue_size").value)
        self.retry_hz = float(self.get_parameter("retry_hz").value)

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- ROS I/O ----------------
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan_rx, 10)
        self.pub_point = self.create_publisher(PointStamped, "/object/target_point", 10)
        self.pub_stationary = self.create_publisher(Bool, "/object/stationary", 10)

        # ---------------- State ----------------
        self.centroid_hist = deque()  # (t_sec, Point) in world frame
        self.have_stationary_target = False

        self.tracked_centroid: Optional[Point] = None
        self.missed = 0

        # TF gating: queue scans until TF is ready for their timestamp
        self.pending_scans = deque(maxlen=self.pending_queue_size)
        period = 1.0 / max(1.0, self.retry_hz)
        self.retry_timer = self.create_timer(period, self.process_pending)

        # Warn-throttle helpers
        self._last_tf_warn_ns = 0
        self._tf_warn_period_ns = int(1.0 * 1e9)  # 1 Hz

        self.get_logger().info(
            f"Detector up. scan={self.scan_topic}, world_frame={self.world_frame}. "
            "Stationarity tested in world frame. TF is synchronized to scan stamp."
        )

    # ---------------- Scan handling (TF gated) ----------------

    def on_scan_rx(self, msg: LaserScan):
        # Enqueue scan; process when TF is available at msg.header.stamp
        self.pending_scans.append(msg)

    def process_pending(self):
        if not self.pending_scans:
            return

        msg = self.pending_scans[0]  # oldest first
        ok = self.try_process_scan(msg)
        if ok:
            self.pending_scans.popleft()
        # else: keep it queued (TF not ready yet)

    def try_process_scan(self, msg: LaserScan) -> bool:
        scan_frame = msg.header.frame_id
        stamp = Time.from_msg(msg.header.stamp)

        # If TF isn't available for this exact stamp yet, wait (no warn spam)
        if not self.tf_buffer.can_transform(
            self.world_frame, scan_frame, stamp, timeout=Duration(seconds=0.0)
        ):
            # Throttled debug warning (optional)
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > self._tf_warn_period_ns:
                self._last_tf_warn_ns = now_ns
                self.get_logger().warn(
                    f"Waiting for TF {self.world_frame} <- {scan_frame} at scan stamp..."
                )
            return False

        '''
        # Convert scan to points in scan frame
        pts_scan = self.scan_to_points(msg)
        if len(pts_scan) < self.min_cluster_size:
            self.publish_stationary(False)
            return True

        # Transform points to world frame at the scan timestamp
        pts_world = self.transform_points_at_stamp(scan_frame, stamp, msg.header.stamp, pts_scan)
        if not pts_world or len(pts_world) < self.min_cluster_size:
            self.publish_stationary(False)
            return True

        # Cluster in world frame
        clusters = self.cluster_points(pts_world)
        clusters = [
            c for c in clusters
            if self.min_cluster_size <= c.size <= self.max_cluster_size
            and (c.span_x > self.min_object_span or c.span_y > self.min_object_span)
            and not c.is_line_like
        ]
        '''

        # Convert scan to points in scan frame
        pts_scan = self.scan_to_points(msg)
        if len(pts_scan) < self.min_cluster_size:
            self.publish_stationary(False)
            return True

        # Cluster in SCAN frame (not world)
        clusters = self.cluster_points(pts_scan)
        clusters = [
            c for c in clusters
            if self.min_cluster_size <= c.size <= self.max_cluster_size
            and (c.span_x > self.min_object_span or c.span_y > self.min_object_span)
            and not c.is_line_like
        ]

        if not clusters:
            self.reset_track()
            self.publish_stationary(False)
            return True

        # Pick target cluster (sticky association) in SCAN frame
        #target = self.select_cluster(clusters)

        # Transform ONLY the target centroid into world frame at the scan timestamp
        #c_world = self.transform_one_point_at_stamp(scan_frame, stamp, msg.header.stamp, target.centroid)

        centroids_world = []
        for c in clusters:
            cw = self.transform_one_point_at_stamp(
                scan_frame, stamp, msg.header.stamp, c.centroid
            )
            if cw is not None:
                centroids_world.append((c, cw))

        if not centroids_world:
            self.reset_track()
            self.publish_stationary(False)
            return True

        # Select target in WORLD frame (sticky in odom/map)
        target, c_world = self.select_cluster_world(centroids_world)

        # Update stationarity history in world frame using scan time
        now = stamp.nanoseconds / 1e9
        self.centroid_hist.append((now, c_world))
        while self.centroid_hist and (now - self.centroid_hist[0][0]) > self.stationary_window_sec:
            self.centroid_hist.popleft()

        stationary = self.is_stationary()
        self.have_stationary_target = stationary

        if stationary:
            self.get_logger().info(
                f"Stationary object @ {self.world_frame}: x={c_world.x:.3f}, y={c_world.y:.3f}, z={c_world.z:.3f}"
            )
            self.publish_target(c_world, self.world_frame, msg.header.stamp)

        self.publish_stationary(stationary)
        return True

    def select_cluster_world(self, clusters_world):
        # clusters_world: [(cluster, world_centroid), ...]

        if self.tracked_centroid is not None:
            tx, ty = self.tracked_centroid.x, self.tracked_centroid.y

            best_c, best_w = min(
                clusters_world,
                key=lambda cw: (cw[1].x - tx) ** 2 + (cw[1].y - ty) ** 2
            )

            d2 = (best_w.x - tx) ** 2 + (best_w.y - ty) ** 2

            if d2 <= self.association_gate_m ** 2:
                self.missed = 0
                self.tracked_centroid = best_w
                return best_c, best_w

            self.missed += 1
            if self.missed <= self.max_missed:
                return best_c, best_w
            else:
                self.tracked_centroid = None
                self.missed = 0

        # No track yet
        if self.prefer_closest:
            best_c, best_w = min(
                clusters_world,
                key=lambda cw: math.hypot(cw[1].x, cw[1].y)
            )
        else:
            best_c, best_w = max(
                clusters_world,
                key=lambda cw: cw[0].size
            )

        self.tracked_centroid = best_w
        self.missed = 0
        return best_c, best_w


    # ---------------- Core logic ----------------

    def scan_to_points(self, msg: LaserScan) -> List[Point]:
        pts = []
        ang = msg.angle_min
        max_r = min(msg.range_max, self.max_range)
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < max_r:
                x, y = polar_to_xy(r, ang)
                pts.append(Point(x=x, y=y, z=0.0))
            ang += msg.angle_increment
        return pts

    def transform_one_point_at_stamp(
        self,
        from_frame: str,
        stamp: Time,      # rclpy Time
        stamp_msg,        # builtin_interfaces/Time from msg.header.stamp
        p: Point,
    ) -> Optional[Point]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                from_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as ex:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > self._tf_warn_period_ns:
                self._last_tf_warn_ns = now_ns
                self.get_logger().warn(f"TF lookup failed {self.world_frame} <- {from_frame}: {ex}")
            return None

        ps = PointStamped()
        ps.header.frame_id = from_frame
        ps.header.stamp = stamp_msg
        ps.point = p
        return do_transform_point(ps, tf).point


    def transform_points_at_stamp(
        self,
        from_frame: str,
        stamp: Time,
        stamp_msg,  # builtin_interfaces/Time from msg.header.stamp
        pts: List[Point],
    ) -> Optional[List[Point]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                from_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )

        except TransformException as ex:
            # Throttle warnings
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > self._tf_warn_period_ns:
                self._last_tf_warn_ns = now_ns
                self.get_logger().warn(f"TF lookup failed {self.world_frame} <- {from_frame}: {ex}")
            return None

        ps = PointStamped()
        ps.header.frame_id = from_frame
        ps.header.stamp = stamp_msg

        out = []
        for p in pts:
            ps.point = p
            out.append(do_transform_point(ps, tf).point)
        return out

    def cluster_points(self, pts: List[Point]) -> List[Cluster]:
        if not pts:
            return []

        # 1D scan-order clustering; TF keeps ordering mostly intact
        clusters_raw = []
        cur = [pts[0]]
        for p in pts[1:]:
            prev = cur[-1]
            d = math.hypot(p.x - prev.x, p.y - prev.y)
            if d < self.cluster_dist:
                cur.append(p)
            else:
                clusters_raw.append(cur)
                cur = [p]
        clusters_raw.append(cur)

        clusters: List[Cluster] = []
        for group in clusters_raw:
            if len(group) < self.min_cluster_size:
                continue

            cx = sum(p.x for p in group) / len(group)
            cy = sum(p.y for p in group) / len(group)
            minx = min(p.x for p in group)
            maxx = max(p.x for p in group)
            miny = min(p.y for p in group)
            maxy = max(p.y for p in group)
            span_x = maxx - minx
            span_y = maxy - miny

            big = max(span_x, span_y)
            small = max(1e-6, min(span_x, span_y))
            ratio = big / small
            is_line_like = ratio > self.line_like_ratio

            clusters.append(
                Cluster(
                    pts=group,
                    centroid=Point(x=cx, y=cy, z=0.0),
                    size=len(group),
                    span_x=span_x,
                    span_y=span_y,
                    is_line_like=is_line_like,
                )
            )
        return clusters

    def select_cluster(self, clusters: List[Cluster]) -> Cluster:
        # Sticky tracking: choose closest to last tracked centroid if within gate
        if self.tracked_centroid is not None:
            tx, ty = self.tracked_centroid.x, self.tracked_centroid.y
            best = min(clusters, key=lambda c: (c.centroid.x - tx) ** 2 + (c.centroid.y - ty) ** 2)
            d2 = (best.centroid.x - tx) ** 2 + (best.centroid.y - ty) ** 2

            if d2 <= self.association_gate_m ** 2:
                self.missed = 0
                self.tracked_centroid = best.centroid
                return best

            # association failed: coast for a few frames before giving up
            self.missed += 1
            if self.missed <= self.max_missed:
                return best  # don't update tracked_centroid
            else:
                self.tracked_centroid = None
                self.missed = 0

        # No track yet: initialize
        if self.prefer_closest:
            best = min(clusters, key=lambda c: math.hypot(c.centroid.x, c.centroid.y))
        else:
            best = max(clusters, key=lambda c: c.size)

        self.tracked_centroid = best.centroid
        self.missed = 0
        return best

    def is_stationary(self) -> bool:
        if not self.centroid_hist:
            return False
        t0 = self.centroid_hist[0][0]
        t1 = self.centroid_hist[-1][0]
        if (t1 - t0) < self.stationary_window_sec * 0.9:
            return False

        xs = [p.x for _, p in self.centroid_hist]
        ys = [p.y for _, p in self.centroid_hist]
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)

        max_dev = 0.0
        for x, y in zip(xs, ys):
            max_dev = max(max_dev, math.hypot(x - mx, y - my))

        return max_dev <= self.stationary_pos_tol_m

    # ---------------- Publishers / reset ----------------

    def publish_target(self, centroid: Point, frame: str, stamp_msg):
        msg = PointStamped()
        msg.header.frame_id = frame
        msg.header.stamp = stamp_msg          # <-- scan stamp, not now()
        msg.point = centroid
        self.pub_point.publish(msg)


    def publish_stationary(self, val: bool):
        self.pub_stationary.publish(Bool(data=val))

    def reset_track(self):
        self.centroid_hist.clear()
        self.have_stationary_target = False
        self.tracked_centroid = None
        self.missed = 0


def main():
    rclpy.init()
    node = StationaryObjectDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
