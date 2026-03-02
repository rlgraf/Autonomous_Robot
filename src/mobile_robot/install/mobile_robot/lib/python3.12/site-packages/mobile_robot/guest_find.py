#!/usr/bin/env python3
"""
ROS 2 (rclpy) Stationary Object Detector (object-tracking + robust stationarity)

Fix B: Sticky tracking / data association:
- Once a target is being tracked, pick the cluster whose centroid (in target_frame)
  is closest to last known target position (within a gate).
- Only fall back to "nearest cluster" when no track exists or track is lost.

Fix C: Robust stationarity test:
- Track centroid position in target_frame over time.
- Declare stationary if:
  - window duration >= stationary_required_sec
  - max deviation from the mean across the window <= stationary_pos_tol_m
"""

import math
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped

from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


def is_valid_range(r: float) -> bool:
    return math.isfinite(r) and r > 0.0


def polar_to_xy(r: float, theta: float) -> Tuple[float, float]:
    return (r * math.cos(theta), r * math.sin(theta))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


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
        self.declare_parameter("wall_width_max_m", 1.80)
        self.declare_parameter("wall_angle_span_max_deg", 55.0)

        # Stationary check (robust)
        self.declare_parameter("stationary_required_sec", 1.0)
        self.declare_parameter("stationary_pos_tol_m", 0.04)
        self.declare_parameter("stationary_min_samples", 10)

        # Tracking / data association (Fix B)
        self.declare_parameter("track_gate_m", 0.35)          # how far target can move (in target_frame) between scans
        self.declare_parameter("lost_release_scans", 8)       # how many scans with no associated cluster before dropping track

        # Output topics
        self.declare_parameter("stationary_topic", "/object/stationary")
        self.declare_parameter("range_topic", "/object/range")
        self.declare_parameter("bearing_topic", "/object/bearing")
        self.declare_parameter("point_topic", "/object/point")

        # Fixed/world-ish frame to evaluate stationarity in
        self.declare_parameter("target_frame", "odom")
        self.target_frame = self.get_parameter("target_frame").value

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- TF warmup / startup grace ---
        self.declare_parameter("tf_warmup_grace_sec", 1.0)
        self.tf_warmup_grace_sec = float(self.get_parameter("tf_warmup_grace_sec").value)
        self._start_time = self.get_clock().now()

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
        self.stationary_min_samples = int(self.get_parameter("stationary_min_samples").value)

        self.track_gate_m = float(self.get_parameter("track_gate_m").value)
        self.lost_release_scans = int(self.get_parameter("lost_release_scans").value)

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

        # Track centroid position in target_frame: deque of (t_sec, x, y)
        self.track = deque(maxlen=120)

        # Sticky target state (Fix B)
        self._has_target = False
        self._last_target_xy = (0.0, 0.0)
        self._lost_streak = 0

        self.get_logger().info(
            f"Started. scan={self.scan_topic} -> {self.stationary_topic}. "
            f"Sticky tracking gate={self.track_gate_m:.2f}m, "
            f"wall reject width>{self.wall_width_max_m:.2f}m or angle_span>{math.degrees(self.wall_angle_span_max):.1f}deg"
        )

    def publish_no_target(self):
        # Only clear if we truly lost track for a while; otherwise don't nuke stationarity window on a TF hiccup.
        self.pub_stationary.publish(Bool(data=False))

    def drop_track(self, reason: str):
        if self._has_target:
            self.get_logger().info(f"Dropping target: {reason}")
        self._has_target = False
        self._lost_streak = 0
        self.track.clear()
        self.pub_stationary.publish(Bool(data=False))

    def is_stationary(self) -> bool:
        """Fix C: robust stationarity using max deviation from mean across window."""
        if len(self.track) < max(6, self.stationary_min_samples):
            return False

        t0 = self.track[0][0]
        t1 = self.track[-1][0]
        if (t1 - t0) < self.stationary_required_sec:
            return False

        xs = [p[1] for p in self.track]
        ys = [p[2] for p in self.track]
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)

        max_dev = 0.0
        for (_, x, y) in self.track:
            d = math.hypot(x - mx, y - my)
            if d > max_dev:
                max_dev = d

        return max_dev <= self.stationary_pos_tol_m

    def on_scan(self, scan: LaserScan):
        # Startup grace: let TF buffer fill so we don't get "future extrapolation" at node start
        if (self.get_clock().now() - self._start_time) < Duration(seconds=self.tf_warmup_grace_sec):
            return

        # Use scan timestamp for consistent transforms
        # Use latest available TF (best for sim-time systems where timestamps may not align)
        t = Time()  # "latest"

        if not self.tf_buffer.can_transform(
            self.target_frame,
            scan.header.frame_id,
            t,
            timeout=Duration(seconds=0.3),
        ):
            self.get_logger().warn("TF not ready (latest). Skipping this scan.")
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                t,
                timeout=Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed (skipping scan): {ex}")
            return



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
            self._lost_streak += 1
            if self._has_target and self._lost_streak >= self.lost_release_scans:
                self.drop_track("no clusters")
            else:
                self.publish_no_target()
            return

        # Build candidate list with features for association
        # Each candidate: dict with centroid (laser), centroid (target), nearest point (laser), nearest range/bearing
        candidates = []
        for c in clusters:
            # wall-like tests
            x0, y0, th0, r0 = c[0]
            x1, y1, th1, r1 = c[-1]
            width = math.hypot(x1 - x0, y1 - y0)
            angle_span = abs(th1 - th0)
            if width > self.wall_width_max_m:
                continue
            if angle_span > self.wall_angle_span_max:
                continue

            # centroid in laser frame
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)

            # nearest point in cluster (laser frame) for your range/bearing outputs
            nx, ny, nth, nr = min(c, key=lambda p: p[3])

            # transform centroid to target_frame
            centroid_pt = PointStamped()
            centroid_pt.header = scan.header
            centroid_pt.point.x = float(cx)
            centroid_pt.point.y = float(cy)
            centroid_pt.point.z = 0.0

            try:
                centroid_world = do_transform_point(centroid_pt, tf)
            except TransformException as ex:
                # should be rare since tf already fetched, but be safe
                continue

            wx = float(centroid_world.point.x)
            wy = float(centroid_world.point.y)

            candidates.append(
                {
                    "centroid_laser": (cx, cy),
                    "centroid_world": (wx, wy),
                    "nearest_laser": (nx, ny),
                    "nearest_range": nr,
                    "nearest_bearing": nth,
                }
            )

        if not candidates:
            self._lost_streak += 1
            if self._has_target and self._lost_streak >= self.lost_release_scans:
                self.drop_track("all clusters rejected")
            else:
                self.publish_no_target()
            return

        # ---------- Fix B: Sticky association ----------
        chosen = None

        if self._has_target:
            lastx, lasty = self._last_target_xy
            # choose candidate with closest centroid to previous target
            best_d = float("inf")
            best_c = None
            for c in candidates:
                wx, wy = c["centroid_world"]
                d = math.hypot(wx - lastx, wy - lasty)
                if d < best_d:
                    best_d = d
                    best_c = c

            # accept only if within gate
            if best_c is not None and best_d <= self.track_gate_m:
                chosen = best_c
            else:
                # temporarily treat as lost (do not immediately jump targets)
                self._lost_streak += 1
                if self._lost_streak >= self.lost_release_scans:
                    # fully lost -> allow reacquire
                    self._has_target = False
                else:
                    # keep current track window but do not append new data this scan
                    self.pub_stationary.publish(Bool(data=self.is_stationary()))
                    return

        # If no target (or we just dropped), acquire by nearest range
        if chosen is None:
            chosen = min(candidates, key=lambda c: c["nearest_range"])
            self._has_target = True
            self._lost_streak = 0
            # reset stationarity window on a fresh acquisition
            self.track.clear()

        # Update target state
        wx, wy = chosen["centroid_world"]
        self._last_target_xy = (wx, wy)
        self._lost_streak = 0

        # Publish nearest point in laser frame (same as before)
        nx, ny = chosen["nearest_laser"]
        nr = chosen["nearest_range"]
        nth = chosen["nearest_bearing"]

        pt = PointStamped()
        pt.header = scan.header
        pt.point.x = float(nx)
        pt.point.y = float(ny)
        pt.point.z = 0.0
        self.pub_point.publish(pt)

        # Track centroid in target_frame (more stable)
        stamp_sec = self.get_clock().now().nanoseconds * 1e-9

        self.track.append((stamp_sec, wx, wy))

        stationary = self.is_stationary()

        self.pub_stationary.publish(Bool(data=bool(stationary)))
        self.pub_range.publish(Float32(data=float(nr)))
        self.pub_bearing.publish(Float32(data=float(nth)))


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
