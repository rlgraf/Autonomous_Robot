#!/usr/bin/env python3
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Deque, Dict, Tuple, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PointStamped


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Odom2D:
    x: float
    y: float
    yaw: float
    stamp_sec: float


@dataclass
class Track:
    xy: Tuple[float, float]                     # last centroid (world)
    last_t: float
    hist: Deque[Tuple[float, float, float]]     # (t, xW, yW)


class DebounceBool:
    """
    Time-based debouncer.
    - require raw True to hold for on_delay before output becomes True
    - require raw False to hold for off_delay before output becomes False
    """
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

        # raw differs from state; start/continue timing candidate transition
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


class WorldFrameStationaryClusterDetector(Node):
    def __init__(self):
        super().__init__("identify")

        # ---------------- Params / Tunables ----------------
        # scan / filtering
        self.range_threshold_m = 10.0
        self.min_valid_range = 0.02

        # clustering
        self.cluster_jump_m = 0.20
        self.min_cluster_points = 5

        # association / tracking
        self.assoc_gate_m = 0.60
        self.track_timeout_sec = 1.0

        # stationarity test
        self.stationary_window_sec = 3.0
        self.stationary_tol_m = 0.08

        # choose which stationary object to report
        # "closest" is typical
        self.select_mode = "closest"  # "closest" or "oldest"

        # debouncing published stationary bool (prevents flicker)
        self.stationary_on_delay = 0.40
        self.stationary_off_delay = 0.60
        self._debounce = DebounceBool(self.stationary_on_delay, self.stationary_off_delay)

        # LiDAR extrinsic in base (keep 0 if LiDAR already in base frame)
        self.lidar_in_base_x = 0.0
        self.lidar_in_base_y = 0.0
        self.lidar_in_base_yaw = 0.0

        # ---------------- ROS I/O ----------------
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom_gt", self.odom_callback, 50)

        self.stationary_pub = self.create_publisher(Bool, "/object_stationary", 10)
        self.target_pub = self.create_publisher(PointStamped, "/stationary_target", 10)
        self.track_id_pub = self.create_publisher(Int32, "/stationary_track_id", 10)

        # ---------------- State ----------------
        self.latest_odom: Optional[Odom2D] = None
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1

        self._warned_no_odom = False
        self._last_reported_tid: Optional[int] = None

        self.get_logger().info("World-frame stationary CLUSTER detector running (debounced output)")

    # ---------------- helpers ----------------
    def publish_stationary(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self.stationary_pub.publish(msg)

    def publish_target(self, t_stamp, frame_id: str, x: float, y: float):
        pt = PointStamped()
        pt.header.stamp = t_stamp
        pt.header.frame_id = frame_id
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = 0.0
        self.target_pub.publish(pt)

    def publish_track_id(self, tid: int):
        m = Int32()
        m.data = int(tid)
        self.track_id_pub.publish(m)

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
        # LiDAR polar -> LiDAR Cartesian
        xL = r * math.cos(theta_lidar)
        yL = r * math.sin(theta_lidar)

        # LiDAR -> Base
        cy = math.cos(self.lidar_in_base_yaw)
        sy = math.sin(self.lidar_in_base_yaw)
        xB = cy * xL - sy * yL + self.lidar_in_base_x
        yB = sy * xL + cy * yL + self.lidar_in_base_y

        # Base -> World (odom_gt)
        c = math.cos(odom.yaw)
        s = math.sin(odom.yaw)
        xW = c * xB - s * yB + odom.x
        yW = s * xB + c * yB + odom.y
        return xW, yW

    def _cluster_indices(self, msg: LaserScan) -> List[List[int]]:
        clusters: List[List[int]] = []
        cur: List[int] = []
        prev_r = None

        for i, r in enumerate(msg.ranges):
            valid = (
                (not math.isinf(r)) and (not math.isnan(r)) and
                (self.min_valid_range <= r <= self.range_threshold_m)
            )

            if not valid:
                if cur:
                    clusters.append(cur)
                    cur = []
                prev_r = None
                continue

            if prev_r is None:
                cur = [i]
            else:
                if abs(r - prev_r) > self.cluster_jump_m:
                    if cur:
                        clusters.append(cur)
                    cur = [i]
                else:
                    cur.append(i)

            prev_r = r

        if cur:
            clusters.append(cur)

        clusters = [c for c in clusters if len(c) >= self.min_cluster_points]
        return clusters

    def _cluster_centroid_world(self, msg: LaserScan, idxs: List[int], odom: Odom2D) -> Tuple[float, float]:
        sx = 0.0
        sy = 0.0
        n = 0
        for i in idxs:
            r = msg.ranges[i]
            theta = msg.angle_min + i * msg.angle_increment
            xW, yW = self.lidar_point_to_world(r, theta, odom)
            sx += xW
            sy += yW
            n += 1
        return (sx / n, sy / n)

    def _associate_tracks(self, t_scan: float, detections_xy: List[Tuple[float, float]]):
        unmatched_tracks = set(self.tracks.keys())
        used_dets = set()

        # Greedy matching: for each detection, match nearest track within gate
        for di, (dx, dy) in enumerate(detections_xy):
            best_tid = None
            best_d = float("inf")
            for tid in list(unmatched_tracks):
                tx, ty = self.tracks[tid].xy
                d = math.hypot(dx - tx, dy - ty)
                if d < best_d:
                    best_d = d
                    best_tid = tid

            if best_tid is not None and best_d <= self.assoc_gate_m:
                tr = self.tracks[best_tid]
                tr.xy = (dx, dy)
                tr.last_t = t_scan
                tr.hist.append((t_scan, dx, dy))
                unmatched_tracks.remove(best_tid)
                used_dets.add(di)

        # New tracks for unmatched detections
        for di, (dx, dy) in enumerate(detections_xy):
            if di in used_dets:
                continue
            tid = self.next_track_id
            self.next_track_id += 1
            self.tracks[tid] = Track(
                xy=(dx, dy),
                last_t=t_scan,
                hist=deque(maxlen=2000),
            )
            self.tracks[tid].hist.append((t_scan, dx, dy))

        # Prune stale tracks
        to_del = []
        for tid, tr in self.tracks.items():
            if (t_scan - tr.last_t) > self.track_timeout_sec:
                to_del.append(tid)
        for tid in to_del:
            del self.tracks[tid]

        # Trim histories to stationary window
        for tr in self.tracks.values():
            while tr.hist and (t_scan - tr.hist[0][0]) > self.stationary_window_sec:
                tr.hist.popleft()

    def _track_stationary(self, tr: Track) -> Tuple[bool, float, float, float]:
        if len(tr.hist) < 2:
            return (False, float("inf"), 0.0, 0.0)
        if (tr.hist[-1][0] - tr.hist[0][0]) < self.stationary_window_sec:
            return (False, float("inf"), 0.0, 0.0)

        xs = [p[1] for p in tr.hist]
        ys = [p[2] for p in tr.hist]
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)

        max_dev = 0.0
        for x, y in zip(xs, ys):
            d = math.hypot(x - mx, y - my)
            if d > max_dev:
                max_dev = d

        return (max_dev <= self.stationary_tol_m, max_dev, mx, my)

    def _select_stationary_track(self, odom: Odom2D) -> Optional[Tuple[int, float, float, float]]:
        """
        Return (tid, dist_to_robot, mean_x, mean_y) for best stationary track.
        """
        rx, ry = odom.x, odom.y
        best = None

        for tid, tr in self.tracks.items():
            is_stat, max_dev, mx, my = self._track_stationary(tr)
            if not is_stat:
                continue

            d_robot = math.hypot(mx - rx, my - ry)

            if self.select_mode == "closest":
                key = d_robot
            else:
                # "oldest": prefer most samples (or longest time span)
                key = -len(tr.hist)

            if best is None or key < best[0]:
                best = (key, tid, d_robot, mx, my, max_dev, len(tr.hist))

        if best is None:
            return None

        _, tid, d_robot, mx, my, max_dev, n = best
        return (tid, d_robot, mx, my)

    # ---------------- main callback ----------------
    def scan_callback(self, msg: LaserScan):
        if self.latest_odom is None:
            if not self._warned_no_odom:
                self.get_logger().warn("No /odom_gt received yet; publishing False.")
                self._warned_no_odom = True
            self.publish_stationary(False)
            return
        self._warned_no_odom = False

        odom = self.latest_odom
        t_scan = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

        clusters = self._cluster_indices(msg)
        if not clusters:
            out = self._debounce.update(False, t_scan)
            self.publish_stationary(out)
            return

        detections_xy: List[Tuple[float, float]] = []
        for idxs in clusters:
            cx, cy = self._cluster_centroid_world(msg, idxs, odom)
            detections_xy.append((cx, cy))

        self._associate_tracks(t_scan, detections_xy)

        choice = self._select_stationary_track(odom)
        raw_stationary = (choice is not None)

        out_stationary = self._debounce.update(raw_stationary, t_scan)
        self.publish_stationary(out_stationary)

        if not out_stationary or choice is None:
            self._last_reported_tid = None
            return

        tid, d_robot, mx, my = choice

        # Publish target + track id
        self.publish_target(msg.header.stamp, "odom", mx, my)
        self.publish_track_id(tid)

        # Log only when track changes or occasionally (optional)
        if self._last_reported_tid != tid:
            self._last_reported_tid = tid
            self.get_logger().info(
                f"STATIONARY SELECT tid={tid} dist={d_robot:.2f}m mean=({mx:.2f},{my:.2f}) "
                f"tracks={len(self.tracks)} clusters={len(clusters)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WorldFrameStationaryClusterDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()