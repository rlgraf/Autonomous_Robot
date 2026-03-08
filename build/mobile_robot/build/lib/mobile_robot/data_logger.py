#!/usr/bin/env python3
"""
Data Logger Node - Robust Recharge Trip + Charging Metrics
- Logs experiment summary to a TSV (tab-delimited) file on shutdown
- Tracks:
  * visited cylinders + travel times
  * recharge trips (start -> arrive -> charging start -> charging end/full)
  * total time to stations, total charging time, battery deltas
  * robot position sampled every second for path plotting
"""

import csv
import math
import os
from typing import Optional, Dict, Any, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

import yaml
from ament_index_python.packages import get_package_share_directory


def _now_s(clock, start_time: Time) -> float:
    return (clock.now() - start_time).nanoseconds * 1e-9


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw in radians from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__("data_logger")

        # ------------------------- Load configuration -------------------------
        pkg = get_package_share_directory("mobile_robot")
        battery_yaml = os.path.join(pkg, "parameters", "battery_tunable_parameters.yaml")

        with open(battery_yaml, "r") as f:
            config = yaml.safe_load(f)

        bat_params = config["battery_node"]["ros__parameters"]
        rech_params = config["auto_recharge_node"]["ros__parameters"]

        self.charging_stations: List[Tuple[float, float]] = [tuple(s) for s in bat_params["charging_stations"]]
        self.max_linear: float = float(rech_params["max_linear"])
        self.max_angular: float = float(rech_params["max_angular"])
        self.dwell_time: float = 10.0  # keep in sync with your navigator DWELL_TIME

        # Load cylinder positions (cache)
        self.cylinder_positions = self._load_cylinder_positions()

        # ------------------------- Runtime data -------------------------
        self.visited_cylinders: List[Tuple[float, float, Time]] = []  # (x,y,arrival_time)
        self.last_visit_time: Optional[Time] = None
        self.total_travel_time: float = 0.0

        # Recharge trips
        self.recharge_trips: List[Dict[str, Any]] = []
        self.current_trip: Optional[Dict[str, Any]] = None
        self.last_recharge_active: bool = False

        # Battery tracking
        self.current_battery_pct: float = 1.0
        self.prev_battery_pct: float = 1.0

        # Charging detection
        self.currently_charging: bool = False
        self.charge_start_time: Optional[Time] = None
        self.battery_at_charge_start: Optional[float] = None

        # Fallback trend-based charging detection (if status is missing)
        self._trend_inc_count: int = 0
        self._trend_dec_count: int = 0
        self._trend_confirm_needed: int = 5
        self._trend_stop_needed: int = 10
        self._trend_inc_thresh: float = 0.0005
        self._trend_stop_thresh: float = 0.0001

        # Totals
        self.total_charging_time: float = 0.0

        # Timing
        self.start_time: Time = self.get_clock().now()

        # ------------------------- Path tracking -------------------------
        self.latest_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.path_samples: List[Tuple[float, float, float, float]] = []  # (t, x, y, yaw)

        # ------------------------- ROS I/O -------------------------
        self.create_subscription(Float32MultiArray, "/visited_cylinders", self._visited_callback, 10)
        self.create_subscription(Bool, "/recharge_active", self._recharge_callback, 10)
        self.create_subscription(BatteryState, "battery_status", self._battery_callback, 10)

        # Choose whichever odometry topic your robot actually publishes
        # Replace "/gt_odom" with your real topic if needed.
        self.create_subscription(Odometry, "/odom_gt", self._odom_callback, 10)

        # Sample pose once per second
        self.create_timer(1.0, self._path_sample_timer_callback)

        # ------------------------- Output file -------------------------
        data_dir = os.path.expanduser("~/Autonomous_Robot/src/mobile_robot/data")
        os.makedirs(data_dir, exist_ok=True)
        self.csv_file = os.path.join(data_dir, "standard_2.csv")  # tab-delimited TSV

        self.get_logger().info(
            f"Data logger ready. Tracking {len(self.cylinder_positions)} cylinders. "
            f"Logging to: {self.csv_file}"
        )

    # -------------------------------------------------------------------------
    # Data loading
    # -------------------------------------------------------------------------
    def _load_cylinder_positions(self) -> List[Tuple[float, float]]:
        data_dir = os.path.expanduser("~/Autonomous_Robot/src/mobile_robot/data")
        cache_file = os.path.join(data_dir, "cylinder_positions.txt")
        cylinders: List[Tuple[float, float]] = []

        if not os.path.exists(cache_file):
            self.get_logger().warn(f"Cylinder cache not found: {cache_file}")
            return cylinders

        try:
            with open(cache_file, "r") as f:
                for line in f:
                    x, y = line.strip().split(",")
                    cylinders.append((float(x), float(y)))
            self.get_logger().info(f"Loaded {len(cylinders)} cylinders from cache")
        except Exception as e:
            self.get_logger().error(f"Failed to load cylinders: {e}")

        return cylinders

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry):
        """Cache latest robot pose from odometry."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.latest_pose = (float(p.x), float(p.y), float(yaw))

    def _path_sample_timer_callback(self):
        """Record one path sample per second using the latest cached pose."""
        if self.latest_pose is None:
            return

        t = _now_s(self.get_clock(), self.start_time)
        x, y, yaw = self.latest_pose
        self.path_samples.append((t, x, y, yaw))

    def _visited_callback(self, msg: Float32MultiArray):
        """Record when robot completes dwelling at a cylinder."""
        if len(msg.data) < 2:
            return

        x, y = float(msg.data[0]), float(msg.data[1])
        now = self.get_clock().now()

        if self.last_visit_time is not None:
            dt = (now - self.last_visit_time).nanoseconds * 1e-9
            dt -= self.dwell_time
            self.total_travel_time += max(0.0, dt)

        self.visited_cylinders.append((x, y, now))
        self.last_visit_time = now

        self.get_logger().info(f"Cylinder visited at ({x:.2f}, {y:.2f}) - Total: {len(self.visited_cylinders)}")

    def _recharge_callback(self, msg: Bool):
        now = self.get_clock().now()

        if msg.data and not self.last_recharge_active:
            if self.current_trip is not None:
                self.get_logger().warn(
                    f"New recharge started while trip #{self.current_trip.get('trip_number')} still open; "
                    "finalizing previous trip."
                )
                self._finalize_current_trip(reason="overlap_start")

            trip_number = len(self.recharge_trips) + 1
            trip: Dict[str, Any] = {
                "trip_number": trip_number,
                "trip_start_time": now,
                "battery_at_trip_start": self.current_battery_pct,

                "arrival_time": None,
                "time_to_station": None,
                "battery_at_arrival": None,
                "battery_spent_to_station": None,

                "charge_start_time": None,
                "charge_end_time": None,
                "charging_time": None,

                "battery_at_charge_start": None,
                "battery_end": None,
                "battery_gain": None,

                "finalized_reason": None,
            }

            self.recharge_trips.append(trip)
            self.current_trip = trip

            self.get_logger().info(f"Recharge trip #{trip_number} started at {self.current_battery_pct*100:.1f}%")

        elif (not msg.data) and self.last_recharge_active:
            if self.current_trip is not None and self.current_trip.get("arrival_time") is None:
                self.current_trip["arrival_time"] = now
                self.current_trip["battery_at_arrival"] = self.current_battery_pct

                trip_duration = (now - self.current_trip["trip_start_time"]).nanoseconds * 1e-9
                self.current_trip["time_to_station"] = trip_duration

                battery_spent = self.current_trip["battery_at_trip_start"] - self.current_battery_pct
                self.current_trip["battery_spent_to_station"] = battery_spent

                self.get_logger().info(
                    f"Arrived at station: {trip_duration:.1f}s, "
                    f"battery {self.current_trip['battery_at_trip_start']*100:.1f}% → {self.current_battery_pct*100:.1f}%"
                )

        self.last_recharge_active = bool(msg.data)

    def _battery_callback(self, msg: BatteryState):
        now = self.get_clock().now()

        self.prev_battery_pct = self.current_battery_pct
        if msg.percentage is not None:
            self.current_battery_pct = float(msg.percentage)

        delta = self.current_battery_pct - self.prev_battery_pct

        status = int(msg.power_supply_status) if msg.power_supply_status is not None else BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        status_known = status != BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        is_status_charging = status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        is_status_full = status == BatteryState.POWER_SUPPLY_STATUS_FULL

        if status_known:
            if (not self.currently_charging) and is_status_charging:
                self._mark_charging_start(now, why="status")

            if self.currently_charging and (is_status_full or (not is_status_charging)):
                self._mark_charging_end(now, why="status_full" if is_status_full else "status_stop")

            return

        if delta > self._trend_inc_thresh:
            self._trend_inc_count += 1
            self._trend_dec_count = 0
        elif abs(delta) <= self._trend_stop_thresh:
            self._trend_dec_count += 1
            self._trend_inc_count = 0
        else:
            self._trend_dec_count += 1
            self._trend_inc_count = 0

        if (not self.currently_charging) and (self._trend_inc_count >= self._trend_confirm_needed):
            self._mark_charging_start(now, why="trend")

        if self.currently_charging:
            if self.current_battery_pct >= 0.98:
                self._mark_charging_end(now, why="trend_full")
            elif self._trend_dec_count >= self._trend_stop_needed:
                self._mark_charging_end(now, why="trend_stop")

    # -------------------------------------------------------------------------
    # Charging event helpers
    # -------------------------------------------------------------------------
    def _mark_charging_start(self, now: Time, why: str):
        self.currently_charging = True
        self.charge_start_time = now
        self.battery_at_charge_start = self.current_battery_pct

        if self.current_trip is not None and self.current_trip.get("charge_start_time") is None:
            self.current_trip["charge_start_time"] = now
            self.current_trip["battery_at_charge_start"] = self.current_battery_pct

        self.get_logger().info(f"Charging started ({why}) at {self.current_battery_pct*100:.1f}%")

    def _mark_charging_end(self, now: Time, why: str):
        if not self.currently_charging or self.charge_start_time is None:
            return

        charge_duration = (now - self.charge_start_time).nanoseconds * 1e-9
        self.total_charging_time += max(0.0, charge_duration)

        battery_start = self.battery_at_charge_start if self.battery_at_charge_start is not None else self.current_battery_pct
        battery_gain = self.current_battery_pct - battery_start

        if self.current_trip is not None:
            if self.current_trip.get("charge_end_time") is None:
                self.current_trip["charge_end_time"] = now
            if self.current_trip.get("charging_time") is None:
                self.current_trip["charging_time"] = charge_duration
            if self.current_trip.get("battery_end") is None:
                self.current_trip["battery_end"] = self.current_battery_pct
            if self.current_trip.get("battery_gain") is None:
                self.current_trip["battery_gain"] = battery_gain

            self.current_trip["finalized_reason"] = f"charging_end:{why}"
            self.current_trip = None

        self.get_logger().info(
            f"Charging ended ({why}): {charge_duration:.1f}s, battery +{battery_gain*100:.1f}%"
        )

        self.currently_charging = False
        self.charge_start_time = None
        self.battery_at_charge_start = None

        self._trend_inc_count = 0
        self._trend_dec_count = 0

    # -------------------------------------------------------------------------
    # Finalization
    # -------------------------------------------------------------------------
    def _finalize_current_trip(self, reason: str = "shutdown"):
        if self.current_trip is None:
            return

        now = self.get_clock().now()

        if self.currently_charging and self.charge_start_time is not None:
            charge_duration = (now - self.charge_start_time).nanoseconds * 1e-9
            self.total_charging_time += max(0.0, charge_duration)

            battery_start = self.battery_at_charge_start if self.battery_at_charge_start is not None else self.current_battery_pct
            battery_gain = self.current_battery_pct - battery_start

            self.current_trip.setdefault("charge_start_time", self.charge_start_time)
            self.current_trip.setdefault("battery_at_charge_start", battery_start)
            self.current_trip["charge_end_time"] = now
            self.current_trip["charging_time"] = charge_duration
            self.current_trip["battery_end"] = self.current_battery_pct
            self.current_trip["battery_gain"] = battery_gain

        if self.current_trip.get("battery_end") is None:
            self.current_trip["battery_end"] = self.current_battery_pct

        if self.current_trip.get("charge_end_time") is None and self.current_trip.get("charge_start_time") is not None:
            self.current_trip["charge_end_time"] = now

        self.current_trip["finalized_reason"] = reason
        self.get_logger().info(f"Trip #{self.current_trip.get('trip_number')} finalized ({reason}).")
        self.current_trip = None

    # -------------------------------------------------------------------------
    # Save
    # -------------------------------------------------------------------------
    def save_data(self):
        try:
            self._finalize_current_trip(reason="shutdown")

            # one final pose sample right before writing, if pose exists
            if self.latest_pose is not None:
                t = _now_s(self.get_clock(), self.start_time)
                x, y, yaw = self.latest_pose
                if not self.path_samples or abs(self.path_samples[-1][0] - t) > 1e-6:
                    self.path_samples.append((t, x, y, yaw))

            end_time = self.get_clock().now()
            total_sim_time = (end_time - self.start_time).nanoseconds * 1e-9
            total_dwell_time = len(self.visited_cylinders) * self.dwell_time

            total_time_to_stations = sum(
                t["time_to_station"] for t in self.recharge_trips
                if t.get("time_to_station") is not None
            )

            total_battery_to_stations = sum(
                t["battery_spent_to_station"] for t in self.recharge_trips
                if t.get("battery_spent_to_station") is not None
            )

            recharge_trip_count = len(self.recharge_trips)

            with open(self.csv_file, "w", newline="") as f:
                writer = csv.writer(f, delimiter="\t")

                writer.writerow(["EXPERIMENT SUMMARY"])
                writer.writerow(["Max Linear Speed (m/s)", f"{self.max_linear:.3f}"])
                writer.writerow(["Max Angular Speed (rad/s)", f"{self.max_angular:.3f}"])
                writer.writerow(["Recharge Trips", recharge_trip_count])
                writer.writerow(["Cylinders Visited", f"{len(self.visited_cylinders)}/{len(self.cylinder_positions)}"])
                writer.writerow([])

                writer.writerow(["TIMING DATA"])
                writer.writerow(["Total Simulation Time (s)", f"{total_sim_time:.3f}"])
                writer.writerow(["Total Dwelling Time (s)", f"{total_dwell_time:.3f}"])
                writer.writerow(["Total Travel Time Between Cylinders (s)", f"{self.total_travel_time:.3f}"])
                writer.writerow(["Total Time To Reach Stations (s)", f"{total_time_to_stations:.3f}"])
                writer.writerow(["Total Charging Time (s)", f"{self.total_charging_time:.3f}"])
                writer.writerow(["Dwelling Time Per Cylinder (s)", f"{self.dwell_time:.3f}"])
                writer.writerow([])

                writer.writerow(["BATTERY DATA"])
                writer.writerow(["Total Battery Spent To Reach Stations (%)", f"{total_battery_to_stations*100:.3f}"])
                writer.writerow([])

                writer.writerow(["RECHARGE TRIPS"])
                writer.writerow([
                    "Trip #",
                    "Trip Start (s)",
                    "Arrive (s)",
                    "Time To Station (s)",
                    "Battery Start (%)",
                    "Battery At Arrival (%)",
                    "Battery Spent To Station (%)",
                    "Charge Start (s)",
                    "Charge End (s)",
                    "Charging Time (s)",
                    "Battery At Charge Start (%)",
                    "Battery End (%)",
                    "Battery Gain (%)",
                    "Finalized Reason",
                ])

                for trip in self.recharge_trips:
                    trip_start_s = (trip["trip_start_time"] - self.start_time).nanoseconds * 1e-9 if trip.get("trip_start_time") else None
                    arrival_s = (trip["arrival_time"] - self.start_time).nanoseconds * 1e-9 if trip.get("arrival_time") else None
                    charge_start_s = (trip["charge_start_time"] - self.start_time).nanoseconds * 1e-9 if trip.get("charge_start_time") else None
                    charge_end_s = (trip["charge_end_time"] - self.start_time).nanoseconds * 1e-9 if trip.get("charge_end_time") else None

                    def fmt_s(v: Optional[float]) -> str:
                        return f"{v:.3f}" if v is not None else ""

                    def fmt_pct(v: Optional[float]) -> str:
                        return f"{v*100:.3f}" if v is not None else ""

                    writer.writerow([
                        trip.get("trip_number", ""),
                        fmt_s(trip_start_s),
                        fmt_s(arrival_s),
                        fmt_s(trip.get("time_to_station")),
                        fmt_pct(trip.get("battery_at_trip_start")),
                        fmt_pct(trip.get("battery_at_arrival")),
                        fmt_pct(trip.get("battery_spent_to_station")),
                        fmt_s(charge_start_s),
                        fmt_s(charge_end_s),
                        fmt_s(trip.get("charging_time")),
                        fmt_pct(trip.get("battery_at_charge_start")),
                        fmt_pct(trip.get("battery_end")),
                        fmt_pct(trip.get("battery_gain")),
                        trip.get("finalized_reason", "") or "",
                    ])

                writer.writerow([])

                writer.writerow(["RECHARGE STATIONS"])
                writer.writerow(["Station #", "X", "Y"])
                for i, (x, y) in enumerate(self.charging_stations):
                    writer.writerow([i + 1, f"{x:.3f}", f"{y:.3f}"])
                writer.writerow([])

                writer.writerow(["ALL CYLINDERS"])
                writer.writerow(["Cylinder #", "X", "Y"])
                for i, (x, y) in enumerate(self.cylinder_positions):
                    writer.writerow([i + 1, f"{x:.3f}", f"{y:.3f}"])
                writer.writerow([])

                writer.writerow(["VISITED CYLINDERS"])
                writer.writerow(["Visit #", "X", "Y", "Arrival Time (s)", "Travel Time (s)"])

                for i, (x, y, arrival_time) in enumerate(self.visited_cylinders):
                    arrival_time_s = (arrival_time - self.start_time).nanoseconds * 1e-9

                    if i == 0:
                        travel_time = arrival_time_s
                    else:
                        prev_arrival = self.visited_cylinders[i - 1][2]
                        travel_time = (arrival_time - prev_arrival).nanoseconds * 1e-9 - self.dwell_time

                    writer.writerow([
                        i + 1,
                        f"{x:.3f}",
                        f"{y:.3f}",
                        f"{arrival_time_s:.3f}",
                        f"{travel_time:.3f}",
                    ])

                writer.writerow([])

                # -------------------- NEW: robot path samples --------------------
                writer.writerow(["ROBOT PATH"])
                writer.writerow(["Sample #", "Time (s)", "X", "Y", "Yaw (rad)"])

                for i, (t, x, y, yaw) in enumerate(self.path_samples):
                    writer.writerow([i + 1, f"{t:.3f}", f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])

            self.get_logger().info(f"✓ Data saved to {self.csv_file}")
            self.get_logger().info(
                f"Summary: {len(self.visited_cylinders)}/{len(self.cylinder_positions)} cylinders, "
                f"{recharge_trip_count} trips, {total_sim_time:.1f}s total, "
                f"{self.total_charging_time:.1f}s charging, "
                f"{len(self.path_samples)} path samples"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[Data Logger] Saving experiment data...")
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()