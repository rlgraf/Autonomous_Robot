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
import signal
import sys
from datetime import datetime
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

        # Supervisor decision threshold (used to decide when to start tracking
        # candidate targets).
        decision_battery_threshold: float = 0.25
        try:
            decider_yaml = os.path.join(pkg, "parameters", "decider_parameters.yaml")
            if os.path.exists(decider_yaml):
                with open(decider_yaml, "r") as f:
                    decider_config = yaml.safe_load(f)
                dec_params = decider_config.get("supervisor_node", {}).get("ros__parameters", {})
                decision_battery_threshold = float(
                    dec_params.get("decision_battery_threshold", decision_battery_threshold)
                )
        except Exception as e:
            # Fall back to a sane default if the YAML can't be read.
            print(f"[DATA LOGGER] Warning: could not load decider_parameters.yaml: {e}")

        # Handle both formats: charging_stations list or charging_station_x/y arrays
        if "charging_stations" in bat_params:
            self.charging_stations: List[Tuple[float, float]] = [tuple(s) for s in bat_params["charging_stations"]]
        else:
            # Fallback to x/y arrays
            xs = bat_params.get("charging_station_x", [-20.0, 20.0])
            ys = bat_params.get("charging_station_y", [0.0, 0.0])
            self.charging_stations = [(float(x), float(y)) for x, y in zip(xs, ys)]

        self.max_linear: float = float(rech_params.get("max_linear", 0.50))
        self.max_angular: float = float(rech_params.get("max_angular", 0.80))
        self.dwell_time: float = 5.0  # keep in sync with your navigator DWELL_TIME

        # Initialize run name (will be set when loading cylinder positions)
        self.run_name: Optional[str] = None

        # Load cylinder positions (cache) - this also sets self.run_name
        self.cylinder_positions = self._load_cylinder_positions()

        # ------------------------- Runtime data -------------------------
        self.visited_cylinders: List[Tuple[float, float, Time, float]] = []  # (x,y,arrival_time,battery_pct)
        self.last_visit_time: Optional[Time] = None
        self.total_travel_time: float = 0.0
        # Used for "below threshold?" labeling and for enabling candidate tracking.
        self.low_battery_threshold: float = decision_battery_threshold

        # Only start recording /recommended_target candidates once the battery
        # level hits the supervisor decision threshold.
        self._candidate_tracking_threshold: float = decision_battery_threshold
        self._candidate_tracking_enabled: bool = False
        # When True, we will only re-enable candidate tracking after the battery
        # crosses downward through the threshold (prev > thr and curr <= thr).
        self._candidate_tracking_waiting_for_threshold_cross: bool = True

        # Supervisor recommended candidate targets (from /recommended_target)
        # Stored as (x, y, time, battery_pct)
        self.candidate_targets: List[Tuple[float, float, Time, float]] = []
        self.last_candidate_xy: Optional[Tuple[float, float]] = None
        # Track unique candidate cylinders so "Total candidates" never exceeds the
        # number of cylinders in this run.
        self._seen_candidate_cylinders: set[int] = set()

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

        # Decision metrics from supervisor (reward/cost, etc.)
        self.decision_metrics_log: List[Dict[str, Any]] = []

        # Timing
        self.start_time: Time = self.get_clock().now()
        # Wall-clock timestamps for easier offline correlation/debugging.
        # (ROS time is monotonic and not human-readable.)
        self.run_start_wall_time_iso: str = datetime.now().isoformat(timespec="seconds")
        
        # Simulation time scaling (real-time factor)
        # Try to read from generated world file, default to 1.0
        self.real_time_factor: float = 1.0
        try:
            # Try to read from the generated SDF file
            import xml.etree.ElementTree as ET
            world_file = "/tmp/arena_generated.sdf"
            if os.path.exists(world_file):
                tree = ET.parse(world_file)
                root = tree.getroot()
                physics = root.find(".//physics")
                if physics is not None:
                    rtf_elem = physics.find("real_time_factor")
                    if rtf_elem is not None and rtf_elem.text:
                        self.real_time_factor = float(rtf_elem.text)
                        self.get_logger().info(f"Detected real-time factor: {self.real_time_factor}")
        except Exception as e:
            self.get_logger().warn(f"Could not read real-time factor from world file: {e}, using default 1.0")

        # ------------------------- Path tracking -------------------------
        self.latest_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.path_samples: List[Tuple[float, float, float, float]] = []  # (t, x, y, yaw)

        # ------------------------- ROS I/O -------------------------
        # Note: move5.py publishes to /visited_columns (not /visited_cylinders)
        self.create_subscription(Float32MultiArray, "/visited_columns", self._visited_callback, 10)
        self.get_logger().info("Subscribed to /visited_columns for cylinder visit tracking")

        # Subscribe to supervisor recommended targets (candidate cylinders)
        self.create_subscription(Float32MultiArray, "/recommended_target", self._candidate_callback, 10)
        self.get_logger().info("Subscribed to /recommended_target for candidate tracking")

        # Subscribe to supervisor decision metrics (includes reward + energy-cost)
        self.create_subscription(Float32MultiArray, "/decision_metrics", self._decision_metrics_callback, 10)
        self.get_logger().info("Subscribed to /decision_metrics for reward/cost tracking")

        self.create_subscription(Bool, "/recharge_active", self._recharge_callback, 10)
        # Supervisor publishes to `/battery_status` (absolute topic name), so match it exactly.
        self.create_subscription(BatteryState, "/battery_status", self._battery_callback, 10)

        # Choose whichever odometry topic your robot actually publishes
        # Replace "/gt_odom" with your real topic if needed.
        self.create_subscription(Odometry, "/odom_gt", self._odom_callback, 10)

        # Sample pose once per second
        self.create_timer(1.0, self._path_sample_timer_callback)

        # ------------------------- Output file -------------------------
        data_dir = os.path.expanduser("~/Autonomous_Robot/data")
        os.makedirs(data_dir, exist_ok=True)
        
        # Find the next run number by checking existing files
        # If we have a run name, include it in the filename for easier identification
        run_number = 1
        while True:
            if self.run_name:
                # Include run name in filename: e.g., "15cyl_run1_run1.csv"
                run_file = os.path.join(data_dir, f"{self.run_name}_run{run_number}.csv")
            else:
                run_file = os.path.join(data_dir, f"run{run_number}.csv")
            if not os.path.exists(run_file):
                break
            run_number += 1
        
        if self.run_name:
            self.csv_file = os.path.join(data_dir, f"{self.run_name}_run{run_number}.csv")
        else:
            self.csv_file = os.path.join(data_dir, f"run{run_number}.csv")  # tab-delimited TSV
        self.run_number = run_number

        num_columns = len(self.cylinder_positions)
        run_display_name = self.run_name if self.run_name else f"Run {run_number}"
        print(f"\n{'='*60}")
        print(f"Data Logger: {run_display_name}")
        print(f"Number of columns tested: {num_columns}")
        print(f"Output file: {self.csv_file}")
        print(f"{'='*60}\n")
        
        self.get_logger().info(
            f"Data logger ready. {run_display_name}. Tracking {num_columns} cylinders. "
            f"Logging to: {self.csv_file}"
        )
        
        # Register signal handlers for graceful shutdown
        self._shutdown_requested = False
        self._data_saved = False
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    # -------------------------------------------------------------------------
    # Data loading
    # -------------------------------------------------------------------------
    def _load_cylinder_positions(self) -> List[Tuple[float, float]]:
        """Load cylinder positions and optional run name from cache file."""
        data_dir = os.path.expanduser("~/Autonomous_Robot/data")
        cache_file = os.path.join(data_dir, "cylinder_positions.txt")
        cylinders: List[Tuple[float, float]] = []

        if not os.path.exists(cache_file):
            self.get_logger().warn(f"Cylinder cache not found: {cache_file}")
            return cylinders

        try:
            with open(cache_file, "r") as f:
                for line in f:
                    line = line.strip()
                    # Run name metadata
                    if line.startswith("#RUN_NAME:"):
                        self.run_name = line.replace("#RUN_NAME:", "").strip()
                        continue
                    # Skip empty lines and comments
                    if not line or line.startswith("#"):
                        continue
                    x, y = line.split(",")
                    cylinders.append((float(x.strip()), float(y.strip())))
            self.get_logger().info(f"Loaded {len(cylinders)} cylinders from cache")
            if self.run_name:
                self.get_logger().info(f"Detected run name: {self.run_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to load cylinders: {e}")

        return cylinders

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals (SIGINT, SIGTERM) to save data before exiting."""
        if not self._shutdown_requested:
            self._shutdown_requested = True
            self.get_logger().warn(f"Received signal {signum}, saving data before shutdown...")
            try:
                if not self._data_saved:
                    self.save_data()
                    self._data_saved = True
                    self.get_logger().info("Data saved successfully from signal handler")
            except Exception as e:
                self.get_logger().error(f"Error saving data on shutdown: {e}")
                # Try to save at least a partial file
                try:
                    self._emergency_save()
                except Exception as e2:
                    self.get_logger().error(f"Emergency save also failed: {e2}")
            # Don't call shutdown/exit here - let the main function's finally block handle it
            # Just set the flag so main() knows to save

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
        self.get_logger().debug(f"_visited_callback called with {len(msg.data)} data elements")
        
        if len(msg.data) < 2:
            self.get_logger().warn(f"Received visited_columns message with insufficient data: {len(msg.data)} elements (need at least 2)")
            return

        try:
            x, y = float(msg.data[0]), float(msg.data[1])
            now = self.get_clock().now()

            if self.last_visit_time is not None:
                dt = (now - self.last_visit_time).nanoseconds * 1e-9
                dt -= self.dwell_time
                self.total_travel_time += max(0.0, dt)

            # Record battery level at visit time
            battery_pct = self.current_battery_pct
            self.visited_cylinders.append((x, y, now, battery_pct))
            self.last_visit_time = now

            low_battery_flag = " [LOW BATTERY <25%]" if battery_pct < self.low_battery_threshold else ""
            self.get_logger().info(
                f"[DATA LOGGER] Cylinder visited at ({x:.2f}, {y:+.2f}) - "
                f"Battery: {battery_pct*100:.1f}%{low_battery_flag} - "
                f"Total: {len(self.visited_cylinders)}/{len(self.cylinder_positions)}"
            )
            print(f"[DATA LOGGER] Cylinder visited at ({x:.2f}, {y:+.2f}) - Battery: {battery_pct*100:.1f}% - Total: {len(self.visited_cylinders)}/{len(self.cylinder_positions)}")
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing visited_columns message: {e}, data: {msg.data}")

    def _match_candidate_to_cylinder(self, wx: float, wy: float, distance_threshold: float = 0.25) -> Optional[int]:
        """Map a recommended (wx, wy) to the nearest cylinder index.

        Returns None if no cylinder is close enough.
        """
        if not self.cylinder_positions:
            return None

        best_idx: Optional[int] = None
        best_d2: float = float("inf")
        for i, (cx, cy) in enumerate(self.cylinder_positions):
            dx = wx - cx
            dy = wy - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i

        if best_idx is None:
            return None

        if best_d2 <= distance_threshold * distance_threshold:
            return best_idx
        return None

    def _candidate_callback(self, msg: Float32MultiArray):
        """Track supervisor recommended candidate targets from /recommended_target."""
        # Empty data means "no candidate" - do not record, just note at debug level
        if len(msg.data) < 2:
            self.get_logger().debug("Received empty /recommended_target (no candidate)")
            return

        # Ignore candidates until battery reaches the supervisor decision threshold.
        if not self._candidate_tracking_enabled:
            return

        try:
            wx = float(msg.data[0])
            wy = float(msg.data[1])
            now = self.get_clock().now()

            cylinder_idx = self._match_candidate_to_cylinder(wx, wy)
            if cylinder_idx is None:
                # Still record the candidate target even if we can't map it to a
                # known cylinder position. This helps debugging coordinate mismatches.
                self.get_logger().debug(
                    f"Candidate at ({wx:.2f}, {wy:+.2f}) had no nearby cylinder match (recording anyway)"
                )
            elif cylinder_idx in self._seen_candidate_cylinders:
                # Already recorded this cylinder as a candidate once during this run.
                return

            # Avoid flooding the log/data with tiny jitter around the same target
            if self.last_candidate_xy is not None:
                last_x, last_y = self.last_candidate_xy
                if math.hypot(wx - last_x, wy - last_y) < 0.1:
                    return

            self.last_candidate_xy = (wx, wy)
            battery_pct = self.current_battery_pct
            self.candidate_targets.append((wx, wy, now, battery_pct))
            if cylinder_idx is not None:
                self._seen_candidate_cylinders.add(cylinder_idx)

            self.get_logger().info(
                f"[DATA LOGGER] Candidate target at ({wx:.2f}, {wy:+.2f}) - "
                f"Battery: {battery_pct*100:.1f}% - "
                f"Total candidates: {len(self.candidate_targets)}"
            )
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing recommended_target message: {e}, data: {msg.data}")

    def _decision_metrics_callback(self, msg: Float32MultiArray):
        """Log supervisor decision metrics so reward/cost can be tracked."""
        data = msg.data
        # Expected order (see supervisor_node.py):
        # [0] reward(score), [1] extra_distance, [2] margin, [3] obj_distance,
        # [4] obj_to_charger_distance, [5] predicted_drain_per_meter,
        # [6] battery_pct, [7] decision_mode_code, [8] energy_needed(cost)
        if len(data) < 8:
            self.get_logger().debug(f"Received /decision_metrics with insufficient length: {len(data)}")
            return

        def _get(i: int) -> float:
            try:
                return float(data[i])
            except Exception:
                return float("nan")

        score = _get(0)
        extra_distance = _get(1)
        margin = _get(2)
        obj_distance = _get(3)
        obj_to_charger_distance = _get(4)
        predicted_drain_per_meter = _get(5)
        battery_pct = _get(6)
        decision_mode_code = _get(7)
        energy_needed = _get(8) if len(data) >= 9 else float("nan")

        # Treat "reward" as the supervisor's total score.
        reward = score
        # Treat "cost" as the battery energy fraction needed to make the visit feasible.
        cost = energy_needed

        if not (math.isfinite(reward) or math.isfinite(cost)):
            return

        now = self.get_clock().now()
        self.decision_metrics_log.append(
            {
                "time": now,
                "reward": reward,
                "cost": cost,
                "score": score,
                "extra_distance": extra_distance,
                "margin": margin,
                "obj_distance": obj_distance,
                "obj_to_charger_distance": obj_to_charger_distance,
                "predicted_drain_per_meter": predicted_drain_per_meter,
                "battery_pct": battery_pct,
                "decision_mode_code": decision_mode_code,
            }
        )

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
            # Stop candidate tracking while we are heading to/charging at the station.
            # We'll re-enable only after battery crosses downward through the
            # supervisor decision threshold again.
            self._candidate_tracking_enabled = False
            self._candidate_tracking_waiting_for_threshold_cross = True

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
                # Reset candidate tracking after reaching the charging station.
                # Then we'll re-track the next low-battery decision phase.
                self._candidate_tracking_enabled = False
                self._candidate_tracking_waiting_for_threshold_cross = True
                self.last_candidate_xy = None

        self.last_recharge_active = bool(msg.data)

    def _battery_callback(self, msg: BatteryState):
        now = self.get_clock().now()

        self.prev_battery_pct = self.current_battery_pct
        if msg.percentage is not None:
            self.current_battery_pct = float(msg.percentage)

        # Enable candidate tracking only when we cross downward through the
        # supervisor threshold (prev > thr and curr <= thr). This prevents
        # enabling immediately after a recharge reset if we're already low.
        if (not self._candidate_tracking_enabled) and self._candidate_tracking_waiting_for_threshold_cross:
            if (self.prev_battery_pct > self._candidate_tracking_threshold) and (self.current_battery_pct <= self._candidate_tracking_threshold):
                self._candidate_tracking_enabled = True
                self._candidate_tracking_waiting_for_threshold_cross = False
                self.get_logger().info(
                    f"[DATA LOGGER] Candidate tracking enabled at "
                    f"{self.current_battery_pct*100:.1f}% (threshold {self._candidate_tracking_threshold*100:.1f}%)"
                )

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

        # Fallback: if status isn't available, detect charging via battery trend.
        if delta > self._trend_inc_thresh:
            # Battery rising => likely charging.
            self._trend_inc_count += 1
            self._trend_dec_count = 0
        elif delta < -self._trend_stop_thresh or abs(delta) <= self._trend_stop_thresh:
            # Battery stable or falling => likely not charging / charging ending.
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
            run_end_wall_time_iso: str = datetime.now().isoformat(timespec="seconds")
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
                writer.writerow(["Run Start Wall Time", self.run_start_wall_time_iso])
                writer.writerow(["Run End Wall Time", run_end_wall_time_iso])
                if self.run_name:
                    writer.writerow(["Run Name", self.run_name])
                writer.writerow(["Max Linear Speed (m/s)", f"{self.max_linear:.3f}"])
                writer.writerow(["Max Angular Speed (rad/s)", f"{self.max_angular:.3f}"])
                writer.writerow(["Real-Time Factor", f"{self.real_time_factor:.3f}"])
                writer.writerow(["Recharge Trips", recharge_trip_count])
                writer.writerow(["Total Cylinders", len(self.cylinder_positions)])
                writer.writerow(["Cylinders Visited", len(self.visited_cylinders)])
                writer.writerow(["Cylinders Visited Ratio", f"{len(self.visited_cylinders)}/{len(self.cylinder_positions)}"])
                
                # Count cylinders visited below 25% battery
                low_battery_visits = [v for v in self.visited_cylinders if v[3] < self.low_battery_threshold]
                writer.writerow(["Cylinders Visited Below 25% Battery", len(low_battery_visits)])
                writer.writerow([])

                writer.writerow(["TIMING DATA"])
                writer.writerow(["Total Simulation Time (s)", f"{total_sim_time:.3f}"])
                if self.real_time_factor > 0:
                    wall_clock_time = total_sim_time / self.real_time_factor
                    writer.writerow(["Estimated Wall Clock Time (s)", f"{wall_clock_time:.3f}"])
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
                writer.writerow(["Visit #", "X", "Y", "Arrival Time (s)", "Travel Time (s)", "Battery Level (%)", "Below 25%?"])
                
                if len(self.visited_cylinders) == 0:
                    writer.writerow(["No cylinders visited during this run"])

                for i, (x, y, arrival_time, battery_pct) in enumerate(self.visited_cylinders):
                    arrival_time_s = (arrival_time - self.start_time).nanoseconds * 1e-9

                    if i == 0:
                        travel_time = arrival_time_s
                    else:
                        prev_arrival = self.visited_cylinders[i - 1][2]
                        travel_time = (arrival_time - prev_arrival).nanoseconds * 1e-9 - self.dwell_time

                    below_threshold = "Yes" if battery_pct < self.low_battery_threshold else "No"

                    writer.writerow([
                        i + 1,
                        f"{x:.3f}",
                        f"{y:.3f}",
                        f"{arrival_time_s:.3f}",
                        f"{travel_time:.3f}",
                        f"{battery_pct*100:.2f}",
                        below_threshold,
                    ])
                
                writer.writerow([])
                
                # Summary of cylinders visited below 25%
                low_battery_visits = [v for v in self.visited_cylinders if v[3] < self.low_battery_threshold]
                if len(low_battery_visits) > 0:
                    writer.writerow(["CYLINDERS VISITED BELOW 25% BATTERY"])
                    writer.writerow(["Visit #", "X", "Y", "Arrival Time (s)", "Battery Level (%)"])
                    for visit in low_battery_visits:
                        x, y, arrival_time, battery_pct = visit
                        arrival_time_s = (arrival_time - self.start_time).nanoseconds * 1e-9
                        visit_num = self.visited_cylinders.index(visit) + 1
                        writer.writerow([
                            visit_num,
                            f"{x:.3f}",
                            f"{y:.3f}",
                            f"{arrival_time_s:.3f}",
                            f"{battery_pct*100:.2f}",
                        ])
                    writer.writerow([])

                writer.writerow([])

                # -------------------- Candidate targets from supervisor --------------------
                writer.writerow(["CANDIDATE TARGETS"])
                writer.writerow(["Candidate #", "X", "Y", "Time (s)", "Battery Level (%)"])

                if len(self.candidate_targets) == 0:
                    writer.writerow(["No candidate targets recorded"])
                else:
                    for i, (wx, wy, t_candidate, batt_pct) in enumerate(self.candidate_targets):
                        t_s = (t_candidate - self.start_time).nanoseconds * 1e-9
                        writer.writerow([
                            i + 1,
                            f"{wx:.3f}",
                            f"{wy:.3f}",
                            f"{t_s:.3f}",
                            f"{batt_pct*100:.2f}",
                        ])

                writer.writerow([])

                # -------------------- Decision metrics (reward/cost) --------------------
                writer.writerow(["DECISION METRICS"])
                writer.writerow(
                    [
                        "Decision #",
                        "Time (s)",
                        "Decision Mode Code",
                        "Reward (score)",
                        "Cost (energy_needed)",
                        "Extra Distance",
                        "Margin",
                        "Obj Distance",
                        "Obj->Charger Distance",
                        "Battery Level (%)",
                    ]
                )

                if len(self.decision_metrics_log) == 0:
                    writer.writerow(["No decision metrics recorded"])
                else:
                    # Summary (only low-battery decision modes where energy cost is expected)
                    low_batt_metrics = [
                        d
                        for d in self.decision_metrics_log
                        if d.get("decision_mode_code") in (1.0, 2.0) and math.isfinite(float(d.get("cost", float("nan"))))
                    ]
                    if low_batt_metrics:
                        avg_reward = sum(float(d["reward"]) for d in low_batt_metrics) / len(low_batt_metrics)
                        avg_cost = sum(float(d["cost"]) for d in low_batt_metrics) / len(low_batt_metrics)
                        writer.writerow(
                            [
                                "LOW_BATT SUMMARY (ALLOW_ONE_VISIT + FORCE_RECHARGE)",
                                "",
                                "",
                                f"{avg_reward:.6f}",
                                f"{avg_cost:.6f}",
                                "",
                                "",
                                "",
                                "",
                                "",
                            ]
                        )
                        writer.writerow([])

                    for i, d in enumerate(self.decision_metrics_log):
                        t_s = (d["time"] - self.start_time).nanoseconds * 1e-9
                        writer.writerow(
                            [
                                i + 1,
                                f"{t_s:.3f}",
                                f"{float(d.get('decision_mode_code', float('nan'))):.1f}",
                                f"{float(d.get('reward', float('nan'))):.6f}",
                                f"{float(d.get('cost', float('nan'))):.6f}" if math.isfinite(float(d.get("cost", float("nan")))) else "nan",
                                f"{float(d.get('extra_distance', float('nan'))):.6f}" if math.isfinite(float(d.get("extra_distance", float("nan")))) else "nan",
                                f"{float(d.get('margin', float('nan'))):.6f}" if math.isfinite(float(d.get("margin", float("nan")))) else "nan",
                                f"{float(d.get('obj_distance', float('nan'))):.6f}" if math.isfinite(float(d.get("obj_distance", float("nan")))) else "nan",
                                f"{float(d.get('obj_to_charger_distance', float('nan'))):.6f}" if math.isfinite(float(d.get("obj_to_charger_distance", float("nan")))) else "nan",
                                f"{float(d.get('battery_pct', float('nan')) * 100.0):.3f}" if math.isfinite(float(d.get("battery_pct", float("nan")))) else "nan",
                            ]
                        )

                writer.writerow([])

                # -------------------- NEW: robot path samples --------------------
                writer.writerow(["ROBOT PATH"])
                writer.writerow(["Sample #", "Time (s)", "X", "Y", "Yaw (rad)"])

                for i, (t, x, y, yaw) in enumerate(self.path_samples):
                    writer.writerow([i + 1, f"{t:.3f}", f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])

            num_columns_tested = len(self.cylinder_positions)
            num_columns_visited = len(self.visited_cylinders)
            run_display_name = self.run_name if self.run_name else f"Run {self.run_number}"
            
            print(f"\n{'='*60}")
            print(f"{run_display_name} Complete - Data saved to {self.csv_file}")
            print(f"Number of columns tested: {num_columns_tested}")
            print(f"Number of columns visited: {num_columns_visited}")
            print(f"Summary: {num_columns_visited}/{num_columns_tested} cylinders, "
                  f"{recharge_trip_count} trips, {total_sim_time:.1f}s total, "
                  f"{self.total_charging_time:.1f}s charging, "
                  f"{len(self.path_samples)} path samples")
            print(f"{'='*60}\n")
            
            self.get_logger().info(f"✓ Data saved to {self.csv_file}")
            self.get_logger().info(
                f"{run_display_name}: {num_columns_visited}/{num_columns_tested} cylinders, "
                f"{recharge_trip_count} trips, {total_sim_time:.1f}s total, "
                f"{self.total_charging_time:.1f}s charging, "
                f"{len(self.path_samples)} path samples"
            )
            self._data_saved = True

        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")
            # Try emergency save as last resort
            try:
                self._emergency_save()
            except Exception as e2:
                self.get_logger().error(f"Emergency save also failed: {e2}")

    def _emergency_save(self):
        """Emergency save - write minimal data if full save fails."""
        try:
            num_columns_tested = len(self.cylinder_positions)
            num_columns_visited = len(self.visited_cylinders)
            run_end_wall_time_iso: str = datetime.now().isoformat(timespec="seconds")
            
            with open(self.csv_file, "w", newline="") as f:
                writer = csv.writer(f, delimiter="\t")
                writer.writerow(["# EMERGENCY SAVE - Partial Data"])
                writer.writerow(["# Process was terminated unexpectedly"])
                writer.writerow(["Run Start Wall Time", self.run_start_wall_time_iso])
                writer.writerow(["Run End Wall Time", run_end_wall_time_iso])
                writer.writerow(["Run Number", self.run_number])
                writer.writerow(["Number of Columns Tested", num_columns_tested])
                writer.writerow(["Cylinders Visited", num_columns_visited])
                writer.writerow(["Recharge Trips", len(self.recharge_trips)])
                if self.visited_cylinders:
                    writer.writerow(["VISITED CYLINDERS"])
                    writer.writerow(["Visit #", "X", "Y", "Battery Level (%)"])
                    for i, (x, y, _, battery_pct) in enumerate(self.visited_cylinders):
                        writer.writerow([i + 1, f"{x:.3f}", f"{y:.3f}", f"{battery_pct*100:.2f}"])
            
            print(f"\n{'='*60}")
            print(f"Run {self.run_number} - EMERGENCY SAVE")
            print(f"Number of columns tested: {num_columns_tested}")
            print(f"Number of columns visited: {num_columns_visited}")
            print(f"Emergency partial data saved to {self.csv_file}")
            print(f"{'='*60}\n")
            
            self.get_logger().warn(f"Emergency partial data saved to {self.csv_file}")
        except Exception as e:
            self.get_logger().error(f"Emergency save failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("\n[Data Logger] KeyboardInterrupt received, saving experiment data...")
        node._shutdown_requested = True
    except Exception as e:
        node.get_logger().error(f"[Data Logger] Exception occurred: {e}, saving data...")
        node._shutdown_requested = True
    finally:
        # Always try to save data on shutdown - this is the last chance.
        if not hasattr(node, "_data_saved") or not node._data_saved:
            try:
                node.get_logger().info("Saving data in finally block...")
                node.save_data()
                node._data_saved = True
                node.get_logger().info("Data saved successfully in finally block")
            except Exception as e:
                node.get_logger().error(f"Failed to save data in finally block: {e}")
                # Last resort - try emergency save
                try:
                    node._emergency_save()
                except Exception as e2:
                    node.get_logger().error(f"Emergency save in finally block also failed: {e2}")
        else:
            node.get_logger().info("Data already saved, skipping save in finally block")

        # Best-effort cleanup.
        try:
            node.destroy_node()
        except Exception as e:
            node.get_logger().error(f"Error during node.destroy_node(): {e}")

        try:
            rclpy.shutdown()
        except Exception as e:
            node.get_logger().error(f"Error during rclpy.shutdown(): {e}")


if __name__ == "__main__":
    main()
