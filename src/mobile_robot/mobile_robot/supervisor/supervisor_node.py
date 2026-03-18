#!/usr/bin/env python3
import math
from collections import deque
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String


def zero_twist() -> Twist:
    return Twist()


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CmdSource:
    """Holds last Twist and timestamp for a candidate source."""
    def __init__(self):
        self.msg = zero_twist()
        self.stamp = None


class SupervisorArbiter(Node):
    """
    Supervisor Arbiter (MUX) with latched low-battery decision logic.

    Priority:
      1) depart reverse
      2) recharge mode:
            /cmd_vel_soft_avoid  (if paused during recharge and not charging)
            /cmd_vel_recharge
      3) normal mode:
            /cmd_vel_avoid
            /cmd_vel_nav

    Low-battery logic:
      - When battery first drops to/below decision_battery_threshold:
            evaluate whether one more person/object visit is feasible.
      - If feasible -> latch ALLOW_ONE_VISIT.
      - After that visit completes (detected via /depart_request), latch FORCE_RECHARGE.
      - No per-tick oscillation.
    """

    LOW_BATT_NORMAL = "NORMAL"
    LOW_BATT_ALLOW_ONE_VISIT = "ALLOW_ONE_VISIT"
    LOW_BATT_FORCE_RECHARGE = "FORCE_RECHARGE"

    MODE_CODE_UNKNOWN = -1.0
    MODE_CODE_NORMAL = 0.0
    MODE_CODE_ALLOW_ONE_VISIT = 1.0
    MODE_CODE_FORCE_RECHARGE = 2.0

    def __init__(self):
        super().__init__("supervisor_node")

        # -----------------------------
        # Parameters: arbitration
        # -----------------------------
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.35)

        # -----------------------------
        # Parameters: battery decision
        # -----------------------------
        self.declare_parameter("decision_battery_threshold", 0.25)
        self.declare_parameter("reserve_battery_fraction", 0.15)
        # When we are homing/holding for recharge, we need to stay in forced mode
        # even if `/recharge_active` goes false upon arrival (auto_recharge_node does
        # that). Use battery fraction to decide when it's safe to resume normal ops.
        self.declare_parameter("recharge_full_battery_threshold", 0.98)

        self.declare_parameter("default_drain_per_meter", 0.02)
        self.declare_parameter("drain_alpha", 0.10)
        self.declare_parameter("min_motion_for_drain_update", 0.1)

        self.declare_parameter("visit_reward", 1.0)
        self.declare_parameter("extra_distance_weight", 0.70)
        self.declare_parameter("margin_weight", 2.5)

        self.declare_parameter("corridor_half_angle_deg", 35.0)
        self.declare_parameter("target_capture_radius", 1.2)
        self.declare_parameter("lidar_clearance_margin", 0.25)
        self.declare_parameter("scan_window_deg", 15.0)
        self.declare_parameter("max_object_considered_m", 10.0)
        # How many sequential cylinder visits we allow after entering the
        # low-battery "visit before recharge" mode.
        # Previously this was effectively limited by a "near station" check
        # at depart completion time; increasing this lets the rewards system
        # continue selecting new cylinders until the next visit is infeasible.
        self.declare_parameter("max_visits_before_recharge", 3)

        # flat list: [x1, y1, x2, y2, ...]
        self.declare_parameter("charging_stations", [0.0, 0.0])

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)

        self.decision_battery_threshold = float(self.get_parameter("decision_battery_threshold").value)
        self.reserve_battery_fraction = float(self.get_parameter("reserve_battery_fraction").value)
        self.recharge_full_battery_threshold = float(
            self.get_parameter("recharge_full_battery_threshold").value
        )

        self.default_drain_per_meter = float(self.get_parameter("default_drain_per_meter").value)
        self.drain_alpha = float(self.get_parameter("drain_alpha").value)
        self.min_motion_for_drain_update = float(self.get_parameter("min_motion_for_drain_update").value)

        self.visit_reward = float(self.get_parameter("visit_reward").value)
        self.extra_distance_weight = float(self.get_parameter("extra_distance_weight").value)
        self.margin_weight = float(self.get_parameter("margin_weight").value)

        self.corridor_half_angle = math.radians(float(self.get_parameter("corridor_half_angle_deg").value))
        self.target_capture_radius = float(self.get_parameter("target_capture_radius").value)
        self.lidar_clearance_margin = float(self.get_parameter("lidar_clearance_margin").value)
        self.scan_window = math.radians(float(self.get_parameter("scan_window_deg").value))
        self.max_object_considered_m = float(self.get_parameter("max_object_considered_m").value)

        flat_stations = list(self.get_parameter("charging_stations").value)
        self.charging_stations = self._parse_station_list(flat_stations)

        # -----------------------------
        # State / latches
        # -----------------------------
        self.recharge_active = False
        self.battery_charging = False
        self.navigation_paused = False
        self.avoid_active = False

        self.depart_active = False
        self.depart_end_time = None

        self.low_batt_mode = self.LOW_BATT_NORMAL
        self.low_batt_decision_made = False
        self.allowed_visit_consumed = False
        self.pending_force_recharge_after_depart = False
        # In FORCE_RECHARGE we want to avoid exiting "too early" before the
        # battery node actually starts charging (it may take a moment to
        # become stationary inside the charging radius).
        self._force_recharge_seen_charging: bool = False
        self.best_candidate = None
        self.low_batt_visit_count = 0  # Track completed low-battery visits
        self.max_visits_before_recharge = int(
            self.get_parameter("max_visits_before_recharge").value
        )

        self.last_logged_mode = ""

        # Candidate command buffers
        self.nav = CmdSource()
        self.avoid = CmdSource()
        self.recharge = CmdSource()
        self.soft = CmdSource()

        # -----------------------------
        # Robot state
        # -----------------------------
        self.have_odom = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.have_battery = False
        self.battery_fraction = 1.0
        self.prev_battery_fraction = None

        self.have_scan = False
        self.scan_msg = None

        # detected objects in robot frame: [(xr, yr), ...]
        self.detected_objects_robot = []
        
        # Track visited objects (world coordinates) to avoid recommending them
        self.visited_objects_world: List[Tuple[float, float]] = []
        self.visited_radius = 1.2  # Same as target_capture_radius - objects within this distance are considered visited

        # online drain estimate
        self.drain_per_meter = self.default_drain_per_meter
        self.prev_batt_for_update = None
        self.prev_pos_for_update = None
        self.energy_history = deque(maxlen=50)
        
        # Track movement direction for sequential visit preference
        self.movement_direction = None  # (dx, dy) normalized direction vector
        self.prev_pos_for_direction = None
        self.direction_history = deque(maxlen=10)  # Recent movement directions

        # -----------------------------
        # Publishers
        # -----------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.supervisor_active_pub = self.create_publisher(Bool, "/supervisor_active", 10)
        self.decision_metrics_pub = self.create_publisher(Float32MultiArray, "/decision_metrics", 10)
        self.event_pub = self.create_publisher(String, "/supervisor_events", 10)
        self.recommended_target_pub = self.create_publisher(Float32MultiArray, "/recommended_target", 10)

        # -----------------------------
        # Subscriptions (candidates)
        # -----------------------------
        self.create_subscription(Twist, "/cmd_vel_nav", self._cb_nav, 10)
        self.create_subscription(Twist, "/cmd_vel_avoid", self._cb_avoid, 10)
        self.create_subscription(Twist, "/cmd_vel_recharge", self._cb_recharge, 10)
        self.create_subscription(Twist, "/cmd_vel_soft_avoid", self._cb_soft, 10)

        # -----------------------------
        # Subscriptions (gates)
        # -----------------------------
        self.create_subscription(Bool, "/recharge_active", self._cb_recharge_active, 10)
        self.create_subscription(Bool, "/battery_charging", self._cb_battery_charging, 10)
        self.create_subscription(Bool, "/navigation_paused", self._cb_navigation_paused, 10)
        self.create_subscription(Bool, "/avoid_active", self._cb_avoid_active, 10)
        self.create_subscription(Bool, "/depart_request", self._cb_depart_request, 10)
        
        # Subscribe to visited cylinders to track which objects have been visited
        self.create_subscription(Float32MultiArray, "/visited_columns", self._cb_visited_columns, 10)

        # -----------------------------
        # Subscriptions (decision inputs)
        # -----------------------------
        self.create_subscription(BatteryState, "/battery_status", self._cb_battery_status, 10)
        self.create_subscription(Odometry, "/odom_gt", self._cb_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._cb_scan, 10)
        self.create_subscription(Float32MultiArray, "/detected_objects", self._cb_detected_objects, 10)

        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            "SupervisorArbiter running with latched low-battery decision logic."
        )
        self._publish_event("SUPERVISOR_START")

    # -----------------------------
    # Event publisher
    # -----------------------------
    def _publish_event(self, text: str):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    # -----------------------------
    # Decision metrics publisher
    # -----------------------------
    def _publish_decision_metrics(self, best_candidate, decision_mode_code: float):
        """
        Publishes decision metrics in this fixed order:

        [0] reward (total score)
        [1] extra_distance
        [2] margin
        [3] obj_distance
        [4] obj_to_charger_distance
        [5] predicted_drain_per_meter
        [6] battery_pct
        [7] decision_mode_code
        [8] energy_needed (cost in battery fraction)

        decision_mode_code:
            0.0 -> NORMAL / no forced low-battery decision active
            1.0 -> ALLOW_ONE_VISIT
            2.0 -> FORCE_RECHARGE
           -1.0 -> UNKNOWN
        """
        msg = Float32MultiArray()

        if best_candidate is None:
            msg.data = [
                float("nan"),                 # score
                float("nan"),                 # extra_distance
                float("nan"),                 # margin
                float("nan"),                 # obj_distance
                float("nan"),                 # obj_to_charger_distance
                float(self.drain_per_meter),  # predicted_drain_per_meter
                float(self.battery_fraction), # battery_pct
                float(decision_mode_code),    # decision_mode_code
                float("nan"),                 # energy_needed (cost)
            ]
        else:
            msg.data = [
                float(best_candidate.get("score", float("nan"))),
                float(best_candidate.get("extra_distance", float("nan"))),
                float(best_candidate.get("margin", float("nan"))),
                float(best_candidate.get("obj_distance", float("nan"))),
                float(best_candidate.get("obj_to_charger_distance", float("nan"))),
                float(self.drain_per_meter),
                float(self.battery_fraction),
                float(decision_mode_code),
                float(best_candidate.get("energy_needed", float("nan"))),
            ]

        self.decision_metrics_pub.publish(msg)

    # -----------------------------
    # Parsing helpers
    # -----------------------------
    def _parse_station_list(self, vals):
        vals = [float(v) for v in vals]
        stations = []
        for i in range(0, len(vals) - 1, 2):
            stations.append((vals[i], vals[i + 1]))
        return stations if stations else [(0.0, 0.0)]

    # -----------------------------
    # Callbacks: candidates
    # -----------------------------
    def _cb_nav(self, msg: Twist):
        self.nav.msg = msg
        self.nav.stamp = self.get_clock().now()

    def _cb_avoid(self, msg: Twist):
        self.avoid.msg = msg
        self.avoid.stamp = self.get_clock().now()

    def _cb_recharge(self, msg: Twist):
        self.recharge.msg = msg
        self.recharge.stamp = self.get_clock().now()

    def _cb_soft(self, msg: Twist):
        self.soft.msg = msg
        self.soft.stamp = self.get_clock().now()

    # -----------------------------
    # Callbacks: gates
    # -----------------------------
    def _cb_recharge_active(self, msg: Bool):
        old = self.recharge_active
        self.recharge_active = bool(msg.data)
        if self.recharge_active != old:
            self._publish_event(f"RECHARGE_ACTIVE_{'ON' if self.recharge_active else 'OFF'}")

    def _cb_battery_charging(self, msg: Bool):
        old = self.battery_charging
        self.battery_charging = bool(msg.data)
        if self.battery_charging != old:
            self._publish_event(f"BATTERY_CHARGING_{'START' if self.battery_charging else 'STOP'}")

    def _cb_navigation_paused(self, msg: Bool):
        old = self.navigation_paused
        self.navigation_paused = bool(msg.data)
        if self.navigation_paused != old:
            self._publish_event(f"NAVIGATION_PAUSED_{'ON' if self.navigation_paused else 'OFF'}")

    def _cb_avoid_active(self, msg: Bool):
        old = self.avoid_active
        self.avoid_active = bool(msg.data)
        if self.avoid_active != old:
            self._publish_event(f"AVOID_ACTIVE_{'ON' if self.avoid_active else 'OFF'}")

    def _cb_depart_request(self, msg: Bool):
        if not bool(msg.data):
            return

        self._publish_event("DEPART_REQUEST_RECEIVED")

        if (
            self.low_batt_mode == self.LOW_BATT_ALLOW_ONE_VISIT
            and not self.allowed_visit_consumed
        ):
            # Allow multiple sequential visits before we force recharge.
            # Each next visit is still chosen via `_evaluate_visit_before_recharge()`,
            # which checks battery feasibility to reach a charger after that visit.
            self.low_batt_visit_count += 1
            self.get_logger().info(
                f"Low-battery visit {self.low_batt_visit_count}/{self.max_visits_before_recharge} completed. "
                f"{'Allowing another visit.' if self.low_batt_visit_count < self.max_visits_before_recharge else 'Max visits reached, will force recharge.'}"
            )

            if self.low_batt_visit_count >= self.max_visits_before_recharge:
                self.allowed_visit_consumed = True
                self.pending_force_recharge_after_depart = True
                self._publish_event("ALLOWED_VISIT_CONSUMED_MAX_REACHED")
            else:
                # Reset so the supervisor can pick a fresh next cylinder.
                self.low_batt_decision_made = False
                self.best_candidate = None
                self._publish_event(f"ALLOWED_VISIT_BEFORE_RECHARGE_{self.low_batt_visit_count}")

        if not self.depart_active:
            self.depart_active = True
            self.depart_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=1.0)
            self.get_logger().info("Depart request received: executing 1.0s reverse.")
            self._publish_event("DEPART_START")

    # -----------------------------
    # Callbacks: state
    # -----------------------------
    def _cb_battery_status(self, msg: BatteryState):
        if math.isfinite(msg.percentage):
            prev = self.battery_fraction if self.have_battery else None
            self.have_battery = True
            self.battery_fraction = clamp(float(msg.percentage), 0.0, 1.0)
            self.prev_battery_fraction = prev

    def _cb_odom(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.robot_x = px
        self.robot_y = py
        self.robot_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_odom = True

        if not self.have_battery:
            return

        curr_pos = (px, py)
        curr_batt = self.battery_fraction

        if self.prev_pos_for_update is None or self.prev_batt_for_update is None:
            self.prev_pos_for_update = curr_pos
            self.prev_batt_for_update = curr_batt
            return

        dx = curr_pos[0] - self.prev_pos_for_update[0]
        dy = curr_pos[1] - self.prev_pos_for_update[1]
        dist = math.hypot(dx, dy)
        
        # Update movement direction for sequential visit preference
        if dist > 0.1:  # Only update if moved significantly
            if self.prev_pos_for_direction is not None:
                dir_dx = curr_pos[0] - self.prev_pos_for_direction[0]
                dir_dy = curr_pos[1] - self.prev_pos_for_direction[1]
                dir_dist = math.hypot(dir_dx, dir_dy)
                if dir_dist > 0.1:
                    # Normalize direction vector
                    self.movement_direction = (dir_dx / dir_dist, dir_dy / dir_dist)
                    self.direction_history.append(self.movement_direction)
                    # Average recent directions for smoother direction estimate
                    if len(self.direction_history) > 1:
                        avg_dx = sum(d[0] for d in self.direction_history) / len(self.direction_history)
                        avg_dy = sum(d[1] for d in self.direction_history) / len(self.direction_history)
                        avg_norm = math.hypot(avg_dx, avg_dy)
                        if avg_norm > 0.01:
                            self.movement_direction = (avg_dx / avg_norm, avg_dy / avg_norm)
            self.prev_pos_for_direction = curr_pos

        if dist >= self.min_motion_for_drain_update:
            dbatt = self.prev_batt_for_update - curr_batt
            if dbatt > 0.0:
                sample = dbatt / dist
                self.drain_per_meter = (
                    (1.0 - self.drain_alpha) * self.drain_per_meter
                    + self.drain_alpha * sample
                )
                self.energy_history.append((dist, dbatt, self.drain_per_meter))

            self.prev_pos_for_update = curr_pos
            self.prev_batt_for_update = curr_batt

    def _cb_scan(self, msg: LaserScan):
        self.have_scan = True
        self.scan_msg = msg

    def _cb_detected_objects(self, msg: Float32MultiArray):
        """
        Parse detected objects from identify5.py.
        Format: [world_x, world_y, r_dist, theta, ...] (4 values per object)
        We need robot frame coordinates, so we use r_dist and theta directly,
        or convert world_x, world_y to robot frame if odom is available.
        """
        if not self.have_odom:
            # Can't convert coordinates without odom, skip
            return
            
        data = list(msg.data)
        objs = []
        # Parse 4 values per object: [world_x, world_y, r_dist, theta]
        for i in range(0, len(data) - 3, 4):
            if i + 3 >= len(data):
                break
            world_x = float(data[i])
            world_y = float(data[i + 1])
            r_dist = float(data[i + 2])
            theta = float(data[i + 3])
            
            # Validate all values are finite
            if not all(math.isfinite(v) for v in [world_x, world_y, r_dist, theta]):
                continue
                
            # Use r_dist and theta directly (already in robot frame)
            # Convert polar to cartesian in robot frame
            xr = r_dist * math.cos(theta)
            yr = r_dist * math.sin(theta)
            
            objs.append((xr, yr))
        self.detected_objects_robot = objs

    def _cb_visited_columns(self, msg: Float32MultiArray):
        """Track visited cylinder positions to avoid recommending them."""
        if len(msg.data) < 2:
            return
        
        try:
            wx = float(msg.data[0])
            wy = float(msg.data[1])
            
            # Check if this position is already in visited list (within visited_radius)
            is_duplicate = False
            for vx, vy in self.visited_objects_world:
                if math.hypot(wx - vx, wy - vy) < self.visited_radius:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                self.visited_objects_world.append((wx, wy))
                self.get_logger().debug(f"Tracked visited object at ({wx:.2f}, {wy:.2f})")
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Error parsing visited_columns message: {e}")

    def _is_object_visited(self, obj_wx: float, obj_wy: float) -> bool:
        """Check if an object at world coordinates has been visited."""
        for vx, vy in self.visited_objects_world:
            if math.hypot(obj_wx - vx, obj_wy - vy) < self.visited_radius:
                return True
        return False

    # -----------------------------
    # Helpers
    # -----------------------------
    def _fresh(self, src: CmdSource, now) -> bool:
        if src.stamp is None:
            return False
        age = (now - src.stamp).nanoseconds * 1e-9
        return age <= self.cmd_timeout

    def _nearest_station(self):
        best = None
        best_d = float("inf")
        for sx, sy in self.charging_stations:
            d = math.hypot(sx - self.robot_x, sy - self.robot_y)
            if d < best_d:
                best_d = d
                best = (sx, sy)
        return best, best_d

    def _world_to_robot_bearing(self, wx, wy):
        dx = wx - self.robot_x
        dy = wy - self.robot_y
        return wrap_to_pi(math.atan2(dy, dx) - self.robot_yaw)

    def _robot_to_world(self, xr, yr):
        c = math.cos(self.robot_yaw)
        s = math.sin(self.robot_yaw)
        wx = self.robot_x + c * xr - s * yr
        wy = self.robot_y + s * xr + c * yr
        return wx, wy

    def _range_at_relative_angle(self, rel_angle):
        if not self.have_scan or self.scan_msg is None:
            return float("inf")

        scan = self.scan_msg
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return float("inf")

        a_min = rel_angle - self.scan_window
        a_max = rel_angle + self.scan_window

        i0 = int((a_min - scan.angle_min) / scan.angle_increment)
        i1 = int((a_max - scan.angle_min) / scan.angle_increment)

        i0 = max(0, i0)
        i1 = min(n - 1, i1)

        best = float("inf")
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and r > 0.0:
                best = min(best, r)
        return best

    def _direction_clear(self, rel_angle, required_distance):
        if required_distance <= 0.0:
            return True
        measured = self._range_at_relative_angle(rel_angle)
        needed = max(0.0, required_distance - self.lidar_clearance_margin)
        return measured >= needed

    def _evaluate_visit_before_recharge(self):
        """
        Returns:
            allow_visit (bool)
            best_candidate (dict or None)
        """
        if not (self.have_battery and self.have_odom):
            self._publish_event("EVAL_SKIP_NO_BATTERY_OR_ODOM")
            return False, None

        station, dist_to_charger = self._nearest_station()
        if station is None:
            self._publish_event("EVAL_SKIP_NO_CHARGING_STATION")
            return False, None

        sx, sy = station
        charger_bearing = self._world_to_robot_bearing(sx, sy)

        # When very close to station (< 10m), be more lenient with path clearance check
        # Use larger clearance distance to account for being near the station
        clearance_dist = min(dist_to_charger, 2.0) if dist_to_charger > 10.0 else max(dist_to_charger, 1.0)
        if not self._direction_clear(charger_bearing, clearance_dist):
            self._publish_event("EVAL_REJECT_CHARGER_PATH_BLOCKED")
            return False, None

        remaining_usable = self.battery_fraction - self.reserve_battery_fraction
        if remaining_usable <= 0.0:
            self._publish_event("EVAL_REJECT_NO_USABLE_BATTERY")
            return False, None

        direct_energy = dist_to_charger * self.drain_per_meter
        if direct_energy > remaining_usable:
            self._publish_event("EVAL_REJECT_DIRECT_CHARGE_NOT_FEASIBLE")
            return False, None

        candidates = []

        for xr, yr in self.detected_objects_robot:
            d_obj = math.hypot(xr, yr)
            if d_obj <= 0.05 or d_obj > self.max_object_considered_m:
                continue

            obj_bearing = math.atan2(yr, xr)

            # When close to station (< 10m), widen the corridor angle to allow more flexibility
            # This allows visits to objects that are slightly off the direct path when near station
            effective_corridor_angle = self.corridor_half_angle
            if dist_to_charger < 10.0:
                # Widen corridor by up to 20 degrees when very close (scales with distance)
                distance_factor = (10.0 - dist_to_charger) / 10.0  # 0.0 at 10m, 1.0 at 0m
                angle_bonus = math.radians(20.0) * distance_factor
                effective_corridor_angle = self.corridor_half_angle + angle_bonus

            if abs(wrap_to_pi(obj_bearing - charger_bearing)) > effective_corridor_angle:
                continue

            if not self._direction_clear(obj_bearing, max(0.0, d_obj - self.target_capture_radius)):
                continue

            obj_wx, obj_wy = self._robot_to_world(xr, yr)
            
            # Skip if this object has already been visited
            if self._is_object_visited(obj_wx, obj_wy):
                continue
            
            d_obj_to_charger = math.hypot(sx - obj_wx, sy - obj_wy)

            total_path = d_obj + d_obj_to_charger
            extra_dist = total_path - dist_to_charger
            energy_needed = total_path * self.drain_per_meter
            margin = remaining_usable - energy_needed

            if margin < 0.0:
                continue

            # When close to station, reduce penalty for extra distance since return trip is short
            # This encourages visits when near the station
            effective_extra_dist_weight = self.extra_distance_weight
            if dist_to_charger < 10.0:
                # Reduce penalty by up to 50% when very close to station
                distance_factor = (10.0 - dist_to_charger) / 10.0  # 0.0 at 10m, 1.0 at 0m
                effective_extra_dist_weight = self.extra_distance_weight * (1.0 - 0.5 * distance_factor)

            # Bonus for objects in the direction of travel (sequential visit preference)
            direction_bonus = 0.0
            if self.movement_direction is not None:
                # Calculate direction from robot to object
                obj_dir_x = xr / d_obj if d_obj > 0.01 else 0.0
                obj_dir_y = yr / d_obj if d_obj > 0.01 else 0.0
                # Dot product: positive if object is in forward direction
                dot_product = (obj_dir_x * self.movement_direction[0] + 
                              obj_dir_y * self.movement_direction[1])
                # Bonus up to 0.3 for objects directly ahead (dot_product = 1.0)
                # No bonus for objects behind (dot_product < 0)
                if dot_product > 0:
                    direction_bonus = 0.3 * dot_product
            
            score = (
                self.visit_reward
                - effective_extra_dist_weight * extra_dist
                + self.margin_weight * margin
                + direction_bonus  # Prefer objects in direction of travel
            )

            candidates.append({
                "xr": xr,
                "yr": yr,
                "wx": obj_wx,  # Store world coordinates for publishing to navigation
                "wy": obj_wy,
                "obj_distance": d_obj,
                "obj_to_charger_distance": d_obj_to_charger,
                "total_path": total_path,
                "extra_distance": extra_dist,
                "energy_needed": energy_needed,
                "margin": margin,
                "score": score,
                "station": station,
            })

        if not candidates:
            self._publish_event("EVAL_REJECT_NO_FEASIBLE_OBJECT")
            return False, None

        best = max(candidates, key=lambda c: c["score"])
        if best["score"] <= 0.0:
            self._publish_event("EVAL_REJECT_NONPOSITIVE_SCORE")
            return False, None

        self._publish_event(
            f"EVAL_ACCEPT_BEST score={best['score']:.3f} "
            f"obj_d={best['obj_distance']:.2f} extra_d={best['extra_distance']:.2f} "
            f"margin={best['margin']:.3f}"
        )
        return True, best

    def _evaluate_best_target_normal_operation(self):
        """
        Evaluates and returns the best target for normal operation (not low battery).
        Returns best candidate dict or None.
        """
        if not (self.have_battery and self.have_odom and self.have_scan):
            return None

        if not self.detected_objects_robot:
            return None

        candidates = []

        for xr, yr in self.detected_objects_robot:
            d_obj = math.hypot(xr, yr)
            if d_obj <= 0.05 or d_obj > self.max_object_considered_m:
                continue

            obj_bearing = math.atan2(yr, xr)

            # Check if path to object is clear
            if not self._direction_clear(obj_bearing, max(0.0, d_obj - self.target_capture_radius)):
                continue

            obj_wx, obj_wy = self._robot_to_world(xr, yr)
            
            # Skip if this object has already been visited
            if self._is_object_visited(obj_wx, obj_wy):
                continue

            # Score based on distance (closer is better), path clearance, and direction
            clearance_bonus = 1.0 if self._direction_clear(obj_bearing, d_obj) else 0.5
            base_score = clearance_bonus / (d_obj + 0.1)  # Inverse distance with small offset
            
            # Bonus for objects in the direction of travel (sequential visit preference)
            direction_bonus = 0.0
            if self.movement_direction is not None:
                obj_dir_x = xr / d_obj if d_obj > 0.01 else 0.0
                obj_dir_y = yr / d_obj if d_obj > 0.01 else 0.0
                dot_product = (obj_dir_x * self.movement_direction[0] + 
                              obj_dir_y * self.movement_direction[1])
                if dot_product > 0:
                    direction_bonus = 0.5 * dot_product  # Larger bonus for normal operation
            
            score = base_score + direction_bonus

            candidates.append({
                "xr": xr,
                "yr": yr,
                "wx": obj_wx,
                "wy": obj_wy,
                "obj_distance": d_obj,
                "score": score,
            })

        if not candidates:
            return None

        best = max(candidates, key=lambda c: c["score"])
        return best

    def _log_mode_once(self, text: str):
        if text != self.last_logged_mode:
            self.last_logged_mode = text
            self.get_logger().info(text)
            self._publish_event(text)

    def _publish_recommended_target(self, candidate):
        """Publishes recommended target to navigation node."""
        if candidate is None:
            msg = Float32MultiArray()
            msg.data = []  # Empty means no recommendation
            self.recommended_target_pub.publish(msg)
            return

        # Validate candidate has required fields
        if "wx" not in candidate or "wy" not in candidate:
            self.get_logger().warn("Candidate missing wx/wy fields, skipping publish")
            return
            
        wx = float(candidate["wx"])
        wy = float(candidate["wy"])
        
        # Validate coordinates are finite
        if not (math.isfinite(wx) and math.isfinite(wy)):
            self.get_logger().warn(f"Invalid candidate coordinates: ({wx}, {wy})")
            return

        msg = Float32MultiArray()
        msg.data = [wx, wy]
        self.recommended_target_pub.publish(msg)

    def _update_low_battery_state(self):
        """
        Latched logic:
          - NORMAL -> evaluate once when battery crosses threshold
          - ALLOW_ONE_VISIT -> hold and repeatedly select new cylinders until
            we reach `max_visits_before_recharge` or no feasible visit exists
          - FORCE_RECHARGE -> hold until recharge_active clears
        """
        if not self.have_battery:
            return

        if self.low_batt_mode == self.LOW_BATT_FORCE_RECHARGE:
            # Important: `auto_recharge_node` publishes `/recharge_active` only while
            # *homing*. On arrival it flips `/recharge_active` back to False while
            # the battery node locks the robot in charging until full.
            #
            # If we cleared FORCE_RECHARGE immediately on `/recharge_active==False`,
            # the supervisor can resume normal navigation before charging completes,
            # which looks like "not fully committing to the recharge station".
            #
            # So we only release forced mode once the battery is effectively full.
            if self.battery_charging:
                self._force_recharge_seen_charging = True

            if self._force_recharge_seen_charging and self.battery_fraction >= self.recharge_full_battery_threshold:
                self.low_batt_mode = self.LOW_BATT_NORMAL
                self.low_batt_decision_made = False
                self.allowed_visit_consumed = False
                self.pending_force_recharge_after_depart = False
                self.best_candidate = None
                self.low_batt_visit_count = 0  # Reset visit counter after recharge
                self._force_recharge_seen_charging = False

                self._publish_decision_metrics(None, self.MODE_CODE_NORMAL)
                self._log_mode_once(
                    f"LOW_BATTERY_RESET_AFTER_RECHARGE batt={self.battery_fraction:.3f}"
                )
            return

        if self.pending_force_recharge_after_depart and not self.depart_active:
            self.pending_force_recharge_after_depart = False
            self.low_batt_mode = self.LOW_BATT_FORCE_RECHARGE
            self._force_recharge_seen_charging = False
            self.best_candidate = None

            self._publish_decision_metrics(None, self.MODE_CODE_FORCE_RECHARGE)
            self._log_mode_once(
                f"LOW_BATTERY_FORCE_RECHARGE batt={self.battery_fraction:.3f} "
                f"drain_per_m={self.drain_per_meter:.4f}"
            )
            return

        if self.low_batt_mode == self.LOW_BATT_ALLOW_ONE_VISIT:
            # If we have a candidate, use it
            if self.best_candidate is not None:
                self._publish_decision_metrics(self.best_candidate, self.MODE_CODE_ALLOW_ONE_VISIT)
                self._log_mode_once(
                    f"LOW_BATTERY_ALLOW_ONE_VISIT "
                    f"score={self.best_candidate['score']:.3f} "
                    f"obj_d={self.best_candidate['obj_distance']:.2f}m "
                    f"extra_d={self.best_candidate['extra_distance']:.2f}m "
                    f"margin={self.best_candidate['margin']:.3f}"
                )
                return
            
            # If no candidate and we still haven't consumed all visits, re-evaluate.
            if not self.allowed_visit_consumed and self.low_batt_visit_count < self.max_visits_before_recharge:
                allow_visit, best = self._evaluate_visit_before_recharge()

                if allow_visit and best is not None:
                    self.best_candidate = best
                    self._publish_decision_metrics(best, self.MODE_CODE_ALLOW_ONE_VISIT)
                    self._log_mode_once(
                        f"LOW_BATTERY_ALLOW_ONE_VISIT (re-evaluated) "
                        f"score={best['score']:.3f} "
                        f"obj_d={best['obj_distance']:.2f}m "
                        f"extra_d={best['extra_distance']:.2f}m "
                        f"margin={best['margin']:.3f}"
                    )
                    return
                else:
                    # No feasible visit found (battery constraints), force recharge
                    self.get_logger().info(
                        "No feasible low-battery visit found. Forcing recharge."
                    )
                    self.allowed_visit_consumed = True
                    self.pending_force_recharge_after_depart = True
                    self.low_batt_visit_count = 0
                    self.low_batt_mode = self.LOW_BATT_FORCE_RECHARGE
                    self._force_recharge_seen_charging = False
                    self._publish_decision_metrics(None, self.MODE_CODE_FORCE_RECHARGE)
                    return
            
            # No candidate and can't re-evaluate - should force recharge
            if self.best_candidate is None:
                self.get_logger().warn(
                    "LOW_BATT_ALLOW_ONE_VISIT mode but best_candidate is None and can't re-evaluate"
                )
            return

        if self.low_batt_mode == self.LOW_BATT_NORMAL:
            crossed = (
                self.battery_fraction <= self.decision_battery_threshold
                and (
                    self.prev_battery_fraction is None
                    or self.prev_battery_fraction > self.decision_battery_threshold
                    or not self.low_batt_decision_made
                )
            )

            if not crossed:
                return

            self._publish_event(
                f"LOW_BATTERY_THRESHOLD_CROSSED batt={self.battery_fraction:.3f}"
            )

            allow_visit, best = self._evaluate_visit_before_recharge()
            self.low_batt_decision_made = True

            if allow_visit:
                self.low_batt_mode = self.LOW_BATT_ALLOW_ONE_VISIT
                self.best_candidate = best
                self.allowed_visit_consumed = False
                self.pending_force_recharge_after_depart = False

                self._publish_decision_metrics(best, self.MODE_CODE_ALLOW_ONE_VISIT)
                self._log_mode_once(
                    f"LOW_BATTERY_ALLOW_ONE_VISIT "
                    f"score={best['score']:.3f} "
                    f"obj_d={best['obj_distance']:.2f}m "
                    f"extra_d={best['extra_distance']:.2f}m "
                    f"margin={best['margin']:.3f}"
                )
            else:
                self.low_batt_mode = self.LOW_BATT_FORCE_RECHARGE
                self._force_recharge_seen_charging = False
                self.best_candidate = None
                self.allowed_visit_consumed = False
                self.pending_force_recharge_after_depart = False

                self._publish_decision_metrics(None, self.MODE_CODE_FORCE_RECHARGE)
                self._log_mode_once(
                    f"LOW_BATTERY_FORCE_RECHARGE batt={self.battery_fraction:.3f} "
                    f"drain_per_m={self.drain_per_meter:.4f}"
                )

    # -----------------------------
    # Arbitration tick
    # -----------------------------
    def _tick(self):
        now = self.get_clock().now()

        self._update_low_battery_state()

        # Publish recommended targets based on current mode
        if self.depart_active or self.recharge_active:
            # Clear recommendation during recharge or depart
            self._publish_recommended_target(None)
        elif self.low_batt_mode == self.LOW_BATT_ALLOW_ONE_VISIT:
            # During ALLOW_ONE_VISIT mode, publish the battery-safe target
            # This is critical: navigation must use the battery-safe target, not pick its own
            if self.best_candidate is not None:
                # Validate that the target has world coordinates
                target_wx = self.best_candidate.get("wx")
                target_wy = self.best_candidate.get("wy")
                if target_wx is not None and target_wy is not None and \
                   math.isfinite(target_wx) and math.isfinite(target_wy):
                    # Publish the battery-safe target - this is the key fix
                    # We publish even if validation fails, because the battery calculation
                    # is more important than perfect object detection matching
                    self._publish_recommended_target(self.best_candidate)
                    
                    # Optional: Log if target doesn't match current detections (for debugging)
                    if self.detected_objects_robot:
                        target_still_valid = False
                        for xr, yr in self.detected_objects_robot:
                            obj_wx, obj_wy = self._robot_to_world(xr, yr)
                            dist = math.hypot(target_wx - obj_wx, target_wy - obj_wy)
                            if dist < 1.5:  # Increased tolerance to 1.5m
                                target_still_valid = True
                                break
                        
                        if not target_still_valid:
                            self.get_logger().debug(
                                f"Battery-safe target ({target_wx:.2f}, {target_wy:.2f}) "
                                "not in current detections, but publishing anyway for battery safety"
                            )
                else:
                    # Missing coordinates - this shouldn't happen, but handle gracefully
                    self.get_logger().warn(
                        "best_candidate missing wx/wy coordinates, cannot publish recommendation"
                    )
                    self._publish_recommended_target(None)
            else:
                # No best candidate - this shouldn't happen in ALLOW_ONE_VISIT mode
                self.get_logger().warn(
                    "LOW_BATT_ALLOW_ONE_VISIT mode but best_candidate is None"
                )
                self._publish_recommended_target(None)
        elif self.low_batt_mode == self.LOW_BATT_NORMAL:
            # During normal operation, evaluate and recommend best target
            best_normal = self._evaluate_best_target_normal_operation()
            self._publish_recommended_target(best_normal)
        else:
            # FORCE_RECHARGE or other modes - clear recommendation
            self._publish_recommended_target(None)

        if self.depart_active:
            if self.depart_end_time is not None and now >= self.depart_end_time:
                self.depart_active = False
                self.depart_end_time = None
                self.cmd_pub.publish(zero_twist())
                self.supervisor_active_pub.publish(Bool(data=False))
                self.get_logger().info("Depart complete: releasing control.")
                self._publish_event("DEPART_COMPLETE")
                return

            cmd = Twist()
            cmd.linear.x = -0.25
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.supervisor_active_pub.publish(Bool(data=True))
            return

        recharge_cmd_ready = self._fresh(self.recharge, now)
        forced_recharge = (self.low_batt_mode == self.LOW_BATT_FORCE_RECHARGE)
        # If we're in forced recharge, keep recharge mode "active" even if the
        # auto_recharge candidate cmd is temporarily stale. In ARRIVED/holding
        # transitions, auto_recharge_node may flip /recharge_active false while
        # battery_node is still locking movement; we must not yield control.
        effective_recharge_mode = self.recharge_active or forced_recharge

        self.supervisor_active_pub.publish(Bool(data=effective_recharge_mode))

        if effective_recharge_mode:
            soft_allowed = (not self.battery_charging)

            if soft_allowed and self.navigation_paused and self._fresh(self.soft, now):
                self.cmd_pub.publish(self.soft.msg)
                return

            if recharge_cmd_ready:
                self.cmd_pub.publish(self.recharge.msg)
                return

            # No fresh recharge command; hold position.
            self.cmd_pub.publish(zero_twist())
            return

        if forced_recharge and not recharge_cmd_ready:
            self.cmd_pub.publish(zero_twist())
            return

        if self.avoid_active and self._fresh(self.avoid, now):
            self.cmd_pub.publish(self.avoid.msg)
            return

        if self._fresh(self.nav, now):
            self.cmd_pub.publish(self.nav.msg)
            return

        self.cmd_pub.publish(zero_twist())


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()