#!/usr/bin/env python3

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32MultiArray, String


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DataLogger(Node):
    def __init__(self):
        super().__init__("data_logger")

        # ------------------------------------------------
        # Create experiment folder
        # ------------------------------------------------
        workspace = os.path.expanduser("~/Autonomous_Robot")
        data_root = os.path.join(workspace, "data")
        os.makedirs(data_root, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(data_root, f"run_{timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)

        self.path_file = os.path.join(self.run_dir, "trajectory.tsv")
        self.battery_file = os.path.join(self.run_dir, "battery.tsv")
        self.decision_file = os.path.join(self.run_dir, "decisions.tsv")
        self.events_file = os.path.join(self.run_dir, "events.tsv")
        self.cycles_file = os.path.join(self.run_dir, "charge_cycles.tsv")

        self.get_logger().info(f"Logging experiment to {self.run_dir}")

        # ------------------------------------------------
        # Runtime storage
        # ------------------------------------------------
        self.start_time = self.get_clock().now()

        self.path_log = []
        self.battery_log = []
        self.decision_log = []
        self.event_log = []
        self.cycle_log = []

        self.latest_pose = None
        self.current_charging = False
        self.last_battery_pct = None

        # ------------------------------------------------
        # Recharge station configuration
        # Edit these if your charger locations are different
        # ------------------------------------------------
        self.recharge_stations = [
            (-20.0, 0.0),
            (20.0, 0.0),
        ]

        # ------------------------------------------------
        # Low battery / charge-cycle state
        # ------------------------------------------------
        self.low_battery_threshold = 0.25
        self.low_battery_active = False

        self.current_cycle_id = 0
        self.cycle_start_time = 0.0

        self.guests_this_cycle = 0
        self.columns_this_cycle = 0

        self.guests_after_25 = 0
        self.columns_after_25 = 0

        self.time_battery_hit_25 = None
        self.distance_to_recharge_at_25 = None

        self.distance_traveled_this_cycle = 0.0
        self.distance_traveled_after_25 = 0.0
        self.last_pose_for_distance = None

        self.battery_pct_at_cycle_start = None
        self.battery_pct_at_25 = None
        self.battery_pct_at_charge_start = None

        # ------------------------------------------------
        # Expected event name patterns
        # Adjust these to match your supervisor publisher exactly
        # ------------------------------------------------
        self.guest_visit_events = {
            "guest_visited",
            "interaction_complete",
            "interaction_completed",
            "guest_interaction",
            "visited_guest",
            "visited_person",
            "person_visited",
        }

        self.column_visit_events = {
            "column_visited",
            "visited_column",
        }

        # ------------------------------------------------
        # ROS subscriptions
        # ------------------------------------------------
        self.create_subscription(
            Odometry,
            "/odom_gt",
            self.odom_callback,
            10
        )

        self.create_subscription(
            BatteryState,
            "/battery_status",
            self.battery_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/battery_charging",
            self.charging_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            "/decision_metrics",
            self.decision_callback,
            10
        )

        self.create_subscription(
            String,
            "/supervisor_events",
            self.supervisor_event_callback,
            10
        )

        # sample pose every second
        self.create_timer(1.0, self.sample_pose)

    # ------------------------------------------------
    # Helpers
    # ------------------------------------------------
    def get_time(self):
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds * 1e-9

    def distance_to_nearest_recharge(self, x, y):
        if not self.recharge_stations:
            return None
        return min(math.hypot(x - cx, y - cy) for cx, cy in self.recharge_stations)

    def normalize_event_name(self, event: str) -> str:
        return event.strip().lower()

    def event_matches(self, normalized_event: str, candidates: set) -> bool:
        if normalized_event in candidates:
            return True
        for candidate in candidates:
            if candidate in normalized_event:
                return True
        return False

    def append_event(self, t, event):
        self.event_log.append([t, event])

    def finalize_cycle(self, t):
        cycle_duration = t - self.cycle_start_time if self.cycle_start_time is not None else None

        battery_used_until_charge = None
        if (
            self.battery_pct_at_cycle_start is not None
            and self.battery_pct_at_charge_start is not None
        ):
            battery_used_until_charge = (
                self.battery_pct_at_cycle_start - self.battery_pct_at_charge_start
            )

        time_from_25_to_charge = None
        if self.time_battery_hit_25 is not None:
            time_from_25_to_charge = t - self.time_battery_hit_25

        self.cycle_log.append([
            self.current_cycle_id,
            self.cycle_start_time,
            t,
            cycle_duration,
            self.battery_pct_at_cycle_start,
            self.battery_pct_at_25,
            self.battery_pct_at_charge_start,
            battery_used_until_charge,
            self.guests_this_cycle,
            self.columns_this_cycle,
            self.guests_after_25,
            self.columns_after_25,
            self.time_battery_hit_25,
            self.distance_to_recharge_at_25,
            self.distance_traveled_this_cycle,
            self.distance_traveled_after_25,
            time_from_25_to_charge,
        ])

        self.append_event(
            t,
            (
                f"cycle_summary "
                f"cycle_id={self.current_cycle_id} "
                f"guests_this_cycle={self.guests_this_cycle} "
                f"columns_this_cycle={self.columns_this_cycle} "
                f"guests_after_25={self.guests_after_25} "
                f"columns_after_25={self.columns_after_25} "
                f"distance_to_recharge_at_25={self.distance_to_recharge_at_25} "
                f"distance_traveled_this_cycle={self.distance_traveled_this_cycle:.3f} "
                f"distance_traveled_after_25={self.distance_traveled_after_25:.3f}"
            )
        )

    def reset_for_new_cycle(self, t):
        self.current_cycle_id += 1
        self.cycle_start_time = t

        self.guests_this_cycle = 0
        self.columns_this_cycle = 0

        self.guests_after_25 = 0
        self.columns_after_25 = 0

        self.low_battery_active = False
        self.time_battery_hit_25 = None
        self.distance_to_recharge_at_25 = None

        self.distance_traveled_this_cycle = 0.0
        self.distance_traveled_after_25 = 0.0
        self.last_pose_for_distance = None

        self.battery_pct_at_cycle_start = self.last_battery_pct
        self.battery_pct_at_25 = None
        self.battery_pct_at_charge_start = None

    # ------------------------------------------------
    # Supervisor events
    # ------------------------------------------------
    def supervisor_event_callback(self, msg: String):
        t = self.get_time()
        raw_event = msg.data
        event = self.normalize_event_name(raw_event)

        self.append_event(t, raw_event)

        if self.event_matches(event, self.guest_visit_events):
            self.guests_this_cycle += 1
            self.append_event(
                t,
                f"guest_count cycle_id={self.current_cycle_id} guests_this_cycle={self.guests_this_cycle}"
            )

            if self.low_battery_active:
                self.guests_after_25 += 1
                self.append_event(
                    t,
                    f"guest_after_25 cycle_id={self.current_cycle_id} guests_after_25={self.guests_after_25}"
                )

        if self.event_matches(event, self.column_visit_events):
            self.columns_this_cycle += 1
            self.append_event(
                t,
                f"column_count cycle_id={self.current_cycle_id} columns_this_cycle={self.columns_this_cycle}"
            )

            if self.low_battery_active:
                self.columns_after_25 += 1
                self.append_event(
                    t,
                    f"column_after_25 cycle_id={self.current_cycle_id} columns_after_25={self.columns_after_25}"
                )

    # ------------------------------------------------
    # Odometry
    # ------------------------------------------------
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        x = float(p.x)
        y = float(p.y)

        # accumulate path length continuously
        if self.last_pose_for_distance is not None:
            prev_x, prev_y = self.last_pose_for_distance
            ds = math.hypot(x - prev_x, y - prev_y)
            self.distance_traveled_this_cycle += ds
            if self.low_battery_active:
                self.distance_traveled_after_25 += ds

        self.last_pose_for_distance = (x, y)

        self.latest_pose = (
            x,
            y,
            float(yaw),
            float(v),
            float(w)
        )

    # ------------------------------------------------
    # Battery
    # ------------------------------------------------
    def battery_callback(self, msg: BatteryState):
        t = self.get_time()

        pct = float(msg.percentage)
        charge = float(msg.charge)
        voltage = float(msg.voltage)
        current = float(msg.current)

        self.last_battery_pct = pct

        if self.battery_pct_at_cycle_start is None:
            self.battery_pct_at_cycle_start = pct

        self.battery_log.append([
            t, pct, charge, voltage, current
        ])

        # detect first crossing below low-battery threshold in current cycle
        if (not self.low_battery_active) and (pct <= self.low_battery_threshold):
            self.low_battery_active = True
            self.time_battery_hit_25 = t
            self.battery_pct_at_25 = pct

            if self.latest_pose is not None:
                x, y, _, _, _ = self.latest_pose
                self.distance_to_recharge_at_25 = self.distance_to_nearest_recharge(x, y)
            else:
                self.distance_to_recharge_at_25 = None

            self.append_event(
                t,
                (
                    f"battery_hit_25 "
                    f"cycle_id={self.current_cycle_id} "
                    f"battery_pct={pct:.4f} "
                    f"distance_to_recharge={self.distance_to_recharge_at_25}"
                )
            )

    # ------------------------------------------------
    # Charging state
    # ------------------------------------------------
    def charging_callback(self, msg: Bool):
        t = self.get_time()

        if msg.data != self.current_charging:
            self.current_charging = msg.data

            if msg.data:
                # charging started -> current cycle ends
                self.battery_pct_at_charge_start = self.last_battery_pct
                self.append_event(t, f"charging_start cycle_id={self.current_cycle_id}")
                self.finalize_cycle(t)
            else:
                # charging stopped -> new cycle begins
                self.append_event(t, f"charging_stop cycle_id={self.current_cycle_id}")
                self.reset_for_new_cycle(t)

    # ------------------------------------------------
    # Decision metrics
    # ------------------------------------------------
    def decision_callback(self, msg: Float32MultiArray):
        t = self.get_time()
        row = [t] + list(msg.data)
        self.decision_log.append(row)

    # ------------------------------------------------
    # Periodic pose sampling
    # ------------------------------------------------
    def sample_pose(self):
        if self.latest_pose is None:
            return

        t = self.get_time()
        x, y, yaw, v, w = self.latest_pose

        self.path_log.append([
            t, x, y, yaw, v, w
        ])

    # ------------------------------------------------
    # Save experiment data
    # ------------------------------------------------
    def save_data(self):
        try:
            # trajectory
            with open(self.path_file, "w", newline="") as f:
                writer = csv.writer(f, delimiter="\t")
                writer.writerow([
                    "time_s",
                    "x",
                    "y",
                    "yaw",
                    "linear_vel",
                    "angular_vel"
                ])
                for row in self.path_log:
                    writer.writerow(row)

            # battery
            with open(self.battery_file, "w", newline="") as f:
                writer = csv.writer(f, delimiter="\t")
                writer.writerow([
                    "time_s",
                    "battery_pct",
                    "charge_ah",
                    "voltage",
                    "current"
                ])
                for row in self.battery_log:
                    writer.writerow(row)

            # decisions
            with open(self.decision_file, "w", newline="") as f:
                writer = csv.writer(f, delimiter="\t")
                writer.writerow([
                    "time_s",
                    "score",
                    "extra_distance",
                    "margin",
                    "obj_distance",
                    "obj_to_charger_distance",
                    "predicted_drain",
                    "battery_pct",
                    "decision_mode_code"
                ])
                for row in self.decision_log:
                    writer.writerow(row)

            # events
            with open(self.events_file, "w", newline="") as f:
                writer = csv.writer(f, delimiter="\t")
                writer.writerow([
                    "time_s",
                    "event"
                ])
                for row in self.event_log:
                    writer.writerow(row)

            # charge-cycle summary
            with open(self.cycles_file, "w", newline="") as f:
                writer = csv.writer(f, delimiter="\t")
                writer.writerow([
                    "cycle_id",
                    "cycle_start_time_s",
                    "charge_start_time_s",
                    "cycle_duration_s",
                    "battery_pct_at_cycle_start",
                    "battery_pct_at_25",
                    "battery_pct_at_charge_start",
                    "battery_used_until_charge",
                    "guests_this_cycle",
                    "columns_this_cycle",
                    "guests_after_25",
                    "columns_after_25",
                    "time_battery_hit_25_s",
                    "distance_to_recharge_at_25_m",
                    "distance_traveled_this_cycle_m",
                    "distance_traveled_after_25_m",
                    "time_from_25_to_charge_s"
                ])
                for row in self.cycle_log:
                    writer.writerow(row)

            self.get_logger().info("Experiment data saved successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to save experiment data: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = DataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()