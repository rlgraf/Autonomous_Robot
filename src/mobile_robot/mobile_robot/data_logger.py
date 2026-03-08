#!/usr/bin/env python3

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray


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

        self.get_logger().info(f"Logging experiment to {self.run_dir}")

        # ------------------------------------------------
        # Runtime storage
        # ------------------------------------------------

        self.start_time = self.get_clock().now()

        self.path_log = []
        self.battery_log = []
        self.decision_log = []
        self.event_log = []

        self.latest_pose = None
        self.current_charging = False

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

        # sample pose every second
        self.create_timer(1.0, self.sample_pose)

    # ------------------------------------------------
    # Time helper
    # ------------------------------------------------

    def get_time(self):

        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds * 1e-9

    # ------------------------------------------------
    # Odometry
    # ------------------------------------------------

    def odom_callback(self, msg: Odometry):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        self.latest_pose = (
            float(p.x),
            float(p.y),
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

        self.battery_log.append([
            t, pct, charge, voltage, current
        ])

    # ------------------------------------------------
    # Charging state
    # ------------------------------------------------

    def charging_callback(self, msg: Bool):

        t = self.get_time()

        if msg.data != self.current_charging:

            self.current_charging = msg.data

            self.event_log.append([
                t,
                "charging_start" if msg.data else "charging_stop"
            ])

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
                    "obj_to_station_distance",
                    "decision_mode"
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