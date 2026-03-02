#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState


def zero_twist() -> Twist:
    return Twist()


class CmdSource:
    """Holds last Twist and timestamp for a candidate source."""
    def __init__(self):
        self.msg = zero_twist()
        self.stamp = None  # rclpy.time.Time


class SupervisorArbiter(Node):
    """
    Supervisor Arbiter (MUX):
      - ONLY publisher to /cmd_vel
      - Subscribes to candidate cmd topics:
          /cmd_vel_nav
          /cmd_vel_avoid              (interaction avoidance)
          /cmd_vel_recharge
          /cmd_vel_soft_avoid         (recharge-only soft avoidance)
      - Uses gating topics:
          /recharge_active            (Bool)  True => recharge mode
          /navigation_paused          (Bool)  True => soft avoid wants control (during recharge)
          /avoid_active               (Bool)  True => interaction avoidance wants control (normal mode)
          /battery_charging           (Bool)  True => sitting at station, should not soft-avoid
      - Publishes:
          /cmd_vel
          /supervisor_active          (optional interlock for legacy nodes)
    """

    def __init__(self):
        super().__init__("supervisor_node")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.35)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)

        # -----------------------------
        # State / latches
        # -----------------------------
        self.recharge_active = False
        self.battery_charging = False

        self.navigation_paused = False      # published by soft avoidance (recharge-only)
        self.avoid_active = False           # published by interaction avoidance

        self.depart_active = False
        self.depart_end_time = None

        # Candidate command buffers
        self.nav = CmdSource()
        self.avoid = CmdSource()
        self.recharge = CmdSource()
        self.soft = CmdSource()

        # -----------------------------
        # Publishers
        # -----------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.supervisor_active_pub = self.create_publisher(Bool, "/supervisor_active", 10)

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

        # Optional: if move5 requests a supervisor “depart”
        self.create_subscription(Bool, "/depart_request", self._cb_depart_request, 10)

        # Main arbitration loop
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            "SupervisorArbiter running. Publishing /cmd_vel only. "
            "Candidates: nav(/cmd_vel_nav), avoid(/cmd_vel_avoid), "
            "recharge(/cmd_vel_recharge), soft(/cmd_vel_soft_avoid)."
        )

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
        self.recharge_active = bool(msg.data)

    def _cb_battery_charging(self, msg: Bool):
        self.battery_charging = bool(msg.data)

    def _cb_navigation_paused(self, msg: Bool):
        self.navigation_paused = bool(msg.data)

    def _cb_avoid_active(self, msg: Bool):
        self.avoid_active = bool(msg.data)

    def _cb_depart_request(self, msg: Bool):
        if bool(msg.data) and not self.depart_active:
            # Simple depart: back up for 1.0s then release
            self.depart_active = True
            self.depart_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=1.0)
            self.get_logger().info("Depart request received: executing 1.0s reverse.")

    # -----------------------------
    # Helpers
    # -----------------------------
    def _fresh(self, src: CmdSource, now) -> bool:
        if src.stamp is None:
            return False
        age = (now - src.stamp).nanoseconds * 1e-9
        return age <= self.cmd_timeout

    # -----------------------------
    # Arbitration tick
    # -----------------------------
    def _tick(self):
        now = self.get_clock().now()

        # Depart has absolute priority (optional feature)
        if self.depart_active:
            if self.depart_end_time is not None and now >= self.depart_end_time:
                self.depart_active = False
                self.depart_end_time = None
                self.cmd_pub.publish(zero_twist())
                self.supervisor_active_pub.publish(Bool(data=False))
                self.get_logger().info("Depart complete: releasing control.")
                return

            cmd = Twist()
            cmd.linear.x = -0.25
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.supervisor_active_pub.publish(Bool(data=True))
            return

        # Publish supervisor_active to pause legacy nodes only when we are in recharge mode
        # (In normal mode, let nav nodes publish candidates freely.)
        self.supervisor_active_pub.publish(Bool(data=self.recharge_active))

        # -----------------------------
        # RECHARGE MODE
        # -----------------------------
        if self.recharge_active:
            # Gate soft avoidance: ONLY while searching/driving to station.
            # If battery is charging, we are ARRIVED; soft avoidance must not intervene.
            soft_allowed = (not self.battery_charging)

            if soft_allowed and self.navigation_paused and self._fresh(self.soft, now):
                self.cmd_pub.publish(self.soft.msg)
                return

            if self._fresh(self.recharge, now):
                self.cmd_pub.publish(self.recharge.msg)
                return

            # Nothing fresh => stop
            self.cmd_pub.publish(zero_twist())
            return

        # -----------------------------
        # NORMAL MODE (PEOPLE BEHAVIOR)
        # -----------------------------
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