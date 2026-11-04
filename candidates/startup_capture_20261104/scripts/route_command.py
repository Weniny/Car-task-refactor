#!/usr/bin/env python3
"""Send one route command and confirm the controller's resulting state."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String


class RouteCommand(Node):
    def __init__(
        self,
        *,
        route_topic="/mission/route_enable",
        status_topic="/control/status",
        safe_topic="/cmd_vel_safe",
        chassis_topic="/cmd_vel",
        odom_topic="/odom",
        controller_node="/rebuild_replan_mppi",
    ):
        super().__init__("rebuild_route_command")
        self.route_topic = route_topic
        self.controller_node = controller_node
        self.status = None
        self.status_at = 0.0
        self.zero_inputs = {}
        self.start_sent = False
        self.start_confirmed = False
        self.publisher = self.create_publisher(
            Bool, route_topic, QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.create_subscription(String, status_topic, self._status, 10)
        self.create_subscription(Twist, safe_topic, lambda m: self._twist("safe", m), 10)
        self.create_subscription(Twist, chassis_topic, lambda m: self._twist("chassis", m), 10)
        self.create_subscription(Odometry, odom_topic, self._odom, 10)

    def _status(self, message):
        try:
            self.status = json.loads(message.data)
            self.status_at = time.monotonic()
        except (ValueError, TypeError):
            self.status = None

    def _twist(self, name, message):
        self.zero_inputs[name] = (
            time.monotonic(),
            max(abs(v) for v in (
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z,
            )) <= 1e-4,
        )

    def _odom(self, message):
        t = message.twist.twist
        self.zero_inputs["odom"] = (
            time.monotonic(),
            max(abs(t.linear.x), abs(t.linear.y), abs(t.angular.z)) <= 0.02,
        )

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def controller_discovered(self):
        subscriptions = self.get_subscriptions_info_by_topic(self.route_topic)
        target = [
            s for s in subscriptions
            if s.node_namespace.rstrip("/") + "/" + s.node_name == self.controller_node
        ]
        if len(target) != 1:
            return False
        if target[0].qos_profile.durability != DurabilityPolicy.VOLATILE:
            return False
        # A recorder is optional. Count actual compatible endpoints instead of
        # always requiring two, and require the controller itself to be present.
        compatible = sum(
            s.topic_type == "std_msgs/msg/Bool"
            and s.qos_profile.durability == DurabilityPolicy.VOLATILE
            for s in subscriptions
        )
        return self.publisher.get_subscription_count() >= compatible >= 1

    def command(self, enabled, timeout):
        stable_since = None

        def connected():
            nonlocal stable_since
            status_fresh = self.status is not None and time.monotonic() - self.status_at < 1.0
            if self.controller_discovered() and (status_fresh or not enabled):
                if stable_since is None:
                    stable_since = time.monotonic()
                return time.monotonic() - stable_since >= 0.5
            stable_since = None
            return False

        if not self.wait(connected, timeout):
            raise RuntimeError("Controller subscription did not become ready before timeout")
        if enabled and (
            self.status.get("route_enabled") is not False
            or self.status.get("status") != "ROUTE_DISABLED"
        ):
            raise RuntimeError("Controller must be ROUTE_DISABLED before a new start")

        sent_at = time.monotonic()
        self.publisher.publish(Bool(data=enabled))
        self.start_sent = enabled
        zero_since = None

        def acknowledged():
            nonlocal zero_since
            if self.status is None or self.status_at <= sent_at:
                return False
            if self.status.get("route_enabled") is not enabled:
                return False
            if enabled:
                return self.status.get("status") == "TRACKING"
            zeros = all(
                name in self.zero_inputs
                and self.zero_inputs[name][0] > sent_at
                and time.monotonic() - self.zero_inputs[name][0] < 1.0
                and self.zero_inputs[name][1]
                for name in ("safe", "chassis", "odom")
            )
            if not zeros:
                zero_since = None
                return False
            if zero_since is None:
                zero_since = time.monotonic()
            return time.monotonic() - zero_since >= 0.3

        if not self.wait(acknowledged, timeout):
            raise RuntimeError(
                "Controller did not confirm the requested state; last status="
                + json.dumps(self.status, ensure_ascii=False)
            )
        self.start_confirmed = enabled
        return {
            "result": "ACKNOWLEDGED",
            "route_enabled": enabled,
            "control_status": self.status.get("status"),
            "zero_commands_and_odom_confirmed": not enabled,
        }

    def cancel_unconfirmed_start(self):
        if self.start_sent and not self.start_confirmed:
            # Never keep repeating true. A failed/interrupted start is disarmed.
            for _ in range(3):
                self.publisher.publish(Bool(data=False))
                self.wait(lambda: False, 0.25)
            self.start_sent = False
            print("Unconfirmed start: route_enable=false sent; confirm physical stop.", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status"))
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.action == "start":
        # This is a read-only preflight. Only this explicit start action may arm.
        preflight = Path(__file__).with_name("navigation_preflight.py")
        try:
            result = subprocess.run(
                [sys.executable, str(preflight), "--timeout", "20"], timeout=35,
            )
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            print("Start cancelled before arming: preflight did not complete.", file=sys.stderr)
            return 1
        if result.returncode:
            print("Start refused: navigation preflight failed.", file=sys.stderr)
            return 1
    rclpy.init()
    node = RouteCommand()
    try:
        if args.action == "status":
            if not node.wait(lambda: node.status is not None, args.timeout):
                raise RuntimeError("No control status received")
            print(json.dumps(node.status, ensure_ascii=False, indent=2))
        else:
            print(json.dumps(node.command(args.action == "start", args.timeout), ensure_ascii=False))
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        print("Route command failed: " + str(exc), file=sys.stderr)
        return 1
    finally:
        node.cancel_unconfirmed_start()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
