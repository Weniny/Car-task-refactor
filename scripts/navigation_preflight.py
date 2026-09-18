#!/usr/bin/env python3
"""Validate the live ROS graph before enabling the real navigation route."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from ranger_msgs.msg import SystemState
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_node_name(namespace: str, name: str) -> str:
    namespace = namespace.rstrip("/")
    return f"{namespace}/{name}" if namespace else f"/{name}"


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Preflight(Node):
    def __init__(self) -> None:
        super().__init__("rebuild_navigation_preflight")
        self.system_state = None
        self.local_stop = None
        self.local_stop_true_at = None
        self.control_status = None
        self.replan_status = None
        self.safe_cmd = None
        self.amcl_pose = None

        self.create_subscription(SystemState, "/system_state", self._system_cb, 10)
        self.create_subscription(Bool, "/planning/local_stop_request", self._stop_cb, 10)
        self.create_subscription(String, "/control/status", self._control_cb, 10)
        self.create_subscription(String, "/planning/local_replan_status", self._replan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._cmd_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _system_cb(self, msg: SystemState) -> None:
        self.system_state = msg

    def _stop_cb(self, msg: Bool) -> None:
        self.local_stop = bool(msg.data)
        if msg.data:
            self.local_stop_true_at = time.monotonic()

    def _control_cb(self, msg: String) -> None:
        try:
            self.control_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.control_status = {"status": "INVALID_JSON"}

    def _replan_cb(self, msg: String) -> None:
        try:
            self.replan_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.replan_status = {"status": "INVALID_JSON"}

    def _cmd_cb(self, msg: Twist) -> None:
        self.safe_cmd = msg

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg

    def ready(self) -> bool:
        return not self.missing_topics()

    def missing_topics(self) -> list[str]:
        values = {
            "/system_state": self.system_state,
            "/planning/local_stop_request": self.local_stop,
            "/control/status": self.control_status,
            "/planning/local_replan_status": self.replan_status,
            "/cmd_vel_safe": self.safe_cmd,
        }
        return [topic for topic, value in values.items() if value is None]

    def validate(self, start_x: float, start_y: float, start_yaw: float) -> list[str]:
        problems = []
        state = self.system_state
        if state.control_mode != 1:
            problems.append(f"control_mode is {state.control_mode}, expected 1")
        if state.error_code != 0:
            problems.append(f"chassis error_code is {state.error_code}")

        if self.local_stop:
            problems.append("local replanner currently requests stop")
        if self.local_stop_true_at is not None:
            age = time.monotonic() - self.local_stop_true_at
            if age < 2.0:
                problems.append(f"local replanner requested stop {age:.2f}s ago")

        control = self.control_status
        if control.get("status") != "ROUTE_DISABLED":
            problems.append(f"controller status is {control.get('status')}")
        if control.get("route_enabled") is not False:
            problems.append("controller route_enabled is not false")

        replan = self.replan_status
        if replan.get("stop_requested") is not False:
            problems.append(f"replanner status is {replan.get('status')}")

        cmd = self.safe_cmd
        command_values = (
            cmd.linear.x,
            cmd.linear.y,
            cmd.linear.z,
            cmd.angular.x,
            cmd.angular.y,
            cmd.angular.z,
        )
        if any(abs(value) > 1.0e-4 for value in command_values):
            problems.append("/cmd_vel_safe is non-zero while route is disabled")

        publishers = self.get_publishers_info_by_topic("/cmd_vel")
        publisher_names = {
            normalize_node_name(info.node_namespace, info.node_name)
            for info in publishers
        }
        expected_publisher = "/rebuild_replan_ranger_twist_adapter"
        if len(publishers) != 1 or publisher_names != {expected_publisher}:
            problems.append(
                f"/cmd_vel publishers are {sorted(publisher_names)}, expected only "
                f"{expected_publisher}"
            )

        subscribers = self.get_subscriptions_info_by_topic("/cmd_vel")
        subscriber_names = {
            normalize_node_name(info.node_namespace, info.node_name)
            for info in subscribers
        }
        if "/ranger_base_node" not in subscriber_names:
            problems.append("ranger_base_node is not subscribed to /cmd_vel")

        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "body", rclpy.time.Time()
            )
        except TransformException as exc:
            problems.append(f"map->body is unavailable: {exc}")
            return problems

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position_error = math.hypot(translation.x - start_x, translation.y - start_y)
        yaw = yaw_from_quaternion(rotation.z, rotation.w)
        yaw_error = abs(math.degrees(wrap(yaw - start_yaw)))
        if position_error > 0.30:
            problems.append(f"start position error is {position_error:.3f}m")
        if yaw_error > 12.0:
            problems.append(f"start heading error is {yaw_error:.2f}deg")

        print(
            json.dumps(
                {
                    "battery_voltage": float(state.battery_voltage),
                    "control_status": control.get("status"),
                    "replan_status": replan.get("status"),
                    "start_position_error_m": round(position_error, 4),
                    "start_heading_error_deg": round(yaw_error, 3),
                    "cmd_vel_publisher": expected_publisher,
                },
                ensure_ascii=False,
            )
        )
        return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--start-x", type=float, default=-16.083)
    parser.add_argument("--start-y", type=float, default=2.309)
    parser.add_argument("--start-yaw", type=float, default=3.0165)
    args = parser.parse_args()

    rclpy.init()
    node = Preflight()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.ready():
            print(
                "PRECHECK FAILED: no messages from "
                + ", ".join(node.missing_topics()),
                file=sys.stderr,
            )
            return 1

        settle_deadline = min(deadline, time.monotonic() + 2.2)
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        problems = node.validate(args.start_x, args.start_y, args.start_yaw)
        if problems:
            for problem in problems:
                print(f"PRECHECK FAILED: {problem}", file=sys.stderr)
            return 1
        print("PRECHECK PASSED")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
