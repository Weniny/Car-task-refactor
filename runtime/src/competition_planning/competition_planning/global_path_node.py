#!/usr/bin/env python3
"""Publish deterministic semantic global paths as nav_msgs/Path."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node

from competition_planning.semantic_planner import load_yaml_file, plan_route


class SemanticGlobalPathNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_global_path")
        self.route_file = str(self.declare_parameter("route_file", "").value)
        self.semantic_map_file = str(self.declare_parameter("semantic_map_file", "").value)
        self.planning_params_file = str(
            self.declare_parameter("planning_params_file", "").value
        )
        self.map_file = str(self.declare_parameter("map_file", "").value)
        self.step_id = str(self.declare_parameter("step_id", "go_traffic_light_1").value)
        self.publish_topic = str(
            self.declare_parameter("publish_topic", "/planning/global_path").value
        )
        publish_period_s = float(self.declare_parameter("publish_period_s", 1.0).value)

        self._publisher = self.create_publisher(PathMsg, self.publish_topic, 10)
        self._path = self._load_path()
        self._timer = self.create_timer(publish_period_s, self._publish_path)

    def _load_path(self) -> PathMsg:
        if not self.route_file or not self.semantic_map_file:
            raise RuntimeError("route_file and semantic_map_file parameters are required")

        route = load_yaml_file(_resolve_path(self.route_file))
        semantic_map = load_yaml_file(_resolve_path(self.semantic_map_file))
        params = (
            load_yaml_file(_resolve_path(self.planning_params_file))
            if self.planning_params_file
            else {}
        )
        if self.map_file:
            params.setdefault("global_planner", {})["map_file"] = self.map_file
        if not self.step_id:
            raise RuntimeError("step_id is required for /planning/global_path publication")

        result = plan_route(route, semantic_map, params)
        selected_failures = [
            failure for failure in result.failures if failure.step_id == self.step_id
        ]
        if selected_failures:
            for failure in selected_failures:
                self.get_logger().error(
                    f"{failure.step_id}: {failure.reason}: {failure.detail}"
                )
            raise RuntimeError(f"semantic global planning failed for {self.step_id}")

        plans = [plan for plan in result.plans if plan.step_id == self.step_id]
        if not plans:
            raise RuntimeError(f"step_id {self.step_id} did not produce a path")

        message = PathMsg()
        message.header.frame_id = result.frame_id
        plan = plans[0]
        for point in plan.path:
            pose = PoseStamped()
            pose.header.frame_id = result.frame_id
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(point.yaw / 2.0)
            pose.pose.orientation.w = math.cos(point.yaw / 2.0)
            message.poses.append(pose)
        self.get_logger().info(
            f"loaded {plan.step_id}: {len(plan.path)} points, "
            f"{plan.path_length_m:.3f} m, {plan.planning_time_ms:.3f} ms, "
            f"plugin={plan.planner_plugin}, smoother={plan.smoother_plugin}"
        )
        return message

    def _publish_path(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._path.header.stamp = stamp
        for pose in self._path.poses:
            pose.header.stamp = stamp
        self._publisher.publish(self._path)


def _resolve_path(path: str) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return Path.cwd() / candidate


def main() -> None:
    rclpy.init()
    node = SemanticGlobalPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
