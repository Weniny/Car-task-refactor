#!/usr/bin/env python3
"""Publish a conservative stop request and inflated grid from a 2D scan."""

from __future__ import annotations

from collections import deque
import json

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from competition_safety.proximity_stop import (
    LocalClearanceResult,
    LocalGridConfig,
    ProximityStopConfig,
    advance_periodic_deadline,
    evaluate_local_clearance,
    evaluate_fused_local_clearance,
    laser_scan_points,
)


class ProximityStopNode(Node):
    def __init__(self) -> None:
        super().__init__("proximity_stop")
        input_type = str(
            self.declare_parameter("input_type", "laser_scan").value
        ).lower()
        if input_type != "laser_scan":
            raise ValueError("input_type must be 'laser_scan'")
        self._input_scan_topic = str(
            self.declare_parameter("input_scan_topic", "/scan").value
        )
        scan_qos_reliability = str(
            self.declare_parameter("scan_qos_reliability", "best_effort").value
        ).lower()
        scan_qos_depth = int(self.declare_parameter("scan_qos_depth", 1).value)
        if scan_qos_reliability not in {"best_effort", "reliable"}:
            raise ValueError(
                "scan_qos_reliability must be 'best_effort' or 'reliable'"
            )
        if scan_qos_depth < 1:
            raise ValueError("scan_qos_depth must be at least 1")
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=scan_qos_depth,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if scan_qos_reliability == "best_effort"
                else ReliabilityPolicy.RELIABLE
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        self._expected_frame_id = str(
            self.declare_parameter("expected_frame_id", "body").value
        )
        self._max_scan_age_s = float(
            self.declare_parameter("max_scan_age_s", 0.25).value
        )
        if self._max_scan_age_s <= 0.0:
            raise ValueError("max_scan_age_s must be positive")
        self._config = ProximityStopConfig(
            x_min_m=float(self.declare_parameter("x_min_m", 0.25).value),
            stop_distance_m=float(
                self.declare_parameter("stop_distance_m", 0.55).value
            ),
            front_half_angle_rad=float(
                self.declare_parameter("front_half_angle_rad", 0.4363).value
            ),
            lateral_half_width_m=float(
                self.declare_parameter("lateral_half_width_m", 0.45).value
            ),
            z_min_m=float(self.declare_parameter("z_min_m", -0.25).value),
            z_max_m=float(self.declare_parameter("z_max_m", 0.80).value),
            min_points=int(self.declare_parameter("min_points", 3).value),
        )
        self._grid_config = LocalGridConfig(
            resolution_m=float(
                self.declare_parameter("grid_resolution_m", 0.05).value
            ),
            x_min_m=float(self.declare_parameter("grid_x_min_m", -0.50).value),
            x_max_m=float(self.declare_parameter("grid_x_max_m", 3.00).value),
            y_min_m=float(self.declare_parameter("grid_y_min_m", -1.50).value),
            y_max_m=float(self.declare_parameter("grid_y_max_m", 1.50).value),
            inflation_radius_m=float(
                self.declare_parameter("grid_inflation_radius_m", 0.20).value
            ),
            scan_bin_count=int(self.declare_parameter("scan_bin_count", 360).value),
            scan_range_min_m=float(
                self.declare_parameter("scan_range_min_m", 0.10).value
            ),
            scan_range_max_m=float(
                self.declare_parameter("scan_range_max_m", 6.00).value
            ),
        )
        visualization_rate_hz = float(
            self.declare_parameter("visualization_rate_hz", 5.0).value
        )
        if visualization_rate_hz <= 0.0:
            raise ValueError("visualization_rate_hz must be positive")
        fusion_frame_count = int(
            self.declare_parameter("fusion_frame_count", 10).value
        )
        if fusion_frame_count < 1:
            raise ValueError("fusion_frame_count must be at least 1")
        self._visualization_period_s = 1.0 / visualization_rate_hz
        self._next_visualization_s: float | None = None
        self._visualization_point_frames: deque[
            tuple[tuple[float, float, float], ...]
        ] = deque(maxlen=fusion_frame_count)
        self._vehicle_length_m = float(
            self.declare_parameter("vehicle_length_m", 0.72).value
        )
        self._vehicle_width_m = float(
            self.declare_parameter("vehicle_width_m", 0.50).value
        )
        if self._vehicle_length_m <= 0.0 or self._vehicle_width_m <= 0.0:
            raise ValueError("vehicle dimensions must be positive")
        self._stop_publisher = self.create_publisher(
            Bool,
            str(
                self.declare_parameter(
                    "stop_request_topic",
                    "/avoidance/stop_request",
                ).value
            ),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "status_topic",
                    "/avoidance/proximity_status",
                ).value
            ),
            10,
        )
        self._costmap_publisher = self.create_publisher(
            OccupancyGrid,
            str(
                self.declare_parameter(
                    "costmap_topic",
                    "/avoidance/local_costmap",
                ).value
            ),
            1,
        )
        self._scan_publisher = self.create_publisher(
            LaserScan,
            str(
                self.declare_parameter(
                    "scan_topic",
                    "/avoidance/scan",
                ).value
            ),
            scan_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            str(
                self.declare_parameter(
                    "marker_topic",
                    "/avoidance/markers",
                ).value
            ),
            1,
        )
        self.create_subscription(
            LaserScan,
            self._input_scan_topic,
            self._scan_callback,
            scan_qos,
        )
        distance_stop_description = (
            "disabled"
            if self._config.stop_distance_m == 0.0
            else (
                f"[{self._config.x_min_m:.2f}, "
                f"{self._config.stop_distance_m:.2f}]"
            )
        )
        self.get_logger().info(
            "Proximity stop ready: "
            f"scan_topic={self._input_scan_topic}, "
            f"distance_stop={distance_stop_description}, "
            f"half_angle={self._config.front_half_angle_rad:.3f}rad, "
            f"lateral_half_width={self._config.lateral_half_width_m:.2f}, "
            f"z=[{self._config.z_min_m:.2f}, {self._config.z_max_m:.2f}], "
            f"min_points={self._config.min_points}, "
            f"obstacle_layer={visualization_rate_hz:.1f}Hz/"
            f"{fusion_frame_count}_frame_fusion, "
            f"qos={scan_qos_reliability}/keep_last_{scan_qos_depth}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _scan_callback(self, message: LaserScan) -> None:
        now_s = self._now_s()
        visualization_due, self._next_visualization_s = (
            advance_periodic_deadline(
                now_s=now_s,
                next_deadline_s=self._next_visualization_s,
                period_s=self._visualization_period_s,
            )
        )
        frame_id = message.header.frame_id
        if self._expected_frame_id and frame_id != self._expected_frame_id:
            self._visualization_point_frames.clear()
            self._publish(True, "frame_mismatch", 0, frame_id, None, None)
            if visualization_due:
                result = evaluate_local_clearance(
                    [],
                    self._config,
                    self._grid_config,
                )
                self._publish_visualization(
                    message.header.stamp,
                    self._expected_frame_id,
                    result,
                    True,
                    "frame_mismatch",
                )
            return

        scan_stamp_s = _stamp_to_seconds(message.header.stamp)
        age_s = now_s - scan_stamp_s if scan_stamp_s > 0.0 else None
        if age_s is not None and age_s > self._max_scan_age_s:
            self._visualization_point_frames.clear()
            self._publish(True, "stale_scan", 0, frame_id, age_s, None)
            if visualization_due:
                result = evaluate_local_clearance(
                    [],
                    self._config,
                    self._grid_config,
                )
                self._publish_visualization(
                    message.header.stamp,
                    frame_id,
                    result,
                    True,
                    "stale_scan",
                )
            return

        points = laser_scan_points(
            message.ranges,
            angle_min_rad=float(message.angle_min),
            angle_increment_rad=float(message.angle_increment),
            range_min_m=max(float(message.range_min), self._grid_config.scan_range_min_m),
            range_max_m=min(float(message.range_max), self._grid_config.scan_range_max_m),
        )
        self._visualization_point_frames.append(points)
        result = evaluate_local_clearance(
            points,
            self._config,
            self._grid_config,
        )
        stop = result.stop
        count = result.point_count
        nearest_distance_m = result.nearest_obstacle_distance_m
        reason = "obstacle_in_stop_box" if stop else "clear"
        self._publish(
            stop,
            reason,
            count,
            frame_id,
            age_s,
            nearest_distance_m,
        )
        if visualization_due:
            visualization_result = evaluate_fused_local_clearance(
                self._visualization_point_frames,
                self._config,
                self._grid_config,
            )
            self._publish_visualization(
                message.header.stamp,
                frame_id,
                visualization_result,
                stop,
                reason,
            )

    def _publish(
        self,
        stop: bool,
        reason: str,
        point_count: int,
        frame_id: str,
        scan_age_s: float | None,
        nearest_obstacle_distance_m: float | None,
    ) -> None:
        self._stop_publisher.publish(Bool(data=stop))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "stop": stop,
                        "reason": reason,
                        "point_count": point_count,
                        "frame_id": frame_id,
                        "scan_age_s": scan_age_s,
                        "nearest_obstacle_distance_m": nearest_obstacle_distance_m,
                    },
                    separators=(",", ":"),
                )
            )
        )

    def _publish_visualization(
        self,
        stamp,
        frame_id: str,
        result: LocalClearanceResult,
        stop: bool,
        reason: str,
    ) -> None:
        costmap = OccupancyGrid()
        costmap.header.stamp = stamp
        costmap.header.frame_id = frame_id
        costmap.info.resolution = result.costmap.resolution_m
        costmap.info.width = result.costmap.width
        costmap.info.height = result.costmap.height
        costmap.info.origin.position.x = result.costmap.origin_x_m
        costmap.info.origin.position.y = result.costmap.origin_y_m
        costmap.info.origin.orientation.w = 1.0
        costmap.data = list(result.costmap.data)
        self._costmap_publisher.publish(costmap)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = frame_id
        scan.angle_min = result.scan_angle_min_rad
        scan.angle_increment = result.scan_angle_increment_rad
        scan.angle_max = scan.angle_min + scan.angle_increment * (
            len(result.scan_ranges_m) - 1
        )
        scan.scan_time = self._visualization_period_s
        scan.range_min = self._grid_config.scan_range_min_m
        scan.range_max = self._grid_config.scan_range_max_m
        scan.ranges = list(result.scan_ranges_m)
        self._scan_publisher.publish(scan)

        self._marker_publisher.publish(
            self._visualization_markers(stamp, frame_id, result, stop, reason)
        )

    def _visualization_markers(
        self,
        stamp,
        frame_id: str,
        result: LocalClearanceResult,
        stop: bool,
        reason: str,
    ) -> MarkerArray:
        footprint = Marker()
        footprint.header.stamp = stamp
        footprint.header.frame_id = frame_id
        footprint.ns = "day5_clearance"
        footprint.id = 0
        footprint.type = Marker.CUBE
        footprint.action = Marker.ADD
        footprint.pose.orientation.w = 1.0
        footprint.pose.position.z = 0.03
        footprint.scale.x = self._vehicle_length_m
        footprint.scale.y = self._vehicle_width_m
        footprint.scale.z = 0.06
        footprint.color.b = 1.0
        footprint.color.a = 0.25

        corridor = Marker()
        corridor.header = footprint.header
        corridor.ns = footprint.ns
        corridor.id = 1
        corridor.type = Marker.CUBE
        corridor.action = Marker.ADD
        corridor.pose.orientation.w = 1.0
        corridor.pose.position.x = (
            self._config.x_min_m + self._config.stop_distance_m
        ) / 2.0
        corridor.pose.position.z = 0.04
        corridor.scale.x = self._config.stop_distance_m - self._config.x_min_m
        corridor.scale.y = 2.0 * self._config.lateral_half_width_m
        corridor.scale.z = 0.08
        corridor.color.r = 1.0 if stop else 0.0
        corridor.color.g = 0.0 if stop else 1.0
        corridor.color.a = 0.30

        status = Marker()
        status.header = footprint.header
        status.ns = footprint.ns
        status.id = 2
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.orientation.w = 1.0
        status.pose.position.z = 1.0
        status.scale.z = 0.22
        status.color.r = 1.0 if stop else 0.0
        status.color.g = 0.0 if stop else 1.0
        status.color.a = 1.0
        nearest = result.nearest_obstacle_distance_m
        nearest_text = "n/a" if nearest is None else f"{nearest:.2f}m"
        status.text = (
            f"{reason} | points={result.point_count} | nearest={nearest_text}"
        )
        return MarkerArray(markers=[footprint, corridor, status])


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = ProximityStopNode()
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
