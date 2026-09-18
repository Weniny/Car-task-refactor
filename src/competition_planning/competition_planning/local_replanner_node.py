#!/usr/bin/env python3
"""ROS adapter from the live inflated costmap to local Hybrid A*."""

from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path as FilePath
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from competition_planning.hybrid_astar_planner import (
    AsymmetricFootprint,
    HybridAStarTimeout,
)
from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
    concatenate_reference_paths,
    docking_context_checkpoint,
    docking_shelf_filter_is_active,
    filter_expected_docking_shelf_points,
    occupied_grid_cell_centers,
    precision_docking_work_sides,
    reference_prefix_to_checkpoint,
)
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


class LocalReplannerNode(Node):
    """Preserve the current ROS safety contract around local Hybrid A*."""

    def __init__(self) -> None:
        super().__init__("local_replanner")
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._base_frame = str(self.declare_parameter("base_frame", "body").value)
        trajectory_file = str(self.declare_parameter("trajectory_file", "").value)
        map_file = str(self.declare_parameter("map_file", "").value)
        semantic_map_file = str(
            self.declare_parameter("semantic_map_file", "").value
        )
        if not trajectory_file or not map_file or not semantic_map_file:
            raise ValueError(
                "trajectory_file, map_file, and semantic_map_file parameters are required"
            )
        self._reference_paths = _load_reference_paths(
            trajectory_file,
            self._map_frame,
        )
        self._reference_path = concatenate_reference_paths(self._reference_paths)
        self._static_map = OccupancyGridMap.from_yaml(map_file)
        with FilePath(semantic_map_file).open("r", encoding="utf-8") as stream:
            semantic_map = yaml.safe_load(stream) or {}
        semantic_points = semantic_map.get("points") or {}
        self._semantic_points = {
            str(ref_id): PathPoint(
                x=float(record["x"]),
                y=float(record["y"]),
                yaw=float(record["yaw"]),
                ref_id=str(ref_id),
            )
            for ref_id, record in semantic_points.items()
        }
        self._dock_work_sides = precision_docking_work_sides(
            semantic_map.get("dock_poses", [])
        )
        self._dock_departure_refs = {
            ref.strip()
            for ref in str(
                self.declare_parameter("docking_departure_refs_csv", "").value
            ).split(",")
            if ref.strip()
        }
        unknown_departure_refs = self._dock_departure_refs - set(self._dock_work_sides)
        if unknown_departure_refs:
            raise ValueError(
                "docking departure refs are not pickup/drop docks: "
                + ", ".join(sorted(unknown_departure_refs))
            )
        self._config = LocalReplanConfig(
            lookahead_distance_m=float(
                self.declare_parameter("lookahead_distance_m", 3.00).value
            ),
            inflation_radius_m=float(
                self.declare_parameter("inflation_radius_m", 0.04).value
            ),
            search_padding_m=float(
                self.declare_parameter("search_padding_m", 1.5).value
            ),
            sample_spacing_m=float(
                self.declare_parameter("sample_spacing_m", 0.10).value
            ),
            min_turning_radius_m=float(
                self.declare_parameter("min_turning_radius_m", 0.60).value
            ),
            step_length_m=float(
                self.declare_parameter("step_length_m", 0.20).value
            ),
            curvature_bins=int(self.declare_parameter("curvature_bins", 9).value),
            heading_bins=int(self.declare_parameter("heading_bins", 72).value),
            goal_position_tolerance_m=float(
                self.declare_parameter("goal_position_tolerance_m", 0.15).value
            ),
            goal_heading_tolerance_rad=math.radians(
                float(
                    self.declare_parameter("goal_heading_tolerance_deg", 8.0).value
                )
            ),
            reference_deviation_weight=float(
                self.declare_parameter("reference_deviation_weight", 2.0).value
            ),
            max_expansions=int(
                self.declare_parameter("max_expansions", 250_000).value
            ),
            planning_timeout_s=float(
                self.declare_parameter("planning_timeout_s", 2.0).value
            ),
            empty_history_planning_timeout_s=float(
                self.declare_parameter(
                    "empty_history_planning_timeout_s",
                    2.0,
                ).value
            ),
            relaxed_extension_timeout_s=float(
                self.declare_parameter(
                    "relaxed_extension_timeout_s",
                    0.75,
                ).value
            ),
            reference_search_window_points=int(
                self.declare_parameter(
                    "reference_search_window_points",
                    160,
                ).value
            ),
            relaxed_segment_entry_ref=str(
                self.declare_parameter("relaxed_segment_entry_ref", "").value
            ),
            relaxed_segment_exit_ref=str(
                self.declare_parameter("relaxed_segment_exit_ref", "").value
            ),
            relaxed_activation_distance_m=float(
                self.declare_parameter("relaxed_activation_distance_m", 0.0).value
            ),
            relaxed_reference_deviation_weight=float(
                self.declare_parameter(
                    "relaxed_reference_deviation_weight",
                    0.5,
                ).value
            ),
            relaxed_corridor_half_width_m=float(
                self.declare_parameter(
                    "relaxed_corridor_half_width_m",
                    0.85,
                ).value
            ),
            relaxed_step_length_m=float(
                self.declare_parameter("relaxed_step_length_m", 0.30).value
            ),
            relaxed_extension_curvature_bins=int(
                self.declare_parameter(
                    "relaxed_extension_curvature_bins",
                    7,
                ).value
            ),
            relaxed_goal_heading_tolerance_rad=math.radians(
                float(
                    self.declare_parameter(
                        "relaxed_goal_heading_tolerance_deg",
                        20.0,
                    ).value
                )
            ),
            trajectory_switch_improvement_ratio=float(
                self.declare_parameter(
                    "trajectory_switch_improvement_ratio",
                    0.15,
                ).value
            ),
            obstacle_clearance_distance_m=float(
                self.declare_parameter(
                    "obstacle_clearance_distance_m",
                    0.0,
                ).value
            ),
            obstacle_clearance_weight=float(
                self.declare_parameter(
                    "obstacle_clearance_weight",
                    0.0,
                ).value
            ),
            search_heuristic_weight=float(
                self.declare_parameter(
                    "search_heuristic_weight",
                    1.0,
                ).value
            ),
            obstacle_aware_heuristic=bool(
                self.declare_parameter("obstacle_aware_heuristic", False).value
            ),
            minimum_short_rejoin_distance_m=float(
                self.declare_parameter("minimum_short_rejoin_distance_m", 0.0).value
            ),
            reference_priority_segment_entry_ref=str(
                self.declare_parameter(
                    "reference_priority_segment_entry_ref",
                    "",
                ).value
            ).strip(),
            reference_priority_segment_exit_ref=str(
                self.declare_parameter(
                    "reference_priority_segment_exit_ref",
                    "",
                ).value
            ).strip(),
            reference_priority_activation_distance_m=float(
                self.declare_parameter(
                    "reference_priority_activation_distance_m",
                    0.0,
                ).value
            ),
            reference_priority_inflation_radius_m=float(
                self.declare_parameter(
                    "reference_priority_inflation_radius_m",
                    0.0,
                ).value
            ),
            reference_priority_footprint=AsymmetricFootprint(
                vehicle_length_m=float(
                    self.declare_parameter(
                        "reference_priority_vehicle_length_m",
                        0.72,
                    ).value
                ),
                vehicle_width_m=float(
                    self.declare_parameter(
                        "reference_priority_vehicle_width_m",
                        0.50,
                    ).value
                ),
                front_clearance_m=float(
                    self.declare_parameter(
                        "reference_priority_front_clearance_m",
                        0.05,
                    ).value
                ),
                rear_clearance_m=float(
                    self.declare_parameter(
                        "reference_priority_rear_clearance_m",
                        0.05,
                    ).value
                ),
                left_clearance_m=float(
                    self.declare_parameter(
                        "reference_priority_left_clearance_m",
                        0.05,
                    ).value
                ),
                right_clearance_m=float(
                    self.declare_parameter(
                        "reference_priority_right_clearance_m",
                        0.05,
                    ).value
                ),
            ),
        )
        self._planner = LocalTrajectoryPlanner(self._static_map, self._config)
        self._docking_activation_distance_m = float(
            self.declare_parameter("docking_activation_distance_m", 1.5).value
        )
        self._docking_min_turning_radius_m = float(
            self.declare_parameter(
                "docking_min_turning_radius_m",
                self._config.min_turning_radius_m,
            ).value
        )
        self._docking_raw_occupancy_threshold = int(
            self.declare_parameter("docking_costmap_occupancy_threshold", 100).value
        )
        self._docking_vehicle_length_m = float(
            self.declare_parameter("docking_vehicle_length_m", 0.72).value
        )
        self._docking_vehicle_width_m = float(
            self.declare_parameter("docking_vehicle_width_m", 0.50).value
        )
        self._docking_front_clearance_m = float(
            self.declare_parameter("docking_front_clearance_m", 0.10).value
        )
        self._docking_rear_clearance_m = float(
            self.declare_parameter("docking_rear_clearance_m", 0.10).value
        )
        self._docking_work_side_clearance_m = float(
            self.declare_parameter("docking_work_side_clearance_m", 0.03).value
        )
        self._docking_non_work_side_clearance_m = float(
            self.declare_parameter("docking_non_work_side_clearance_m", 0.10).value
        )
        self._docking_shelf_filter_distance_m = float(
            self.declare_parameter("docking_shelf_filter_distance_m", 1.00).value
        )
        self._docking_shelf_physical_guard_m = float(
            self.declare_parameter("docking_shelf_physical_guard_m", 0.05).value
        )
        self._final_obstacle_checkpoint_ref = str(
            self.declare_parameter(
                "final_obstacle_checkpoint_ref",
                "finish_park",
            ).value
        ).strip()
        self._final_obstacle_raw_occupancy_threshold = int(
            self.declare_parameter(
                "final_obstacle_costmap_occupancy_threshold",
                100,
            ).value
        )
        self._reference_priority_raw_occupancy_threshold = int(
            self.declare_parameter(
                "reference_priority_costmap_occupancy_threshold",
                100,
            ).value
        )
        self._final_obstacle_inflation_radius_m = float(
            self.declare_parameter(
                "final_obstacle_inflation_radius_m",
                0.05,
            ).value
        )
        self._final_obstacle_vehicle_length_m = float(
            self.declare_parameter(
                "final_obstacle_vehicle_length_m",
                0.72,
            ).value
        )
        self._final_obstacle_vehicle_width_m = float(
            self.declare_parameter(
                "final_obstacle_vehicle_width_m",
                0.50,
            ).value
        )
        self._final_obstacle_front_clearance_m = float(
            self.declare_parameter(
                "final_obstacle_front_clearance_m",
                0.05,
            ).value
        )
        self._final_obstacle_rear_clearance_m = float(
            self.declare_parameter(
                "final_obstacle_rear_clearance_m",
                0.05,
            ).value
        )
        self._final_obstacle_left_clearance_m = float(
            self.declare_parameter(
                "final_obstacle_left_clearance_m",
                0.05,
            ).value
        )
        self._final_obstacle_right_clearance_m = float(
            self.declare_parameter(
                "final_obstacle_right_clearance_m",
                0.05,
            ).value
        )
        if self._docking_activation_distance_m <= 0.0:
            raise ValueError("docking_activation_distance_m must be positive")
        if self._docking_min_turning_radius_m <= 0.0:
            raise ValueError("docking_min_turning_radius_m must be positive")
        if not 0 <= self._docking_raw_occupancy_threshold <= 100:
            raise ValueError("docking_costmap_occupancy_threshold must be in [0, 100]")
        if self._docking_shelf_filter_distance_m <= 0.0:
            raise ValueError("docking_shelf_filter_distance_m must be positive")
        if self._docking_shelf_physical_guard_m < 0.0:
            raise ValueError("docking_shelf_physical_guard_m must be non-negative")
        if not 0 <= self._final_obstacle_raw_occupancy_threshold <= 100:
            raise ValueError(
                "final_obstacle_costmap_occupancy_threshold must be in [0, 100]"
            )
        if not 0 <= self._reference_priority_raw_occupancy_threshold <= 100:
            raise ValueError(
                "reference_priority_costmap_occupancy_threshold must be in [0, 100]"
            )
        if self._final_obstacle_inflation_radius_m < 0.0:
            raise ValueError(
                "final_obstacle_inflation_radius_m must be non-negative"
            )
        if (
            self._final_obstacle_vehicle_length_m <= 0.0
            or self._final_obstacle_vehicle_width_m <= 0.0
        ):
            raise ValueError("final obstacle vehicle dimensions must be positive")
        if min(
            self._final_obstacle_front_clearance_m,
            self._final_obstacle_rear_clearance_m,
            self._final_obstacle_left_clearance_m,
            self._final_obstacle_right_clearance_m,
        ) < 0.0:
            raise ValueError("final obstacle clearances must be non-negative")
        self._docking_mode = False
        self._final_obstacle_mode = False

        obstacle_source = str(
            self.declare_parameter("obstacle_source", "costmap").value
        ).lower()
        if obstacle_source != "costmap":
            raise ValueError("obstacle_source must be 'costmap'")
        self._expected_obstacle_frame = str(
            self.declare_parameter("expected_obstacle_frame", "body").value
        )
        self._max_obstacle_age_s = float(
            self.declare_parameter("max_obstacle_age_s", 2.0).value
        )
        self._max_odom_age_s = float(
            self.declare_parameter("max_odom_age_s", 0.50).value
        )
        self._obstacle_x_min_m = float(
            self.declare_parameter("obstacle_x_min_m", 0.05).value
        )
        self._obstacle_x_max_m = float(
            self.declare_parameter("obstacle_x_max_m", 4.0).value
        )
        self._obstacle_y_half_width_m = float(
            self.declare_parameter("obstacle_y_half_width_m", 2.5).value
        )
        self._costmap_occupancy_threshold = int(
            self.declare_parameter("costmap_occupancy_threshold", 50).value
        )
        self._max_obstacle_points = int(
            self.declare_parameter("max_obstacle_points", 2000).value
        )
        if self._max_obstacle_age_s <= 0.0 or self._max_odom_age_s <= 0.0:
            raise ValueError("sensor age limits must be positive")
        if (
            self._obstacle_x_max_m <= self._obstacle_x_min_m
            or self._obstacle_y_half_width_m <= 0.0
        ):
            raise ValueError("obstacle bounds are invalid")
        if not 0 <= self._costmap_occupancy_threshold <= 100:
            raise ValueError("costmap_occupancy_threshold must be in [0, 100]")
        if self._max_obstacle_points < 1:
            raise ValueError("max_obstacle_points must be positive")

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._obstacle_points_map: tuple[tuple[float, float], ...] | None = None
        self._raw_obstacle_points_map: tuple[tuple[float, float], ...] | None = None
        self._final_obstacle_points_map: (
            tuple[tuple[float, float], ...] | None
        ) = None
        self._reference_priority_obstacle_points_map: (
            tuple[tuple[float, float], ...] | None
        ) = None
        self._obstacle_frame = ""
        self._obstacle_received_s = 0.0
        self._obstacle_header_stamp_s = 0.0
        self._odom_received_s = 0.0
        self._previous_reference_index = 0
        self._active_checkpoint_ref: str | None = None

        self._path_publisher = self.create_publisher(
            Path,
            str(
                self.declare_parameter(
                    "local_trajectory_topic",
                    "/planning/local_trajectory",
                ).value
            ),
            1,
        )
        self._stop_publisher = self.create_publisher(
            Bool,
            str(
                self.declare_parameter(
                    "local_stop_request_topic",
                    "/planning/local_stop_request",
                ).value
            ),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "status_topic",
                    "/planning/local_replan_status",
                ).value
            ),
            10,
        )
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        costmap_topic = str(
            self.declare_parameter(
                "costmap_topic",
                "/avoidance/local_costmap",
            ).value
        )
        self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self._costmap_callback,
            costmap_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.declare_parameter("odom_topic", "/odom").value),
            self._odom_callback,
            20,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_callback,
            10,
        )
        checkpoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "active_checkpoint_ref_topic",
                    "/mission/active_checkpoint_ref",
                ).value
            ),
            self._active_checkpoint_ref_callback,
            checkpoint_qos,
        )
        frequency_hz = float(self.declare_parameter("frequency_hz", 1.0).value)
        if frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be positive")
        self.create_timer(1.0 / frequency_hz, self._planning_cycle)
        self._stop_publisher.publish(Bool(data=True))
        self.get_logger().info(
            "Local Hybrid A* ready: "
            f"inflated_costmap={costmap_topic}, "
            f"lookahead={self._config.lookahead_distance_m:.2f}m, "
            f"inflation={self._config.inflation_radius_m:.2f}m, "
            f"frequency={frequency_hz:.2f}Hz"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        received_s = self._now_s()
        if (
            self._expected_obstacle_frame
            and message.header.frame_id != self._expected_obstacle_frame
        ):
            self._obstacle_points_map = None
            self._raw_obstacle_points_map = None
            self._final_obstacle_points_map = None
            self._reference_priority_obstacle_points_map = None
            self._publish_stop(
                "COSTMAP_FRAME_MISMATCH",
                detail=(
                    f"{message.header.frame_id}->{self._expected_obstacle_frame}"
                ),
            )
            return
        origin_yaw = _yaw_from_quaternion(message.info.origin.orientation)
        if abs(origin_yaw) > 1e-6:
            self._obstacle_points_map = None
            self._raw_obstacle_points_map = None
            self._final_obstacle_points_map = None
            self._reference_priority_obstacle_points_map = None
            self._publish_stop(
                "COSTMAP_ORIGIN_ROTATED",
                detail=f"yaw={origin_yaw:.6f}",
            )
            return

        try:
            map_from_obstacle = self._tf_buffer.lookup_transform(
                self._map_frame,
                message.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            obstacle_points = occupied_grid_cell_centers(
                message.data,
                width=int(message.info.width),
                height=int(message.info.height),
                resolution_m=float(message.info.resolution),
                origin_x_m=float(message.info.origin.position.x),
                origin_y_m=float(message.info.origin.position.y),
                occupancy_threshold=self._costmap_occupancy_threshold,
                x_min_m=self._obstacle_x_min_m,
                x_max_m=self._obstacle_x_max_m,
                y_half_width_m=self._obstacle_y_half_width_m,
                max_points=self._max_obstacle_points,
            )
            raw_obstacle_points = occupied_grid_cell_centers(
                message.data,
                width=int(message.info.width),
                height=int(message.info.height),
                resolution_m=float(message.info.resolution),
                origin_x_m=float(message.info.origin.position.x),
                origin_y_m=float(message.info.origin.position.y),
                occupancy_threshold=self._docking_raw_occupancy_threshold,
                x_min_m=self._obstacle_x_min_m,
                x_max_m=self._obstacle_x_max_m,
                y_half_width_m=self._obstacle_y_half_width_m,
                max_points=self._max_obstacle_points,
            )
            final_obstacle_points = occupied_grid_cell_centers(
                message.data,
                width=int(message.info.width),
                height=int(message.info.height),
                resolution_m=float(message.info.resolution),
                origin_x_m=float(message.info.origin.position.x),
                origin_y_m=float(message.info.origin.position.y),
                occupancy_threshold=self._final_obstacle_raw_occupancy_threshold,
                x_min_m=self._obstacle_x_min_m,
                x_max_m=self._obstacle_x_max_m,
                y_half_width_m=self._obstacle_y_half_width_m,
                max_points=self._max_obstacle_points,
            )
            reference_priority_obstacle_points = occupied_grid_cell_centers(
                message.data,
                width=int(message.info.width),
                height=int(message.info.height),
                resolution_m=float(message.info.resolution),
                origin_x_m=float(message.info.origin.position.x),
                origin_y_m=float(message.info.origin.position.y),
                occupancy_threshold=(
                    self._reference_priority_raw_occupancy_threshold
                ),
                x_min_m=self._obstacle_x_min_m,
                x_max_m=self._obstacle_x_max_m,
                y_half_width_m=self._obstacle_y_half_width_m,
                max_points=self._max_obstacle_points,
            )
        except (TransformException, ValueError) as exc:
            self._obstacle_points_map = None
            self._raw_obstacle_points_map = None
            self._final_obstacle_points_map = None
            self._reference_priority_obstacle_points_map = None
            self._publish_stop("COSTMAP_REJECTED", detail=str(exc))
            return

        translation = map_from_obstacle.transform.translation
        transform_yaw = _yaw_from_quaternion(
            map_from_obstacle.transform.rotation
        )
        cos_yaw = math.cos(transform_yaw)
        sin_yaw = math.sin(transform_yaw)
        self._obstacle_points_map = tuple(
            (
                float(translation.x)
                + cos_yaw * obstacle_x
                - sin_yaw * obstacle_y,
                float(translation.y)
                + sin_yaw * obstacle_x
                + cos_yaw * obstacle_y,
            )
            for obstacle_x, obstacle_y in obstacle_points
        )
        self._raw_obstacle_points_map = tuple(
            (
                float(translation.x)
                + cos_yaw * obstacle_x
                - sin_yaw * obstacle_y,
                float(translation.y)
                + sin_yaw * obstacle_x
                + cos_yaw * obstacle_y,
            )
            for obstacle_x, obstacle_y in raw_obstacle_points
        )
        self._final_obstacle_points_map = tuple(
            (
                float(translation.x)
                + cos_yaw * obstacle_x
                - sin_yaw * obstacle_y,
                float(translation.y)
                + sin_yaw * obstacle_x
                + cos_yaw * obstacle_y,
            )
            for obstacle_x, obstacle_y in final_obstacle_points
        )
        self._reference_priority_obstacle_points_map = tuple(
            (
                float(translation.x)
                + cos_yaw * obstacle_x
                - sin_yaw * obstacle_y,
                float(translation.y)
                + sin_yaw * obstacle_x
                + cos_yaw * obstacle_y,
            )
            for obstacle_x, obstacle_y in reference_priority_obstacle_points
        )
        self._obstacle_frame = message.header.frame_id
        self._obstacle_received_s = received_s
        self._obstacle_header_stamp_s = _stamp_to_seconds(message.header.stamp)

    def _odom_callback(self, message: Odometry) -> None:
        del message
        self._odom_received_s = self._now_s()

    def _initialpose_callback(self, message: PoseWithCovarianceStamped) -> None:
        frame_id = message.header.frame_id or self._map_frame
        if frame_id != self._map_frame:
            self._publish_stop(
                "INITIAL_POSE_FRAME_MISMATCH",
                detail=f"{frame_id}->{self._map_frame}",
            )
            return
        self._previous_reference_index = 0
        self._planner = LocalTrajectoryPlanner(self._static_map, self._config)
        self._docking_mode = False
        self._final_obstacle_mode = False
        self._publish_stop("INITIAL_POSE_RESET")

    def _active_checkpoint_ref_callback(self, message: String) -> None:
        checkpoint_ref = message.data.strip()
        if not checkpoint_ref or checkpoint_ref == self._active_checkpoint_ref:
            return
        self._active_checkpoint_ref = checkpoint_ref
        self._planner = LocalTrajectoryPlanner(self._static_map, self._config)
        self._docking_mode = False
        self._final_obstacle_mode = False
        # The controller keeps its last fresh, collision-checked local path
        # while this forward checkpoint retarget is replanned. Normal releases
        # from a stopped checkpoint remain held by the controller itself.
        self._publish_status(
            "ACTIVE_CHECKPOINT_UPDATED",
            stop_requested=False,
            detail=checkpoint_ref,
        )

    def _planning_cycle(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        docking_departure_context = False
        if self._obstacle_points_map is None or self._obstacle_received_s <= 0.0:
            self._publish_stop("WAITING_FOR_COSTMAP")
            return
        obstacle_age_s = now_s - self._obstacle_received_s
        if obstacle_age_s > self._max_obstacle_age_s:
            self._publish_stop(
                "COSTMAP_STALE",
                obstacle_age_s=obstacle_age_s,
            )
            return
        if self._odom_received_s <= 0.0:
            self._publish_stop(
                "WAITING_FOR_ODOMETRY",
                obstacle_age_s=obstacle_age_s,
            )
            return
        odom_age_s = now_s - self._odom_received_s
        if odom_age_s > self._max_odom_age_s:
            self._publish_stop(
                "ODOMETRY_STALE",
                obstacle_age_s=obstacle_age_s,
                odom_age_s=odom_age_s,
            )
            return

        docking_filtered_obstacle_count = 0
        try:
            reference_path = self._reference_path
            if self._active_checkpoint_ref is not None:
                checkpoint = self._semantic_points.get(self._active_checkpoint_ref)
                if checkpoint is None:
                    raise ValueError(
                        "active checkpoint missing from semantic map: "
                        f"{self._active_checkpoint_ref}"
                    )
                reference_path = reference_prefix_to_checkpoint(
                    reference_path,
                    checkpoint,
                    exact_pose=(
                        self._active_checkpoint_ref in self._dock_work_sides
                    ),
                    minimum_index=self._previous_reference_index,
                )
            map_from_base = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            current_pose = PathPoint(
                x=float(map_from_base.transform.translation.x),
                y=float(map_from_base.transform.translation.y),
                yaw=_yaw_from_quaternion(map_from_base.transform.rotation),
            )
            active_checkpoint = (
                self._semantic_points.get(self._active_checkpoint_ref)
                if self._active_checkpoint_ref is not None
                else None
            )
            docking_checkpoint, docking_checkpoint_ref = docking_context_checkpoint(
                current_pose=current_pose,
                active_checkpoint=active_checkpoint,
                active_checkpoint_ref=self._active_checkpoint_ref,
                departure_checkpoints={
                    ref: self._semantic_points[ref]
                    for ref in self._dock_departure_refs
                    if ref in self._semantic_points
                },
                docking_refs=set(self._dock_work_sides),
                activation_distance_m=self._docking_activation_distance_m,
            )
            docking_mode = docking_checkpoint is not None
            final_obstacle_mode = (
                not docking_mode
                and self._active_checkpoint_ref
                == self._final_obstacle_checkpoint_ref
            )
            docking_departure_context = (
                docking_mode
                and docking_checkpoint_ref is not None
                and docking_checkpoint_ref != self._active_checkpoint_ref
            )
            if (
                docking_mode != self._docking_mode
                or final_obstacle_mode != self._final_obstacle_mode
            ):
                self._docking_mode = docking_mode
                self._final_obstacle_mode = final_obstacle_mode
                self._planner = LocalTrajectoryPlanner(
                    self._static_map,
                    self._docking_config(docking_checkpoint_ref)
                    if docking_mode
                    else (
                        self._final_obstacle_config()
                        if final_obstacle_mode
                        else self._config
                    ),
                )
            obstacle_points = (
                self._raw_obstacle_points_map
                if docking_mode
                else (
                    self._final_obstacle_points_map
                    if final_obstacle_mode
                    else self._obstacle_points_map
                )
            )
            if obstacle_points is None:
                raise ValueError("selected costmap has no occupied cells")
            if (
                docking_mode
                and docking_checkpoint is not None
                and docking_shelf_filter_is_active(
                    current_pose=current_pose,
                    checkpoint=docking_checkpoint,
                    activation_distance_m=self._docking_shelf_filter_distance_m,
                    heading_tolerance_rad=self._config.goal_heading_tolerance_rad,
                )
            ):
                obstacle_points, docking_filtered_obstacle_count = (
                    filter_expected_docking_shelf_points(
                        obstacle_points,
                        checkpoint=docking_checkpoint,
                        work_side=self._dock_work_sides.get(
                            str(docking_checkpoint_ref),
                            "RIGHT",
                        ),
                        vehicle_length_m=self._docking_vehicle_length_m,
                        vehicle_width_m=self._docking_vehicle_width_m,
                        front_clearance_m=self._docking_front_clearance_m,
                        approach_distance_m=self._docking_shelf_filter_distance_m,
                        physical_guard_m=self._docking_shelf_physical_guard_m,
                    )
                )
            started_at = time.perf_counter()
            result = self._planner.plan(
                reference_path=reference_path,
                current_pose=current_pose,
                dynamic_obstacle_points=obstacle_points,
                reference_priority_obstacle_points=(
                    self._reference_priority_obstacle_points_map
                ),
                previous_reference_index=self._previous_reference_index,
                departure_context=docking_departure_context,
            )
            planning_time_ms = (time.perf_counter() - started_at) * 1000.0
        except HybridAStarTimeout as exc:
            self._publish_stop(
                "HYBRID_ASTAR_TIMEOUT",
                detail=str(exc),
                rejoin_attempts=getattr(exc, "rejoin_attempts", []),
                obstacle_age_s=obstacle_age_s,
                odom_age_s=odom_age_s,
                planning_time_ms=(time.perf_counter() - started_at) * 1000.0,
                docking_departure_context=docking_departure_context,
                docking_filtered_obstacle_count=docking_filtered_obstacle_count,
            )
            return
        except (TransformException, GridPlanningError, ValueError) as exc:
            self._publish_stop(
                "HYBRID_ASTAR_NO_FEASIBLE_PATH",
                detail=str(exc),
                rejoin_attempts=getattr(exc, "rejoin_attempts", []),
                obstacle_age_s=obstacle_age_s,
                odom_age_s=odom_age_s,
                docking_departure_context=docking_departure_context,
                docking_filtered_obstacle_count=docking_filtered_obstacle_count,
            )
            return

        self._previous_reference_index = result.reference_start_index
        publish_now = self.get_clock().now()
        self._path_publisher.publish(
            _path_message(result.path, self._map_frame, publish_now.to_msg())
        )
        self._stop_publisher.publish(Bool(data=False))
        self._publish_status(
            result.status,
            stop_requested=False,
            obstacle_age_s=obstacle_age_s,
            obstacle_header_age_s=(
                now_s - self._obstacle_header_stamp_s
                if self._obstacle_header_stamp_s > 0.0
                else None
            ),
            odom_age_s=odom_age_s,
            planning_time_ms=planning_time_ms,
            obstacle_count=result.dynamic_obstacle_count,
            path_point_count=len(result.path),
            reference_start_index=result.reference_start_index,
            rejoin_index=result.rejoin_index,
            planning_grid_cell_count=result.planning_grid_cell_count,
            docking_mode=self._docking_mode,
            final_obstacle_mode=self._final_obstacle_mode,
            reference_priority_mode=(
                result.status == "REFERENCE_PRIORITY_CLEAR"
            ),
            docking_context_ref=docking_checkpoint_ref,
            docking_departure_context=docking_departure_context,
            docking_filtered_obstacle_count=docking_filtered_obstacle_count,
        )

    def _docking_config(self, checkpoint_ref: str | None) -> LocalReplanConfig:
        work_side = self._dock_work_sides.get(str(checkpoint_ref), "RIGHT")
        if work_side not in {"LEFT", "RIGHT"}:
            raise ValueError(f"unsupported docking work_side: {work_side}")
        left_clearance = (
            self._docking_work_side_clearance_m
            if work_side == "LEFT"
            else self._docking_non_work_side_clearance_m
        )
        right_clearance = (
            self._docking_work_side_clearance_m
            if work_side == "RIGHT"
            else self._docking_non_work_side_clearance_m
        )
        return replace(
            self._config,
            inflation_radius_m=0.0,
            min_turning_radius_m=self._docking_min_turning_radius_m,
            footprint=AsymmetricFootprint(
                vehicle_length_m=self._docking_vehicle_length_m,
                vehicle_width_m=self._docking_vehicle_width_m,
                front_clearance_m=self._docking_front_clearance_m,
                rear_clearance_m=self._docking_rear_clearance_m,
                left_clearance_m=left_clearance,
                right_clearance_m=right_clearance,
            ),
        )

    def _final_obstacle_config(self) -> LocalReplanConfig:
        return replace(
            self._config,
            inflation_radius_m=self._final_obstacle_inflation_radius_m,
            footprint=AsymmetricFootprint(
                vehicle_length_m=self._final_obstacle_vehicle_length_m,
                vehicle_width_m=self._final_obstacle_vehicle_width_m,
                front_clearance_m=self._final_obstacle_front_clearance_m,
                rear_clearance_m=self._final_obstacle_rear_clearance_m,
                left_clearance_m=self._final_obstacle_left_clearance_m,
                right_clearance_m=self._final_obstacle_right_clearance_m,
            ),
        )

    def _publish_stop(self, status: str, **fields: object) -> None:
        self._stop_publisher.publish(Bool(data=True))
        self._publish_status(status, stop_requested=True, **fields)

    def _publish_status(self, status: str, **fields: object) -> None:
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "status": status,
                        "costmap_frame": self._obstacle_frame,
                        "docking_mode": getattr(self, "_docking_mode", False),
                        "final_obstacle_mode": getattr(
                            self,
                            "_final_obstacle_mode",
                            False,
                        ),
                        **fields,
                    },
                    separators=(",", ":"),
                )
            )
        )


def _load_reference_paths(
    path: str,
    expected_frame: str,
) -> tuple[tuple[PathPoint, ...], ...]:
    with FilePath(path).open("r", encoding="utf-8") as stream:
        artifact = yaml.safe_load(stream)
    if not isinstance(artifact, dict) or not artifact.get("ok", False):
        raise ValueError(f"trajectory_file is not a successful artifact: {path}")
    frame_id = str(artifact.get("frame_id", "map"))
    if frame_id != expected_frame:
        raise ValueError(
            f"trajectory frame_id={frame_id} does not match map_frame={expected_frame}"
        )
    point_record_groups = []
    point_records = artifact.get("points", [])
    if point_records:
        point_record_groups.append(point_records)
    else:
        trajectories = artifact.get("trajectories", [])
        if not isinstance(trajectories, list) or not trajectories:
            raise ValueError("trajectory artifact has no reference path")
        for trajectory in trajectories:
            if not isinstance(trajectory, dict):
                raise ValueError("trajectory artifact contains an invalid segment")
            point_record_groups.append(trajectory.get("points", []))
    paths = tuple(
        tuple(
            PathPoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point["yaw"]),
                ref_id=str(point["ref_id"]) if point.get("ref_id") else None,
            )
            for point in records
        )
        for records in point_record_groups
    )
    if not paths or any(len(points) < 2 for points in paths):
        raise ValueError("trajectory artifact requires at least two points")
    return paths


def _load_reference_path(path: str, expected_frame: str) -> tuple[PathPoint, ...]:
    return _load_reference_paths(path, expected_frame)[0]


def _path_message(
    points: tuple[PathPoint, ...],
    frame_id: str,
    stamp,
) -> Path:
    message = Path()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    for point in points:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(point.yaw * 0.5)
        pose.pose.orientation.w = math.cos(point.yaw * 0.5)
        message.poses.append(pose)
    return message


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0
        * (
            float(quaternion.w) * float(quaternion.z)
            + float(quaternion.x) * float(quaternion.y)
        ),
        1.0
        - 2.0
        * (
            float(quaternion.y) ** 2
            + float(quaternion.z) ** 2
        ),
    )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = LocalReplannerNode()
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
