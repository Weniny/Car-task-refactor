#!/usr/bin/env python3
"""Guarded local Hybrid A* replanning integration."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WS = Path("/home/agilex/competition_rebuild_ws")


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def generate_launch_description() -> LaunchDescription:
    control = _load_yaml(WS / "config/control/control_params_rebuild_020mps.yaml")
    safety_config = _load_yaml(
        WS / "config/safety/safety_params_rebuild_020mps.yaml"
    )

    tracker = control["trajectory_tracker"]
    mppi = tracker["mppi"]
    motion = control["motion"]
    estimator = control["state_estimator"]
    visualization = control["visualization"]
    safety = safety_config["safety"]

    trajectory_file = LaunchConfiguration("trajectory_file")
    route_file = str(WS / "routes/indoor_manual_six_anchor_v3_radius090_loop.yaml")
    semantic_map_file = str(
        WS / "maps/indoor_manual_final/semantic_map_six_anchor_v3_radius090.yaml"
    )
    planning_params_file = str(
        WS / "config/planning/semantic_lidar_guarded_six_anchor_radius090.yaml"
    )
    optimizer_params_file = str(
        WS / "config/planning/continuous_trajectory_radius090_turnrate250_020mps.yaml"
    )
    map_file = str(WS / "maps/indoor_manual_final/map.yaml")
    base_frame = str(estimator.get("tracking_base_frame", estimator["base_frame"]))

    scan = dict(safety_config["pointcloud_to_laserscan"])
    cloud_topic = str(scan.pop("input_topic"))
    scan_topic = str(scan.pop("output_topic"))

    proximity = dict(safety_config["proximity_stop"])
    # Costmap sees obstacles out to 5.5 m; only this close range is a hard stop.
    proximity["stop_distance_m"] = 0.40

    mppi_parameters = {
        "trajectory_file": trajectory_file,
        "route_file": route_file,
        "semantic_map_file": semantic_map_file,
        "planning_params_file": planning_params_file,
        "optimizer_params_file": optimizer_params_file,
        "map_frame": str(estimator["map_frame"]),
        "base_frame": base_frame,
        "odom_topic": "/odom",
        "shelf_scan_topic": "/localization/scan",
        "frequency_hz": tracker["frequency_hz"],
        "horizon_steps": mppi["horizon_steps"],
        "rollout_count": mppi["rollout_count"],
        "iterations": mppi["iterations"],
        "temperature": mppi["temperature"],
        "speed_noise_std_mps": mppi["speed_noise_std_mps"],
        "curvature_noise_std_1pm": mppi["curvature_noise_std_1pm"],
        "progress_search_window_points": mppi["progress_search_window_points"],
        "max_progress_advance_points": mppi["max_progress_advance_points"],
        "lateral_feedback_gain_1pm_per_m": mppi["lateral_feedback_gain_1pm_per_m"],
        "heading_feedback_gain_1pm_per_rad": mppi["heading_feedback_gain_1pm_per_rad"],
        "feedback_blend": mppi["feedback_blend"],
        "max_speed_mps": motion["max_speed_mps"],
        "command_speed_memory_limit_mps": motion["max_speed_mps"],
        "max_acceleration_mps2": motion["max_acceleration_mps2"],
        "max_deceleration_mps2": motion["max_deceleration_mps2"],
        "max_jerk_mps3": motion["max_jerk_mps3"],
        "min_turning_radius_m": motion["min_turning_radius_m"],
        "max_curvature_rate_1pmps": motion["max_curvature_rate_1pmps"],
        "pose_timeout_s": estimator["pose_timeout_s"],
        "velocity_timeout_s": estimator["velocity_timeout_s"],
        "max_pose_prediction_s": estimator["max_pose_prediction_s"],
        "max_position_jump_m": estimator["max_position_jump_m"],
        "max_heading_jump_deg": estimator["max_heading_jump_deg"],
        "initial_pose_settle_s": estimator["initial_pose_settle_s"],
        "body_command_topic": "/control/body_cmd",
        "tracking_error_topic": "/control/tracking_error",
        "state_valid_topic": "/control/state_valid",
        "status_topic": "/control/status",
        "reference_path_topic": "/planning/replan_reference_path",
        "executed_path_topic": "/control/replan_executed_path",
        "executed_path_min_separation_m": visualization[
            "executed_path_min_separation_m"
        ],
        "executed_path_max_points": visualization["executed_path_max_points"],
        "replanning_enabled": True,
        "local_trajectory_topic": "/planning/local_trajectory",
        "local_stop_request_topic": "/planning/local_stop_request",
        "local_trajectory_timeout_s": 2.0,
        "require_route_enable": True,
    }

    replanner_parameters = {
        "trajectory_file": trajectory_file,
        "map_file": map_file,
        "semantic_map_file": semantic_map_file,
        "map_frame": str(estimator["map_frame"]),
        "base_frame": base_frame,
        "frequency_hz": 2.0,
        "obstacle_source": "costmap",
        "costmap_topic": "/avoidance/local_costmap",
        "expected_obstacle_frame": "body",
        "costmap_occupancy_threshold": 50,
        "odom_topic": "/odom",
        "max_obstacle_age_s": 0.50,
        "max_odom_age_s": 0.50,
        "obstacle_x_min_m": 0.05,
        "obstacle_x_max_m": 4.0,
        "obstacle_y_half_width_m": 2.5,
        "lookahead_distance_m": 3.0,
        "inflation_radius_m": 0.04,
        "sample_spacing_m": 0.10,
        "min_turning_radius_m": motion["min_turning_radius_m"],
        "step_length_m": 0.20,
        "curvature_bins": 9,
        "heading_bins": 72,
        "search_heuristic_weight": 1.6,
        "goal_position_tolerance_m": 0.15,
        "goal_heading_tolerance_deg": 8.0,
        "reference_deviation_weight": 2.0,
        "max_expansions": 250000,
        "planning_timeout_s": 2.0,
        "empty_history_planning_timeout_s": 0.75,
        "relaxed_extension_timeout_s": 0.50,
        "reference_search_window_points": 160,
        "local_trajectory_topic": "/planning/local_trajectory",
        "local_stop_request_topic": "/planning/local_stop_request",
        "status_topic": "/planning/local_replan_status",
    }

    safety_parameters = {
        "frequency_hz": tracker["frequency_hz"],
        "command_output_topic": LaunchConfiguration("safety_output_topic"),
        "command_timeout_s": safety["command_timeout_s"],
        "state_timeout_s": safety["state_timeout_s"],
        "system_state_timeout_s": safety["system_state_timeout_s"],
        "max_speed_mps": safety["max_speed_mps"],
        "max_acceleration_mps2": safety["max_acceleration_mps2"],
        "max_deceleration_mps2": safety["max_deceleration_mps2"],
        "min_turning_radius_m": safety["min_turning_radius_m"],
        "recovery_lateral_error_m": safety["recovery_lateral_error_m"],
        "recovery_heading_error_deg": safety["recovery_heading_error_deg"],
        "recovery_clear_lateral_error_m": safety["recovery_clear_lateral_error_m"],
        "recovery_clear_heading_error_deg": safety["recovery_clear_heading_error_deg"],
        "recovery_speed_mps": safety["recovery_speed_mps"],
        "max_lateral_error_m": safety["max_lateral_error_m"],
        "max_heading_error_deg": safety["max_heading_error_deg"],
        "tracking_error_timeout_s": safety["tracking_error_timeout_s"],
        "avoidance_stop_topic": safety["avoidance_stop_topic"],
        "require_avoidance_source": True,
        "avoidance_timeout_s": safety["avoidance_timeout_s"],
        "traffic_rules_stop_topic": safety["traffic_rules_stop_topic"],
        "require_traffic_rules": False,
        "traffic_rules_timeout_s": safety["traffic_rules_timeout_s"],
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory_file",
                default_value=str(
                    WS / "artifacts/indoor_manual_first_motion_0p8m_trajectory.yaml"
                ),
                description="Validated trajectory; defaults to the supervised 0.8 m test.",
            ),
            DeclareLaunchArgument(
                "start_chassis_adapter",
                default_value="false",
                description="False keeps real /cmd_vel disconnected.",
            ),
            DeclareLaunchArgument(
                "safety_output_topic",
                default_value="/cmd_vel_safe",
            ),
            DeclareLaunchArgument(
                "chassis_adapter_input_topic",
                default_value="/cmd_vel_safe",
            ),
            DeclareLaunchArgument(
                "chassis_adapter_output_topic",
                default_value="/cmd_vel",
            ),
            LogInfo(
                msg=(
                    "GUARDED REPLAN: strict Safety is active; chassis adapter is OFF "
                    "unless start_chassis_adapter:=true is explicitly supplied."
                )
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="rebuild_replan_proximity_scan",
                output="screen",
                parameters=[scan],
                remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
            ),
            Node(
                package="competition_safety",
                executable="proximity_stop_node",
                name="rebuild_replan_proximity_stop",
                output="screen",
                parameters=[proximity],
            ),
            Node(
                package="competition_planning",
                executable="local_replanner_node",
                name="rebuild_local_replanner",
                output="screen",
                parameters=[replanner_parameters],
            ),
            Node(
                package="competition_control",
                executable="mppi_control_node",
                name="rebuild_replan_mppi",
                output="screen",
                parameters=[mppi_parameters],
            ),
            Node(
                package="competition_safety",
                executable="safety_node",
                name="rebuild_replan_safety",
                output="screen",
                parameters=[safety_parameters],
            ),
            Node(
                package="competition_control",
                executable="ranger_twist_adapter_node",
                name="rebuild_replan_ranger_twist_adapter",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_chassis_adapter")),
                parameters=[
                    {
                        "input_topic": LaunchConfiguration(
                            "chassis_adapter_input_topic"
                        ),
                        "output_topic": LaunchConfiguration(
                            "chassis_adapter_output_topic"
                        ),
                        "wheelbase_m": motion["wheelbase_m"],
                        "track_width_m": motion["track_width_m"],
                        "driver_min_turn_radius_m": motion[
                            "ranger_driver_min_turn_radius_m"
                        ],
                    }
                ],
            ),
        ]
    )
