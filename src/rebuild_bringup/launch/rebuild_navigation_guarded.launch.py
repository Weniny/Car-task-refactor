#!/usr/bin/env python3
"""Guarded navigation: proximity, MPPI, Safety, optional chassis adapter."""

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
    control = _load_yaml(
        WS / "config/control/control_params_rebuild_010mps.yaml"
    )
    safety_config = _load_yaml(
        WS / "config/safety/safety_params_rebuild_010mps.yaml"
    )

    tracker = control["trajectory_tracker"]
    mppi = tracker["mppi"]
    motion = control["motion"]
    estimator = control["state_estimator"]
    visualization = control["visualization"]
    safety = safety_config["safety"]

    base_frame = str(estimator.get("tracking_base_frame", estimator["base_frame"]))

    scan = dict(safety_config["pointcloud_to_laserscan"])
    cloud_topic = str(scan.pop("input_topic"))
    scan_topic = str(scan.pop("output_topic"))

    proximity = dict(safety_config["proximity_stop"])
    # Same 0.80 m distance that passed the physical dry proximity test.
    proximity["stop_distance_m"] = 0.80

    mppi_parameters = {
        "trajectory_file": LaunchConfiguration("trajectory_file"),
        "route_file": str(
            WS / "routes/indoor_manual_six_anchor_v3_loop.yaml"
        ),
        "semantic_map_file": str(
            WS / "maps/indoor_manual_final/semantic_map_six_anchor_v3.yaml"
        ),
        "planning_params_file": str(
            WS / "config/planning/semantic_lidar_guarded_six_anchor.yaml"
        ),
        "optimizer_params_file": str(
            WS / "config/planning/continuous_trajectory_low_speed.yaml"
        ),
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
        "controller_goal_position_tolerance_m": mppi[
            "controller_goal_position_tolerance_m"
        ],
        "controller_goal_heading_tolerance_deg": mppi[
            "controller_goal_heading_tolerance_deg"
        ],
        "goal_heading_tolerance_deg": mppi[
            "checkpoint_goal_heading_tolerance_deg"
        ],
        "checkpoint_overshoot_tolerance_m": mppi[
            "checkpoint_overshoot_tolerance_m"
        ],
        "checkpoint_slowdown_distance_m": motion["docking_slowdown_distance_m"],
        "checkpoint_min_speed_mps": motion["docking_speed_min_mps"],
        "checkpoint_max_speed_mps": motion["docking_speed_max_mps"],
        "max_speed_mps": motion["max_speed_mps"],
        "max_acceleration_mps2": motion["max_acceleration_mps2"],
        "max_deceleration_mps2": motion["max_deceleration_mps2"],
        "max_jerk_mps3": motion["max_jerk_mps3"],
        "min_turning_radius_m": motion["min_turning_radius_m"],
        "max_curvature_rate_1pmps": motion["max_curvature_rate_1pmps"],
        "command_speed_memory_limit_mps": motion["max_speed_mps"],
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
        "reference_path_topic": "/planning/guarded_reference_path",
        "active_checkpoint_topic": "/mission/guarded_checkpoint_index",
        "executed_path_topic": "/control/guarded_executed_path",
        "executed_path_min_separation_m": visualization[
            "executed_path_min_separation_m"
        ],
        "executed_path_max_points": visualization["executed_path_max_points"],
        "replanning_enabled": False,
        "require_route_enable": True,
        "local_trajectory_timeout_s": 1.0,
    }

    safety_parameters = {
        "frequency_hz": tracker["frequency_hz"],
        "command_output_topic": LaunchConfiguration("chassis_adapter_input_topic"),
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

    adapter_parameters = {
        "input_topic": LaunchConfiguration("chassis_adapter_input_topic"),
        "output_topic": LaunchConfiguration("chassis_adapter_output_topic"),
        "wheelbase_m": motion["wheelbase_m"],
        "track_width_m": motion["track_width_m"],
        "driver_min_turn_radius_m": motion[
            "ranger_driver_min_turn_radius_m"
        ],
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory_file",
                default_value=str(
                    WS / "artifacts/"
                    "indoor_manual_six_anchor_v3_continuous_trajectory.yaml"
                ),
                description=(
                    "Validated continuous trajectory artifact. "
                    "The chassis remains disconnected unless explicitly enabled."
                ),
            ),
            DeclareLaunchArgument(
                "start_chassis_adapter",
                default_value="false",
                description="False keeps real /cmd_vel disconnected.",
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
                    "GUARDED NAVIGATION: proximity, MPPI and Safety are active; "
                    "real chassis adapter defaults to OFF."
                )
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="rebuild_guarded_proximity_scan",
                output="screen",
                parameters=[scan],
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", scan_topic),
                ],
            ),
            Node(
                package="competition_safety",
                executable="proximity_stop_node",
                name="rebuild_guarded_proximity_stop",
                output="screen",
                parameters=[proximity],
            ),
            Node(
                package="competition_control",
                executable="mppi_control_node",
                name="rebuild_guarded_mppi",
                output="screen",
                parameters=[mppi_parameters],
            ),
            Node(
                package="competition_safety",
                executable="safety_node",
                name="rebuild_guarded_safety",
                output="screen",
                parameters=[safety_parameters],
            ),
            Node(
                package="competition_control",
                executable="ranger_twist_adapter_node",
                name="rebuild_guarded_ranger_twist_adapter",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_chassis_adapter")
                ),
                parameters=[adapter_parameters],
            ),
        ]
    )
