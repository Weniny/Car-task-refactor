"""Supervised bringup for the unvalidated 0.5 m/s navigation trial."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_file(name: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("rebuild_bringup"), "launch", name])
    )


def generate_launch_description():
    ws = LaunchConfiguration("workspace_root")
    map_yaml = LaunchConfiguration("map_yaml")
    trajectory_file = LaunchConfiguration("trajectory_file")
    optimizer_params_file = LaunchConfiguration("optimizer_params_file")
    rviz = LaunchConfiguration("rviz")
    start_chassis_adapter = LaunchConfiguration("start_chassis_adapter")
    local_replanner_frequency_hz = LaunchConfiguration(
        "local_replanner_frequency_hz"
    )
    debug_rviz_config = PathJoinSubstitution(
        [FindPackageShare("rebuild_bringup"), "rviz", "navigation_debug_view.rviz"]
    )

    scan_params = PathJoinSubstitution(
        [ws, "config", "localization", "pointcloud_to_laserscan_rebuild_localization.yaml"]
    )
    amcl_params = PathJoinSubstitution(
        [ws, "config", "localization", "amcl_rebuild_indoor.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workspace_root",
                default_value="/home/agilex/competition_rebuild_ws",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=(
                    "/home/agilex/competition_rebuild_ws/"
                    "maps/indoor_manual_final/map.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "trajectory_file",
                default_value=(
                    "/home/agilex/competition_rebuild_ws/artifacts/"
                    "indoor_manual_first_segment_30m_radius120_"
                    "turnrate250_050mps_trajectory.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "optimizer_params_file",
                default_value=(
                    "/home/agilex/competition_rebuild_ws/config/planning/"
                    "continuous_trajectory_radius090_turnrate250_050mps.yaml"
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "path_aware_stop_enabled",
                default_value="false",
                description="Experimental path-aware gate; keep false until dry-run checks pass.",
            ),
            DeclareLaunchArgument(
                "fixed_stop_distance_m",
                default_value="1.80",
                description="Forward fixed-stop extent passed to the guarded navigation launch.",
            ),
            DeclareLaunchArgument(
                "local_replanner_frequency_hz",
                default_value="5.0",
                description="Local replanner update frequency in Hz for unified navigation.",
            ),
            DeclareLaunchArgument(
                "obstacle_aware_heuristic",
                default_value="false",
                description="Candidate obstacle-distance search guidance; defaults off.",
            ),
            DeclareLaunchArgument("local_replanner_lookahead_m", default_value="3.0"),
            DeclareLaunchArgument("obstacle_x_max_m", default_value="4.0"),
            DeclareLaunchArgument("planning_timeout_s", default_value="0.4"),
            DeclareLaunchArgument("max_obstacle_age_s", default_value="0.5"),
            DeclareLaunchArgument(
                "start_chassis_adapter",
                default_value="false",
                description="Explicitly connect validated Safety output to /cmd_vel.",
            ),
            LogInfo(
                msg=(
                    "0.5 M/S TRIAL: route starts disabled; the chassis adapter "
                    "is enabled only when explicitly requested."
                )
            ),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        _launch_file("rebuild_mapping.launch.py"),
                        launch_arguments={
                            "start_livox": "true",
                            "force_livox_host_timestamps": "true",
                            "start_fast_lio": "true",
                            "fast_lio_config": "fast_lio_mid360_navigation.yaml",
                            "start_base": "true",
                            "port_name": "can2",
                            "publish_odom_tf": "false",
                            "start_scan": "false",
                            "start_slam": "false",
                            "start_anchor": "false",
                            "rviz": "false",
                        }.items(),
                    ),
                ],
                scoped=True,
            ),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        _launch_file("rebuild_final_map.launch.py"),
                        launch_arguments={
                            "map_yaml": map_yaml,
                            "rviz": "false",
                            "autostart": "false",
                        }.items(),
                    ),
                ],
                scoped=True,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_navigation_debug",
                arguments=["-d", debug_rviz_config],
                condition=IfCondition(rviz),
                output="screen",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="rebuild_localization_scan",
                output="screen",
                parameters=[scan_params],
                remappings=[
                    ("cloud_in", "/cloud_registered_body"),
                    ("scan", "/localization/scan"),
                ],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[amcl_params],
                remappings=[("scan", "/localization/scan")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="localization_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "bond_timeout": 4.0,
                        "node_names": ["amcl"],
                    }
                ],
            ),
            IncludeLaunchDescription(
                _launch_file(
                    "rebuild_navigation_replan_radius120_050mps_guarded.launch.py"
                ),
                launch_arguments={
                    "trajectory_file": trajectory_file,
                    "optimizer_params_file": optimizer_params_file,
                    "start_chassis_adapter": start_chassis_adapter,
                    "local_replanner_frequency_hz": local_replanner_frequency_hz,
                    "path_aware_stop_enabled": LaunchConfiguration(
                        "path_aware_stop_enabled"
                    ),
                    "fixed_stop_distance_m": LaunchConfiguration(
                        "fixed_stop_distance_m"
                    ),
                    "obstacle_aware_heuristic": LaunchConfiguration(
                        "obstacle_aware_heuristic"
                    ),
                    "local_replanner_lookahead_m": LaunchConfiguration(
                        "local_replanner_lookahead_m"
                    ),
                    "obstacle_x_max_m": LaunchConfiguration("obstacle_x_max_m"),
                    "planning_timeout_s": LaunchConfiguration("planning_timeout_s"),
                    "max_obstacle_age_s": LaunchConfiguration("max_obstacle_age_s"),
                }.items(),
            ),
        ]
    )
