"""Stationary competition-style obstacle policy validation for the 0.5 m/s trial.

The fixed proximity box is disabled.  Obstacle clearance is instead decided by
the costmap-backed local Hybrid A* planner; a failed or stale local trajectory
still raises /planning/local_stop_request and holds the MPPI controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ws = "/home/agilex/competition_rebuild_ws"
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        LogInfo(
            msg=(
                "COMPETITION-STYLE STOP CANDIDATE: fixed proximity stop is disabled; "
                "stationary validation only, /cmd_vel adapter remains disabled."
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("rebuild_bringup"), "launch",
                "rebuild_full_navigation_050_trial.launch.py",
            ])),
            launch_arguments={
                "trajectory_file": ws + "/artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_050mps_localcap_trajectory.yaml",
                "optimizer_params_file": ws + "/config/planning/continuous_trajectory_radius090_turnrate250_050mps_localcap.yaml",
                "path_aware_stop_enabled": "false",
                "fixed_stop_distance_m": "0.0",
                "obstacle_aware_heuristic": "true",
                "local_replanner_lookahead_m": "4.0",
                "obstacle_x_max_m": "5.5",
                "planning_timeout_s": "0.6",
                "max_obstacle_age_s": "0.9",
                "local_replanner_frequency_hz": "2.0",
                "start_chassis_adapter": "false",
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
