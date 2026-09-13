#!/usr/bin/env python3
"""Mapping bringup for the no-arm car rebuild.

This launch file starts the sensor/mapping stack only. It does not send any
motion command. The Ranger base driver defaults to off and is not needed for
mapping.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _is_true(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    actions = []

    config_path = LaunchConfiguration("config_path").perform(context)
    fast_lio_config = LaunchConfiguration("fast_lio_config").perform(context)
    scan_config = LaunchConfiguration("scan_config").perform(context)
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic").perform(context)
    slam_config = LaunchConfiguration("slam_config").perform(context)
    port_name = LaunchConfiguration("port_name").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    publish_odom_tf = LaunchConfiguration("publish_odom_tf").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context)
    start_anchor = LaunchConfiguration("start_anchor").perform(context)
    anchor_map_frame = LaunchConfiguration("anchor_map_frame").perform(context)
    anchor_odom_frame = LaunchConfiguration("anchor_odom_frame").perform(context)
    anchor_base_frame = LaunchConfiguration("anchor_base_frame").perform(context)

    if _is_true(LaunchConfiguration("start_livox").perform(context)):
        livox_pkg = get_package_share_directory("livox_ros_driver2")
        livox_node = Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[
                {
                    "xfer_format": 1,
                    "multi_topic": 0,
                    "data_src": 0,
                    "publish_freq": float(
                        LaunchConfiguration(
                            "livox_publish_frequency_hz"
                        ).perform(context)
                    ),
                    "output_data_type": 0,
                    "frame_id": "livox_frame",
                    "lvx_file_path": "/home/livox/livox_test.lvx",
                    "user_config_path": os.path.join(
                        livox_pkg, "config", "MID360_config.json"
                    ),
                    "cmdline_input_bd_code": "livox0000000001",
                }
            ],
        )
        livox_actions = [
            SetEnvironmentVariable(
                "LIVOX_ROS_MAX_PACKET_QUEUE",
                LaunchConfiguration("livox_raw_packet_queue_limit").perform(context),
            )
        ]
        if _is_true(
            LaunchConfiguration("force_livox_host_timestamps").perform(context)
        ):
            livox_actions.append(
                SetEnvironmentVariable(
                    "LIVOX_ROS_FORCE_HOST_TIMESTAMP", "1"
                )
            )
        livox_actions.append(livox_node)
        actions.append(GroupAction(livox_actions))

    if _is_true(LaunchConfiguration("start_fast_lio").perform(context)):
        fast_lio_pkg = get_package_share_directory("fast_lio")
        fast_lio_launch = os.path.join(fast_lio_pkg, "launch", "mapping.launch.py")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fast_lio_launch),
                launch_arguments={
                    "config_path": config_path,
                    "config_file": fast_lio_config,
                    "rviz": use_rviz,
                }.items(),
            )
        )

    if _is_true(LaunchConfiguration("start_base").perform(context)):
        ranger_bringup_pkg = get_package_share_directory("ranger_bringup")
        ranger_launch = os.path.join(ranger_bringup_pkg, "launch", "ranger_mini_v3.launch.py")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ranger_launch),
                launch_arguments={
                    "port_name": port_name,
                    "robot_model": robot_model,
                    "update_rate": "50",
                    "publish_odom_tf": publish_odom_tf,
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "odom_topic_name": "odom",
                }.items(),
            )
        )

    if _is_true(LaunchConfiguration("start_scan").perform(context)):
        actions.append(
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[scan_config],
                remappings=[
                    ("cloud_in", scan_cloud_topic),
                    ("scan", "/scan"),
                ],
            )
        )

    if _is_true(start_anchor):
        actions.append(
            Node(
                package="competition_localization",
                executable="fastlio_anchor_node",
                name="fastlio_anchor",
                output="screen",
                parameters=[
                    {
                        "map_frame": anchor_map_frame,
                        "odom_frame": anchor_odom_frame,
                        "base_frame": anchor_base_frame,
                    }
                ],
            )
        )

    if _is_true(LaunchConfiguration("start_slam").perform(context)):
        actions.append(
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_config],
            )
        )

    return actions


def generate_launch_description():
    rebuild_ws_default = os.environ.get("REBUILD_WS", "/home/agilex/competition_rebuild_ws")
    mapping_config_dir = os.path.join(rebuild_ws_default, "config", "mapping")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value=mapping_config_dir),
            DeclareLaunchArgument("fast_lio_config", default_value="fast_lio_mid360_day1.yaml"),
            DeclareLaunchArgument(
                "scan_config",
                default_value=os.path.join(mapping_config_dir, "pointcloud_to_laserscan_day1.yaml"),
            ),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument(
                "slam_config",
                default_value=os.path.join(mapping_config_dir, "slam_toolbox_day1.yaml"),
            ),
            DeclareLaunchArgument("port_name", default_value="can3"),
            DeclareLaunchArgument("robot_model", default_value="ranger_mini_v3"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument(
                "force_livox_host_timestamps", default_value="false"
            ),
            DeclareLaunchArgument(
                "livox_publish_frequency_hz", default_value="10.0"
            ),
            DeclareLaunchArgument(
                "livox_raw_packet_queue_limit", default_value="1"
            ),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="false"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("start_scan", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="false"),
            DeclareLaunchArgument("start_anchor", default_value="false"),
            DeclareLaunchArgument("anchor_map_frame", default_value="map"),
            DeclareLaunchArgument("anchor_odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("anchor_base_frame", default_value="body"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
