#!/usr/bin/env python3
"""Keep the arm-wrist RealSense online and run traffic-rule perception."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    competition_ws = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
    realsense_share = get_package_share_directory("realsense2_camera")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_share, "launch", "rs_launch.py")
                ),
                launch_arguments={
                    "camera_namespace": "left_wrist_camera",
                    "camera_name": "camera",
                    "usb_port_id": "2-3.3.2",
                    "initial_reset": "true",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_infra": "false",
                    "enable_infra1": "false",
                    "enable_infra2": "false",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "enable_motion": "false",
                    "enable_sync": "true",
                    "align_depth.enable": "true",
                    "spatial_filter.enable": "true",
                    "temporal_filter.enable": "true",
                    "hole_filling_filter.enable": "true",
                    "pointcloud.enable": "false",
                    "publish_tf": "false",
                    "diagnostics_period": "0.0",
                    "rgb_camera.color_profile": "640,480,15",
                    "depth_module.depth_profile": "640,480,15",
                    "log_level": "warn",
                }.items(),
            ),
            Node(
                package="competition_perception",
                executable="wrist_traffic_node",
                name="wrist_traffic_perception",
                output="screen",
                parameters=[
                    os.path.join(
                        competition_ws,
                        "config",
                        "perception",
                        "wrist_traffic_rules.yaml",
                    )
                ],
            ),
            Node(
                package="competition_perception",
                executable="traffic_light_node",
                name="traffic_light_recognition",
                output="screen",
                parameters=[
                    os.path.join(
                        competition_ws,
                        "config",
                        "perception",
                        "wrist_traffic_rules.yaml",
                    )
                ],
            ),
        ]
    )
