#!/usr/bin/env python3
"""Live LiDAR proximity-stop test without any chassis command path."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


WS = Path("/home/agilex/competition_rebuild_ws")


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def generate_launch_description() -> LaunchDescription:
    config = _load_yaml(
        WS / "config/safety/safety_params_rebuild_010mps.yaml"
    )

    scan = dict(config["pointcloud_to_laserscan"])
    cloud_topic = str(scan.pop("input_topic"))
    scan_topic = str(scan.pop("output_topic"))

    proximity = dict(config["proximity_stop"])
    proximity["stop_distance_m"] = 0.80

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "PROXIMITY DRY TEST: no MPPI, no Safety command output, "
                    "no Ranger chassis adapter; stop distance is 0.80 m."
                )
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="rebuild_proximity_scan",
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
                name="rebuild_proximity_stop",
                output="screen",
                parameters=[proximity],
            ),
        ]
    )
