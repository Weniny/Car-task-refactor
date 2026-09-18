"""One-command viewer for the manually finalized indoor map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    start_rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rebuild_bringup"), "rviz", "final_map_view.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_yaml",
                default_value=(
                    "/home/agilex/competition_rebuild_ws/"
                    "maps/indoor_manual_final/map.yaml"
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Manage map_server lifecycle inside this launch.",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="map_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
                condition=IfCondition(autostart),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_display_frame",
                arguments=["0", "0", "0", "0", "0", "0", "map", "map_display"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_final_map",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
                output="screen",
            ),
        ]
    )
