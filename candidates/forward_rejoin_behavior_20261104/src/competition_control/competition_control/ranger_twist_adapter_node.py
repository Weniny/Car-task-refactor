#!/usr/bin/env python3
"""Adapt safe body Twist commands to Ranger Mini V3 driver Twist semantics."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from competition_control.ranger_twist_adapter import (
    RangerMiniV3Geometry,
    adapt_yaw_rate_for_ranger_driver,
)


def adapt_twist_for_ranger_driver(
    message: Twist, *, geometry: RangerMiniV3Geometry = RangerMiniV3Geometry()
) -> Twist:
    """Copy ``message`` while adapting ``angular.z`` for the Ranger driver."""

    adapted = Twist()
    adapted.linear.x = message.linear.x
    adapted.linear.y = message.linear.y
    adapted.linear.z = message.linear.z
    adapted.angular.x = message.angular.x
    adapted.angular.y = message.angular.y
    adapted.angular.z = adapt_yaw_rate_for_ranger_driver(
        linear_x_mps=message.linear.x,
        desired_yaw_rate_radps=message.angular.z,
        geometry=geometry,
    )
    return adapted


class RangerTwistAdapterNode(Node):
    """ROS adapter from safe body Twist to Ranger Mini driver Twist."""

    def __init__(self) -> None:
        super().__init__("ranger_twist_adapter")
        input_topic = str(self.declare_parameter("input_topic", "/cmd_vel_safe").value)
        output_topic = str(self.declare_parameter("output_topic", "/cmd_vel").value)
        geometry = RangerMiniV3Geometry(
            wheelbase_m=float(self.declare_parameter("wheelbase_m", 0.494).value),
            track_width_m=float(self.declare_parameter("track_width_m", 0.364).value),
            driver_min_turn_radius_m=float(
                self.declare_parameter("driver_min_turn_radius_m", 0.47644).value
            ),
        )
        self._geometry = geometry
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._command_callback, 10)
        self.get_logger().info(
            "Ranger twist adapter ready: "
            f"{input_topic} -> {output_topic}, "
            f"wheelbase={geometry.wheelbase_m:.3f}m, "
            f"track={geometry.track_width_m:.3f}m"
        )

    def _command_callback(self, message: Twist) -> None:
        self._publisher.publish(
            adapt_twist_for_ranger_driver(message, geometry=self._geometry)
        )


def main() -> None:
    rclpy.init()
    node = RangerTwistAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
