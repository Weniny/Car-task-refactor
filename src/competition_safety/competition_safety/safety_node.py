#!/usr/bin/env python3
"""ROS adapter that owns the only chassis-bound velocity publisher."""

from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ranger_msgs.msg import MotionState, SystemState
from std_msgs.msg import Bool, Float32, String

from competition_control.mppi_controller import BodyCommand
from competition_safety.shutdown_guard import publish_shutdown_zero_if_ready
from competition_safety.supervisor import SafetyContext, SafetyLimits, SafetySupervisor


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("competition_safety")
        frequency_hz = float(self.declare_parameter("frequency_hz", 20.0).value)
        self._period_s = 1.0 / frequency_hz
        self._system_timeout_s = float(
            self.declare_parameter("system_state_timeout_s", 0.20).value
        )
        self._require_avoidance_source = bool(
            self.declare_parameter("require_avoidance_source", True).value
        )
        self._avoidance_timeout_s = float(
            self.declare_parameter("avoidance_timeout_s", 0.30).value
        )
        self._require_avoidance_speed_limit = bool(
            self.declare_parameter("require_avoidance_speed_limit", False).value
        )
        self._avoidance_speed_limit_seen = False
        self._avoidance_speed_limit_received_s = 0.0
        self._avoidance_speed_limit_mps: float | None = None
        self._require_traffic_rules = bool(
            self.declare_parameter("require_traffic_rules", False).value
        )
        self._traffic_rules_timeout_s = float(
            self.declare_parameter("traffic_rules_timeout_s", 1.0).value
        )
        self._supervisor = SafetySupervisor(
            SafetyLimits(
                command_timeout_s=float(
                    self.declare_parameter("command_timeout_s", 0.15).value
                ),
                state_timeout_s=float(
                    self.declare_parameter("state_timeout_s", 0.15).value
                ),
                max_speed_mps=float(
                    self.declare_parameter("max_speed_mps", 0.20).value
                ),
                max_acceleration_mps2=float(
                    self.declare_parameter("max_acceleration_mps2", 0.20).value
                ),
                max_deceleration_mps2=float(
                    self.declare_parameter("max_deceleration_mps2", 0.30).value
                ),
                min_turning_radius_m=float(
                    self.declare_parameter("min_turning_radius_m", 0.81).value
                ),
                recovery_lateral_error_m=float(
                    self.declare_parameter("recovery_lateral_error_m", 0.10).value
                ),
                recovery_heading_error_rad=math.radians(
                    float(
                        self.declare_parameter(
                            "recovery_heading_error_deg",
                            10.0,
                        ).value
                    )
                ),
                recovery_clear_lateral_error_m=float(
                    self.declare_parameter(
                        "recovery_clear_lateral_error_m",
                        0.06,
                    ).value
                ),
                recovery_clear_heading_error_rad=math.radians(
                    float(
                        self.declare_parameter(
                            "recovery_clear_heading_error_deg",
                            5.0,
                        ).value
                    )
                ),
                recovery_speed_mps=float(
                    self.declare_parameter("recovery_speed_mps", 0.06).value
                ),
                max_lateral_error_m=float(
                    self.declare_parameter("max_lateral_error_m", 0.40).value
                ),
                max_heading_error_rad=math.radians(
                    float(self.declare_parameter("max_heading_error_deg", 45.0).value)
                ),
                tracking_error_timeout_s=float(
                    self.declare_parameter("tracking_error_timeout_s", 1.0).value
                ),
                nominal_period_s=self._period_s,
                max_spin_yaw_rate_radps=float(
                    self.declare_parameter("max_spin_yaw_rate_radps", 0.15).value
                ),
                max_parallel_speed_mps=float(
                    self.declare_parameter("max_parallel_speed_mps", 0.06).value
                ),
                motion_mode_transition_speed_mps=float(
                    self.declare_parameter(
                        "motion_mode_transition_speed_mps",
                        0.03,
                    ).value
                ),
                motion_mode_transition_timeout_s=float(
                    self.declare_parameter(
                        "motion_mode_transition_timeout_s",
                        0.50,
                    ).value
                ),
            )
        )
        self._latest_command: TwistStamped | None = None
        self._latest_error: Vector3Stamped | None = None
        self._controller_status = "NO_COMMAND"
        self._precision_phase = "NORMAL_NAV"
        self._state_valid = False
        self._avoidance_stop = False
        self._avoidance_seen = False
        self._avoidance_received_s = 0.0
        self._traffic_rules_stop = False
        self._traffic_rules_seen = False
        self._traffic_rules_received_s = 0.0
        self._mission_task_stop = False
        self._measured_speed_mps = 0.0
        self._system_state: SystemState | None = None
        self._system_received_s = 0.0
        self._dry_run_allow_standby_mode = bool(
            self.declare_parameter("dry_run_allow_standby_mode", False).value
        )
        self._motion_mode: MotionState | None = None
        self._motion_received_s = 0.0

        self.create_subscription(
            TwistStamped, "/control/body_cmd", self._command_callback, 20
        )
        self.create_subscription(
            Vector3Stamped,
            "/control/tracking_error",
            self._error_callback,
            20,
        )
        self.create_subscription(Bool, "/control/state_valid", self._valid_callback, 20)
        self.create_subscription(String, "/control/status", self._status_callback, 20)
        self.create_subscription(
            Bool,
            str(
                self.declare_parameter(
                    "avoidance_stop_topic",
                    "/avoidance/stop_request",
                ).value
            ),
            self._avoidance_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(
                self.declare_parameter(
                    "traffic_rules_stop_topic",
                    "/perception/traffic_stop_request",
                ).value
            ),
            self._traffic_rules_callback,
            20,
        )
        if self._require_avoidance_speed_limit:
            self.create_subscription(
                Float32,
                str(
                    self.declare_parameter(
                        "avoidance_speed_limit_topic", "/avoidance/speed_limit"
                    ).value
                ),
                self._avoidance_speed_limit_callback,
                20,
            )
        self.create_subscription(
            Bool,
            str(
                self.declare_parameter(
                    "mission_task_stop_topic",
                    "/mission/visual_task_stop_request",
                ).value
            ),
            self._mission_task_stop_callback,
            20,
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(
            SystemState, "/system_state", self._system_callback, 20
        )
        self.create_subscription(
            MotionState, "/motion_state", self._motion_callback, 20
        )
        self._command_publisher = self.create_publisher(
            Twist,
            str(
                self.declare_parameter(
                    "command_output_topic",
                    "/cmd_vel_safe",
                ).value
            ),
            10,
        )
        self._status_publisher = self.create_publisher(String, "/safety/event", 10)
        self._timer = self.create_timer(self._period_s, self._cycle)
        self.get_logger().info(
            "Safety exit ready; output remains zero until TF/odom/system/motion inputs are healthy"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _command_callback(self, message: TwistStamped) -> None:
        self._latest_command = message

    def _error_callback(self, message: Vector3Stamped) -> None:
        self._latest_error = message

    def _valid_callback(self, message: Bool) -> None:
        self._state_valid = bool(message.data)

    def _status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._controller_status = str(payload.get("status", "INVALID_STATUS"))
            self._precision_phase = str(payload.get("precision_phase", "NORMAL_NAV"))
        except (json.JSONDecodeError, AttributeError):
            self._controller_status = "INVALID_STATUS"
            self._precision_phase = "NORMAL_NAV"

    def _avoidance_callback(self, message: Bool) -> None:
        self._avoidance_stop = bool(message.data)
        self._avoidance_seen = True
        self._avoidance_received_s = self._now_s()

    def _avoidance_speed_limit_callback(self, message: Float32) -> None:
        limit = float(message.data)
        self._avoidance_speed_limit_seen = math.isfinite(limit) and limit >= 0.0
        self._avoidance_speed_limit_received_s = self._now_s()
        self._avoidance_speed_limit_mps = limit if limit != 0.0 else None

    def _traffic_rules_callback(self, message: Bool) -> None:
        self._traffic_rules_stop = bool(message.data)
        self._traffic_rules_seen = True
        self._traffic_rules_received_s = self._now_s()

    def _mission_task_stop_callback(self, message: Bool) -> None:
        self._mission_task_stop = bool(message.data)

    def _odom_callback(self, message: Odometry) -> None:
        self._measured_speed_mps = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )

    def _system_callback(self, message: SystemState) -> None:
        self._system_state = message
        self._system_received_s = self._now_s()

    def _motion_callback(self, message: MotionState) -> None:
        self._motion_mode = message
        self._motion_received_s = self._now_s()

    def _cycle(self) -> None:
        now_s = self._now_s()
        command = self._body_command()
        command_stamp_s = (
            _stamp_to_seconds(self._latest_command.header.stamp)
            if self._latest_command is not None
            else 0.0
        )
        state_stamp_s = (
            _stamp_to_seconds(self._latest_error.header.stamp)
            if self._latest_error is not None
            else 0.0
        )
        system_fresh = (
            self._system_state is not None
            and now_s - self._system_received_s <= self._system_timeout_s
        )
        motion_fresh = (
            self._motion_mode is not None
            and now_s - self._motion_received_s <= self._system_timeout_s
        )
        avoidance_ready = bool(
            not self._require_avoidance_source
            or (
                self._avoidance_seen
                and now_s - self._avoidance_received_s <= self._avoidance_timeout_s
            )
        )
        traffic_rules_ready = bool(
            not self._require_traffic_rules
            or (
                self._traffic_rules_seen
                and now_s - self._traffic_rules_received_s
                <= self._traffic_rules_timeout_s
            )
        )
        system_state = self._system_state
        estop_ready = bool(
            system_fresh
            and system_state is not None
            and system_state.vehicle_state != SystemState.VEHICLE_STATE_ESTOP
        )
        can_control_ready = bool(
            system_fresh
            and system_state is not None
            and system_state.control_mode == SystemState.CONTROL_MODE_CAN
        )
        chassis_fault = bool(
            not system_fresh
            or system_state is None
            or system_state.vehicle_state == SystemState.VEHICLE_STATE_EXCEPTION
            or system_state.error_code != 0
        )
        ackermann_mode = bool(
            motion_fresh
            and self._motion_mode is not None
            and self._motion_mode.motion_mode == MotionState.MOTION_MODE_DUAL_ACKERMAN
        )
        motion_mode = "unknown"
        if motion_fresh and self._motion_mode is not None:
            if self._motion_mode.motion_mode == MotionState.MOTION_MODE_DUAL_ACKERMAN:
                motion_mode = "dual_ackermann"
            elif self._motion_mode.motion_mode == MotionState.MOTION_MODE_PARALLEL:
                motion_mode = "parallel"
            elif self._motion_mode.motion_mode == MotionState.MOTION_MODE_SPINNING:
                motion_mode = "spin"
        output = self._supervisor.filter_command(
            command,
            SafetyContext(
                now_s=now_s,
                command_stamp_s=command_stamp_s,
                state_stamp_s=state_stamp_s,
                measured_speed_mps=self._measured_speed_mps,
                estop_ready=estop_ready,
                remote_ready=(
                    can_control_ready or self._dry_run_allow_standby_mode
                ),
                state_valid=self._state_valid,
                avoidance_ready=avoidance_ready,
                avoidance_stop=self._avoidance_stop,
                avoidance_speed_limit_ready=(
                    not self._require_avoidance_speed_limit
                    or (
                        self._avoidance_speed_limit_seen
                        and 0.0 <= now_s - self._avoidance_speed_limit_received_s
                        <= self._avoidance_timeout_s
                    )
                ),
                avoidance_speed_limit_mps=self._avoidance_speed_limit_mps,
                chassis_fault=chassis_fault,
                system_ready=system_fresh and motion_fresh,
                ackermann_mode=ackermann_mode,
                motion_mode=motion_mode,
                precision_motion_allowed=self._precision_phase
                in {
                    "HEADING_TRIM",
                    "POSITION_TRIM",
                },
                traffic_rules_ready=traffic_rules_ready,
                traffic_rules_stop=self._traffic_rules_stop,
                mission_task_stop=self._mission_task_stop,
            ),
        )
        message = Twist()
        message.linear.x = output.linear_x_mps
        message.linear.y = output.linear_y_mps
        message.angular.z = output.yaw_rate_radps
        self._command_publisher.publish(message)
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {"status": output.status, "reasons": list(output.reasons)},
                    separators=(",", ":"),
                )
            )
        )

    def _body_command(self) -> BodyCommand:
        if self._latest_command is None or self._latest_error is None:
            return BodyCommand.hold(
                target_index=0,
                lateral_error_m=0.0,
                heading_error_rad=0.0,
                status="NO_COMMAND",
            )
        speed = float(self._latest_command.twist.linear.x)
        lateral_speed = float(self._latest_command.twist.linear.y)
        yaw_rate = float(self._latest_command.twist.angular.z)
        curvature = yaw_rate / speed if abs(speed) > 1e-9 else 0.0
        return BodyCommand(
            linear_x_mps=speed,
            yaw_rate_radps=yaw_rate,
            curvature_1pm=curvature,
            target_index=int(round(self._latest_error.vector.z)),
            lateral_error_m=float(self._latest_error.vector.x),
            heading_error_rad=float(self._latest_error.vector.y),
            status=self._controller_status,
            linear_y_mps=lateral_speed,
        )


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        publish_shutdown_zero_if_ready(
            context_is_valid=rclpy.ok,
            publish_zero=lambda: node._command_publisher.publish(Twist()),
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
