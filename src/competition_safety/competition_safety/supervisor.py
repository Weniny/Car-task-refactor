"""Independent hard-rule safety exit for chassis velocity commands."""

from __future__ import annotations

from dataclasses import dataclass
import math

from competition_control.mppi_controller import BodyCommand


_TRACKING_COMMAND_STATUSES = frozenset(
    {"TRACKING", "FAST_CHECKPOINT_RECOVERY"}
)


@dataclass(frozen=True)
class SafetyLimits:
    command_timeout_s: float = 0.15
    state_timeout_s: float = 0.15
    max_speed_mps: float = 0.20
    max_acceleration_mps2: float = 0.20
    max_deceleration_mps2: float = 0.30
    min_turning_radius_m: float = 0.81
    recovery_lateral_error_m: float = 0.10
    recovery_heading_error_rad: float = math.radians(10.0)
    recovery_clear_lateral_error_m: float = 0.06
    recovery_clear_heading_error_rad: float = math.radians(5.0)
    recovery_speed_mps: float = 0.06
    max_lateral_error_m: float = 0.40
    max_heading_error_rad: float = math.radians(45.0)
    tracking_error_timeout_s: float = 1.0
    nominal_period_s: float = 0.05
    max_spin_yaw_rate_radps: float = 0.15
    max_parallel_speed_mps: float = 0.06
    motion_mode_transition_speed_mps: float = 0.03
    motion_mode_transition_timeout_s: float = 0.50


@dataclass(frozen=True)
class SafetyContext:
    now_s: float
    command_stamp_s: float
    state_stamp_s: float
    measured_speed_mps: float
    estop_ready: bool
    remote_ready: bool
    state_valid: bool
    avoidance_ready: bool
    avoidance_stop: bool
    chassis_fault: bool
    system_ready: bool = True
    ackermann_mode: bool = True
    motion_mode: str = "dual_ackermann"
    precision_motion_allowed: bool = False
    traffic_rules_ready: bool = True
    traffic_rules_stop: bool = False
    mission_task_stop: bool = False


@dataclass(frozen=True)
class SafeCommand:
    linear_x_mps: float
    yaw_rate_radps: float
    status: str
    reasons: tuple[str, ...]
    linear_y_mps: float = 0.0


class SafetySupervisor:
    """Apply readiness, freshness, tracking, and motion-envelope rules."""

    def __init__(self, limits: SafetyLimits) -> None:
        if limits.min_turning_radius_m <= 0.0:
            raise ValueError("min_turning_radius_m must be positive")
        if not (
            0.0
            <= limits.recovery_clear_lateral_error_m
            < limits.recovery_lateral_error_m
            < limits.max_lateral_error_m
        ):
            raise ValueError("lateral recovery thresholds must be strictly ordered")
        if not (
            0.0
            <= limits.recovery_clear_heading_error_rad
            < limits.recovery_heading_error_rad
            < limits.max_heading_error_rad
        ):
            raise ValueError("heading recovery thresholds must be strictly ordered")
        if limits.recovery_speed_mps <= 0.0:
            raise ValueError("recovery_speed_mps must be positive")
        if limits.tracking_error_timeout_s < 0.0:
            raise ValueError("tracking_error_timeout_s must be non-negative")
        if limits.max_spin_yaw_rate_radps <= 0.0:
            raise ValueError("max_spin_yaw_rate_radps must be positive")
        if limits.max_parallel_speed_mps <= 0.0:
            raise ValueError("max_parallel_speed_mps must be positive")
        if limits.motion_mode_transition_timeout_s <= 0.0:
            raise ValueError("motion_mode_transition_timeout_s must be positive")
        self._limits = limits
        self._last_output_speed_mps: float | None = None
        self._last_output_time_s: float | None = None
        self._tracking_recovery_active = False
        self._hard_tracking_error_since_s: float | None = None
        self._pending_motion_mode: str | None = None
        self._motion_mode_transition_started_s: float | None = None

    def reset(self) -> None:
        self._last_output_speed_mps = None
        self._last_output_time_s = None
        self._tracking_recovery_active = False
        self._hard_tracking_error_since_s = None
        self._pending_motion_mode = None
        self._motion_mode_transition_started_s = None

    def filter_command(
        self,
        command: BodyCommand,
        context: SafetyContext,
    ) -> SafeCommand:
        fault_reasons = self._fault_reasons(command, context)
        fault_reasons.extend(self._motion_mode_faults(command, context))
        if fault_reasons:
            self._record_output(0.0, context.now_s)
            return SafeCommand(0.0, 0.0, "SAFE_HOLD", tuple(fault_reasons))
        if command.status == "GOAL_REACHED":
            previous_speed = (
                self._last_output_speed_mps
                if self._last_output_speed_mps is not None
                else max(0.0, context.measured_speed_mps)
            )
            speed = max(
                0.0,
                previous_speed
                - self._limits.max_deceleration_mps2 * self._period(context.now_s),
            )
            self._record_output(speed, context.now_s)
            if speed > 1e-9:
                return SafeCommand(
                    speed,
                    0.0,
                    "SAFE_LIMITED",
                    ("goal_decelerating",),
                )
            return SafeCommand(0.0, 0.0, "SAFE_STOP", ("goal_reached",))
        if command.status not in _TRACKING_COMMAND_STATUSES:
            self._record_output(0.0, context.now_s)
            return SafeCommand(0.0, 0.0, "SAFE_HOLD", ("controller_not_tracking",))

        tracking_faults = self._update_tracking_policy(command, context.now_s)
        if tracking_faults:
            self._record_output(0.0, context.now_s)
            return SafeCommand(0.0, 0.0, "SAFE_HOLD", tuple(tracking_faults))

        requested_mode = _requested_motion_mode(command)
        reasons: list[str] = []
        if requested_mode != _actual_motion_mode(context):
            reasons.append("motion_mode_transition")
        speed = command.linear_x_mps
        lateral_speed = command.linear_y_mps
        yaw_rate = command.yaw_rate_radps
        if requested_mode == "spin":
            bounded_yaw_rate = min(
                self._limits.max_spin_yaw_rate_radps,
                max(-self._limits.max_spin_yaw_rate_radps, yaw_rate),
            )
            if abs(bounded_yaw_rate - yaw_rate) > 1e-12:
                reasons.append("spin_rate_limited")
            self._record_output(0.0, context.now_s)
            return SafeCommand(
                linear_x_mps=0.0,
                yaw_rate_radps=bounded_yaw_rate,
                status="SAFE_LIMITED" if reasons else "SAFE_ACTIVE",
                reasons=tuple(reasons),
                linear_y_mps=0.0,
            )

        if requested_mode == "parallel":
            requested_speed = math.hypot(speed, lateral_speed)
            bounded_speed = min(
                self._limits.max_parallel_speed_mps,
                self._limits.max_speed_mps,
                requested_speed,
            )
            if bounded_speed + 1e-12 < requested_speed:
                reasons.append("parallel_speed_limited")
            if self._tracking_recovery_active:
                bounded_speed = min(bounded_speed, self._limits.recovery_speed_mps)
                reasons.append("tracking_recovery")
            bounded_speed = self._bounded_translational_speed(
                bounded_speed,
                context,
                reasons,
            )
            scale = bounded_speed / requested_speed if requested_speed > 1e-12 else 0.0
            self._record_output(bounded_speed, context.now_s)
            return SafeCommand(
                linear_x_mps=speed * scale,
                yaw_rate_radps=0.0,
                status=(
                    "SAFE_RECOVERY"
                    if self._tracking_recovery_active
                    else "SAFE_LIMITED" if reasons else "SAFE_ACTIVE"
                ),
                reasons=tuple(reasons),
                linear_y_mps=lateral_speed * scale,
            )

        bounded_speed = min(self._limits.max_speed_mps, max(0.0, speed))
        if abs(bounded_speed - speed) > 1e-12:
            reasons.append("speed_limited")
        if self._tracking_recovery_active:
            bounded_speed = min(bounded_speed, self._limits.recovery_speed_mps)
            reasons.append("tracking_recovery")

        period = self._period(context.now_s)
        previous_speed = (
            self._last_output_speed_mps
            if self._last_output_speed_mps is not None
            else max(0.0, context.measured_speed_mps)
        )
        upper_speed = previous_speed + self._limits.max_acceleration_mps2 * period
        lower_speed = max(
            0.0,
            previous_speed - self._limits.max_deceleration_mps2 * period,
        )
        acceleration_bounded_speed = min(upper_speed, max(lower_speed, bounded_speed))
        if abs(acceleration_bounded_speed - bounded_speed) > 1e-12:
            reasons.append("acceleration_limited")
        bounded_speed = acceleration_bounded_speed

        max_yaw_rate = abs(bounded_speed) / self._limits.min_turning_radius_m
        bounded_yaw_rate = min(max_yaw_rate, max(-max_yaw_rate, yaw_rate))
        if abs(bounded_yaw_rate - yaw_rate) > 1e-12:
            reasons.append("curvature_limited")

        self._record_output(bounded_speed, context.now_s)
        return SafeCommand(
            linear_x_mps=bounded_speed,
            yaw_rate_radps=bounded_yaw_rate,
            status=(
                "SAFE_RECOVERY"
                if self._tracking_recovery_active
                else "SAFE_LIMITED" if reasons else "SAFE_ACTIVE"
            ),
            reasons=tuple(reasons),
            linear_y_mps=0.0,
        )

    def _bounded_translational_speed(
        self,
        speed_mps: float,
        context: SafetyContext,
        reasons: list[str],
    ) -> float:
        period = self._period(context.now_s)
        previous_speed = (
            self._last_output_speed_mps
            if self._last_output_speed_mps is not None
            else max(0.0, context.measured_speed_mps)
        )
        upper_speed = previous_speed + self._limits.max_acceleration_mps2 * period
        lower_speed = max(
            0.0,
            previous_speed - self._limits.max_deceleration_mps2 * period,
        )
        bounded = min(upper_speed, max(lower_speed, speed_mps))
        if abs(bounded - speed_mps) > 1e-12:
            reasons.append("acceleration_limited")
        return bounded

    def _update_tracking_policy(
        self,
        command: BodyCommand,
        now_s: float,
    ) -> list[str]:
        lateral_error = abs(command.lateral_error_m)
        heading_error = abs(command.heading_error_rad)
        hard_lateral = lateral_error > self._limits.max_lateral_error_m
        hard_heading = heading_error > self._limits.max_heading_error_rad
        if hard_lateral or hard_heading:
            if self._hard_tracking_error_since_s is None:
                self._hard_tracking_error_since_s = now_s
        else:
            self._hard_tracking_error_since_s = None

        hard_reasons: list[str] = []
        if (
            self._hard_tracking_error_since_s is not None
            and now_s - self._hard_tracking_error_since_s
            >= self._limits.tracking_error_timeout_s
        ):
            if hard_lateral:
                hard_reasons.append("lateral_error_persistent")
            if hard_heading:
                hard_reasons.append("heading_error_persistent")

        if self._tracking_recovery_active:
            if (
                lateral_error <= self._limits.recovery_clear_lateral_error_m
                and heading_error <= self._limits.recovery_clear_heading_error_rad
            ):
                self._tracking_recovery_active = False
        elif (
            lateral_error > self._limits.recovery_lateral_error_m
            or heading_error > self._limits.recovery_heading_error_rad
        ):
            self._tracking_recovery_active = True
        return hard_reasons

    def _fault_reasons(
        self,
        command: BodyCommand,
        context: SafetyContext,
    ) -> list[str]:
        reasons: list[str] = []
        values = (
            context.now_s,
            context.command_stamp_s,
            context.state_stamp_s,
            context.measured_speed_mps,
            command.linear_x_mps,
            command.linear_y_mps,
            command.yaw_rate_radps,
            command.lateral_error_m,
            command.heading_error_rad,
        )
        if not all(math.isfinite(value) for value in values):
            reasons.append("non_finite_input")
        if not context.estop_ready:
            reasons.append("estop_not_ready")
        if not context.remote_ready:
            reasons.append("remote_not_ready")
        if not context.state_valid:
            reasons.append("invalid_state")
        if not context.avoidance_ready:
            reasons.append("avoidance_stale")
        if context.avoidance_stop:
            reasons.append("avoidance_stop")
        if not context.traffic_rules_ready:
            reasons.append("traffic_rules_stale")
        if context.traffic_rules_stop:
            reasons.append("traffic_rules_stop")
        if context.mission_task_stop:
            reasons.append("mission_task_stop")
        if context.chassis_fault:
            reasons.append("chassis_fault")
        if not context.system_ready:
            reasons.append("system_state_unavailable")
        if (
            _requested_motion_mode(command) != "dual_ackermann"
            and not context.precision_motion_allowed
        ):
            reasons.append("non_ackermann_outside_precision")
        if (
            _requested_motion_mode(command) != _actual_motion_mode(context)
            and abs(context.measured_speed_mps)
            > self._limits.motion_mode_transition_speed_mps
        ):
            reasons.append("unexpected_motion_mode")
        if context.now_s - context.command_stamp_s > self._limits.command_timeout_s:
            reasons.append("stale_command")
        if context.now_s - context.state_stamp_s > self._limits.state_timeout_s:
            reasons.append("stale_state")
        if context.command_stamp_s > context.now_s + 1e-6:
            reasons.append("future_command")
        if context.state_stamp_s > context.now_s + 1e-6:
            reasons.append("future_state")
        if (
            self._last_output_time_s is not None
            and context.now_s < self._last_output_time_s
        ):
            reasons.append("safety_time_reversed")
        return reasons

    def _motion_mode_faults(
        self,
        command: BodyCommand,
        context: SafetyContext,
    ) -> list[str]:
        requested = _requested_motion_mode(command)
        actual = _actual_motion_mode(context)
        if requested == actual:
            self._pending_motion_mode = None
            self._motion_mode_transition_started_s = None
            return []
        if self._pending_motion_mode != requested:
            self._pending_motion_mode = requested
            self._motion_mode_transition_started_s = context.now_s
            return []
        if self._motion_mode_transition_started_s is None:
            self._motion_mode_transition_started_s = context.now_s
            return []
        if (
            context.now_s - self._motion_mode_transition_started_s
            >= self._limits.motion_mode_transition_timeout_s
        ):
            return ["motion_mode_transition_timeout"]
        return []

    def _period(self, now_s: float) -> float:
        if self._last_output_time_s is None:
            return self._limits.nominal_period_s
        return min(
            max(now_s - self._last_output_time_s, 0.0),
            max(self._limits.nominal_period_s * 2.0, self._limits.nominal_period_s),
        )

    def _record_output(self, speed_mps: float, now_s: float) -> None:
        self._last_output_speed_mps = speed_mps
        self._last_output_time_s = now_s


def _requested_motion_mode(command: BodyCommand) -> str:
    if abs(command.linear_y_mps) > 1e-9:
        return "parallel"
    if abs(command.linear_x_mps) <= 1e-9 and abs(command.yaw_rate_radps) > 1e-9:
        return "spin"
    return "dual_ackermann"


def _actual_motion_mode(context: SafetyContext) -> str:
    if context.ackermann_mode:
        return "dual_ackermann"
    return context.motion_mode
