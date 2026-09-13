"""Sampling-based MPPI trajectory tracker for the Ranger four-wheel chassis."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class ControlTrajectoryPoint:
    x: float
    y: float
    yaw: float
    s: float
    curvature: float
    v: float
    t: float
    ref_id: str | None = None


@dataclass(frozen=True)
class ControlTrajectory:
    frame_id: str
    route_name: str
    points: tuple[ControlTrajectoryPoint, ...]

    @classmethod
    def from_dict(cls, artifact: dict[str, Any]) -> "ControlTrajectory":
        raw_points = artifact.get("points", [])
        if not isinstance(raw_points, list) or len(raw_points) < 2:
            raise ValueError("continuous trajectory artifact requires at least two points")
        points = tuple(
            ControlTrajectoryPoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point["yaw"]),
                s=float(point["s"]),
                curvature=float(point["curvature"]),
                v=float(point["v"]),
                t=float(point["t"]),
                ref_id=str(point["ref_id"]) if point.get("ref_id") else None,
            )
            for point in raw_points
        )
        return cls(
            frame_id=str(artifact.get("frame_id", "map")),
            route_name=str(artifact.get("route_name", "")),
            points=points,
        )


def control_trajectories_from_dict(
    artifact: dict[str, Any],
) -> tuple[ControlTrajectory, ...]:
    """Load either one continuous artifact or a stop-bounded segment sequence."""

    raw_segments = artifact.get("trajectories")
    if raw_segments is None:
        return (ControlTrajectory.from_dict(artifact),)
    if not isinstance(raw_segments, list) or not raw_segments:
        raise ValueError("segmented trajectory artifact requires trajectories")

    frame_id = str(artifact.get("frame_id", "map"))
    route_name = str(artifact.get("route_name", ""))
    trajectories: list[ControlTrajectory] = []
    for index, raw_segment in enumerate(raw_segments):
        if not isinstance(raw_segment, dict):
            raise ValueError(f"trajectory segment {index} is not a mapping")
        segment = dict(raw_segment)
        segment["frame_id"] = frame_id
        segment["route_name"] = str(
            raw_segment.get("step_id", f"{route_name}:{index}")
        )
        trajectories.append(ControlTrajectory.from_dict(segment))
    return tuple(trajectories)

@dataclass(frozen=True)
class VehicleState:
    x: float
    y: float
    yaw: float
    linear_speed_mps: float


@dataclass(frozen=True)
class BodyCommand:
    linear_x_mps: float
    yaw_rate_radps: float
    curvature_1pm: float
    target_index: int
    lateral_error_m: float
    heading_error_rad: float
    status: str
    linear_y_mps: float = 0.0

    @classmethod
    def hold(
        cls,
        *,
        target_index: int,
        lateral_error_m: float,
        heading_error_rad: float,
        status: str,
    ) -> "BodyCommand":
        return cls(
            linear_x_mps=0.0,
            yaw_rate_radps=0.0,
            curvature_1pm=0.0,
            target_index=target_index,
            lateral_error_m=lateral_error_m,
            heading_error_rad=heading_error_rad,
            status=status,
            linear_y_mps=0.0,
        )


def apply_mission_speed_limit(
    command: BodyCommand,
    speed_limit_mps: float | None,
) -> BodyCommand:
    """Apply a state-machine speed cap without changing path curvature."""

    if speed_limit_mps is None:
        return command
    if not math.isfinite(speed_limit_mps) or speed_limit_mps <= 0.0:
        raise ValueError("mission speed limit must be finite and positive")
    if command.status != "TRACKING" or command.linear_x_mps <= speed_limit_mps:
        return command
    return replace(
        command,
        linear_x_mps=speed_limit_mps,
        yaw_rate_radps=speed_limit_mps * command.curvature_1pm,
    )


def shape_checkpoint_approach_command(
    command: BodyCommand,
    *,
    position_error_m: float,
    longitudinal_error_m: float,
    checkpoint_heading_error_rad: float,
    checkpoint_heading_tolerance_rad: float,
    capture_distance_m: float,
    slowdown_distance_m: float,
    min_speed_mps: float,
    max_speed_mps: float,
    fast_stop: bool = False,
    current_speed_mps: float | None = None,
    max_deceleration_mps2: float | None = None,
    control_period_s: float | None = None,
    fast_stop_recovery_speed_mps: float = 0.12,
) -> BodyCommand:
    """Keep a forward-only checkpoint approach executable and bounded."""

    if capture_distance_m < 0.0:
        raise ValueError("capture_distance_m must be non-negative")
    if checkpoint_heading_tolerance_rad < 0.0:
        raise ValueError("checkpoint heading tolerance must be non-negative")
    if slowdown_distance_m <= capture_distance_m:
        raise ValueError("slowdown_distance_m must exceed capture_distance_m")
    if not 0.0 < min_speed_mps <= max_speed_mps:
        raise ValueError("checkpoint speeds must satisfy 0 < min <= max")
    if (
        command.status != "TRACKING"
        or not math.isfinite(position_error_m)
        or not math.isfinite(longitudinal_error_m)
    ):
        return command
    if fast_stop:
        if (
            current_speed_mps is None
            or max_deceleration_mps2 is None
            or control_period_s is None
            or not math.isfinite(current_speed_mps)
            or not math.isfinite(max_deceleration_mps2)
            or not math.isfinite(control_period_s)
            or max_deceleration_mps2 <= 0.0
            or control_period_s <= 0.0
            or not math.isfinite(fast_stop_recovery_speed_mps)
            or fast_stop_recovery_speed_mps <= 0.0
        ):
            raise ValueError("invalid fast-stop braking parameters")
        if longitudinal_error_m >= 0.0:
            return BodyCommand.hold(
                target_index=command.target_index,
                lateral_error_m=command.lateral_error_m,
                heading_error_rad=command.heading_error_rad,
                status="FAST_CHECKPOINT_BRAKE",
            )
        speed_mps = max(0.0, abs(current_speed_mps))
        braking_distance_m = (
            speed_mps * speed_mps / (2.0 * max_deceleration_mps2)
            + speed_mps * control_period_s
        )
        if -longitudinal_error_m <= braking_distance_m:
            return BodyCommand.hold(
                target_index=command.target_index,
                lateral_error_m=command.lateral_error_m,
                heading_error_rad=command.heading_error_rad,
                status="FAST_CHECKPOINT_BRAKE",
            )
        if command.linear_x_mps <= 0.0 and position_error_m > capture_distance_m:
            return replace(
                command,
                linear_x_mps=fast_stop_recovery_speed_mps,
                yaw_rate_radps=(
                    fast_stop_recovery_speed_mps * command.curvature_1pm
                ),
                status="FAST_CHECKPOINT_RECOVERY",
            )
        return command
    if position_error_m > slowdown_distance_m:
        return command
    if longitudinal_error_m >= 0.0:
        return BodyCommand.hold(
            target_index=command.target_index,
            lateral_error_m=command.lateral_error_m,
            heading_error_rad=command.heading_error_rad,
            status="CHECKPOINT_PLANE_HOLD",
        )

    remaining_distance_m = -longitudinal_error_m
    if remaining_distance_m > slowdown_distance_m:
        return command
    blend = min(
        1.0,
        max(
            0.0,
            (remaining_distance_m - capture_distance_m)
            / (slowdown_distance_m - capture_distance_m),
        ),
    )
    speed_limit_mps = min_speed_mps + blend * (
        max_speed_mps - min_speed_mps
    )
    if (
        math.isfinite(checkpoint_heading_error_rad)
        and abs(checkpoint_heading_error_rad) > checkpoint_heading_tolerance_rad
    ):
        speed_limit_mps = min_speed_mps
    speed_mps = min(
        speed_limit_mps,
        max(min_speed_mps, command.linear_x_mps),
    )
    return replace(
        command,
        linear_x_mps=speed_mps,
        yaw_rate_radps=speed_mps * command.curvature_1pm,
    )


@dataclass(frozen=True)
class MPPIParams:
    control_dt_s: float = 0.05
    horizon_steps: int = 30
    rollout_count: int = 768
    iterations: int = 2
    temperature: float = 0.35
    speed_noise_std_mps: float = 0.05
    curvature_noise_std_1pm: float = 0.25
    max_speed_mps: float = 0.20
    max_acceleration_mps2: float = 0.20
    max_deceleration_mps2: float = 0.30
    max_jerk_mps3: float = 2.00
    min_turning_radius_m: float = 0.81
    max_curvature_rate_1pmps: float = 0.80
    command_speed_memory_limit_mps: float = 0.05
    goal_position_tolerance_m: float = 0.10
    goal_heading_tolerance_rad: float = math.pi
    position_weight: float = 45.0
    heading_weight: float = 12.0
    speed_weight: float = 4.0
    curvature_weight: float = 0.8
    control_smoothness_weight: float = 2.0
    terminal_weight: float = 80.0
    progress_search_window_points: int = 40
    max_progress_advance_points: int = 3
    lateral_feedback_gain_1pm_per_m: float = 1.5
    heading_feedback_gain_1pm_per_rad: float = 1.0
    feedback_blend: float = 0.35


@dataclass(frozen=True)
class _ReferenceHorizon:
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    speed: np.ndarray
    curvature: np.ndarray
    upper_indices: np.ndarray


class MPPIController:
    """Optimize bounded speed/curvature sequences through one small interface."""

    def __init__(
        self,
        trajectory: ControlTrajectory,
        params: MPPIParams,
        *,
        random_seed: int | None = None,
    ) -> None:
        if len(trajectory.points) < 2:
            raise ValueError("MPPI requires at least two trajectory points")
        if params.min_turning_radius_m <= 0.0:
            raise ValueError("min_turning_radius_m must be positive")
        if params.control_dt_s <= 0.0:
            raise ValueError("control_dt_s must be positive")
        if params.command_speed_memory_limit_mps < 0.0:
            raise ValueError("command_speed_memory_limit_mps must be non-negative")
        if params.max_jerk_mps3 <= 0.0:
            raise ValueError("max_jerk_mps3 must be positive")
        if params.goal_heading_tolerance_rad < 0.0:
            raise ValueError("goal_heading_tolerance_rad must be non-negative")
        self._trajectory = trajectory
        self._params = params
        self._rng = np.random.default_rng(random_seed)
        self._xy = np.array([(point.x, point.y) for point in trajectory.points], dtype=float)
        self._yaw = np.array([point.yaw for point in trajectory.points], dtype=float)
        self._s = np.array([point.s for point in trajectory.points], dtype=float)
        self._time = np.array([point.t for point in trajectory.points], dtype=float)
        self._speed = np.array([point.v for point in trajectory.points], dtype=float)
        self._curvature = np.array(
            [point.curvature for point in trajectory.points],
            dtype=float,
        )
        if np.any(np.abs(self._curvature) > 1.0 / params.min_turning_radius_m + 1e-6):
            raise ValueError("trajectory exceeds the runtime turning-radius envelope")
        if np.any(np.diff(self._s) <= 0.0) or np.any(np.diff(self._time) <= 0.0):
            raise ValueError("trajectory distance and time must be strictly increasing")
        self._nominal = np.zeros((params.horizon_steps, 2), dtype=float)
        self._progress_index = 0
        self._last_speed = 0.0
        self._last_acceleration = 0.0
        self._last_curvature = 0.0
        self._has_commanded = False

    def reset(self) -> None:
        self._nominal.fill(0.0)
        self._progress_index = 0
        self._last_speed = 0.0
        self._last_acceleration = 0.0
        self._last_curvature = 0.0
        self._has_commanded = False

    def replace_trajectory(self, trajectory: ControlTrajectory) -> None:
        """Switch to a newly parameterized local trajectory atomically."""

        if len(trajectory.points) < 2:
            raise ValueError("MPPI requires at least two trajectory points")
        xy = np.array([(point.x, point.y) for point in trajectory.points], dtype=float)
        yaw = np.array([point.yaw for point in trajectory.points], dtype=float)
        distance = np.array([point.s for point in trajectory.points], dtype=float)
        timestamps = np.array([point.t for point in trajectory.points], dtype=float)
        speed = np.array([point.v for point in trajectory.points], dtype=float)
        curvature = np.array(
            [point.curvature for point in trajectory.points],
            dtype=float,
        )
        if np.any(
            np.abs(curvature) > 1.0 / self._params.min_turning_radius_m + 1e-6
        ):
            raise ValueError("trajectory exceeds the runtime turning-radius envelope")
        if np.any(np.diff(distance) <= 0.0) or np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("trajectory distance and time must be strictly increasing")

        self._trajectory = trajectory
        self._xy = xy
        self._yaw = yaw
        self._s = distance
        self._time = timestamps
        self._speed = speed
        self._curvature = curvature
        self._nominal.fill(0.0)
        self._progress_index = 0

    def compute_command(self, state: VehicleState) -> BodyCommand:
        nearest = self._nearest_index(state)
        reference = self._reference_horizon(nearest)
        lateral_error, heading_error = self._tracking_errors(state, nearest)
        goal_distance = math.hypot(
            state.x - self._xy[-1, 0],
            state.y - self._xy[-1, 1],
        )
        goal_heading_error = abs(_wrap_angle(state.yaw - self._yaw[-1]))
        if (
            nearest >= len(self._trajectory.points) - 2
            and goal_distance <= self._params.goal_position_tolerance_m
            and goal_heading_error <= self._params.goal_heading_tolerance_rad
        ):
            self._last_speed = 0.0
            self._last_acceleration = 0.0
            self._has_commanded = False
            return BodyCommand.hold(
                target_index=len(self._trajectory.points) - 1,
                lateral_error_m=lateral_error,
                heading_error_rad=heading_error,
                status="GOAL_REACHED",
            )
        self._warm_start(reference, lateral_error, heading_error)
        for _ in range(max(1, self._params.iterations)):
            noise = self._sample_noise()
            candidates = self._nominal[np.newaxis, :, :] + noise
            candidates = self._enforce_controls(candidates, state.linear_speed_mps)
            costs = self._rollout_costs(state, candidates, reference)
            normalized = costs - np.min(costs)
            weights = np.exp(-normalized / max(self._params.temperature, 1e-6))
            weight_sum = float(np.sum(weights))
            if not math.isfinite(weight_sum) or weight_sum <= 1e-12:
                break
            weights /= weight_sum
            effective_noise = candidates - self._nominal[np.newaxis, :, :]
            self._nominal += np.sum(weights[:, np.newaxis, np.newaxis] * effective_noise, axis=0)
            self._nominal = self._enforce_controls(
                self._nominal[np.newaxis, :, :],
                state.linear_speed_mps,
            )[0]

        speed = self._jerk_limited_output_speed(
            float(self._nominal[0, 0]),
            measured_speed_mps=state.linear_speed_mps,
        )
        feedback_curvature = self._feedback_curvature(lateral_error, heading_error)
        blend = self._feedback_blend()
        stabilized_curvature = (
            float(reference.curvature[0]) + feedback_curvature
        )
        curvature = (
            (1.0 - blend) * float(self._nominal[0, 1])
            + blend * stabilized_curvature
        )
        max_curvature = 1.0 / self._params.min_turning_radius_m
        max_curvature_step = (
            self._params.max_curvature_rate_1pmps * self._params.control_dt_s
        )
        curvature = min(
            max_curvature,
            self._last_curvature + max_curvature_step,
            max(
                -max_curvature,
                self._last_curvature - max_curvature_step,
                curvature,
            ),
        )
        self._nominal[0, 1] = curvature
        self._nominal[0, 0] = speed
        yaw_rate = speed * curvature
        self._last_speed = speed
        self._last_curvature = curvature
        self._nominal[:-1] = self._nominal[1:]
        self._nominal[-1] = self._nominal[-2]
        return BodyCommand(
            linear_x_mps=speed,
            yaw_rate_radps=yaw_rate,
            curvature_1pm=curvature,
            target_index=int(reference.upper_indices[0]),
            lateral_error_m=lateral_error,
            heading_error_rad=heading_error,
            status="TRACKING",
        )

    def _jerk_limited_output_speed(
        self,
        target_speed_mps: float,
        *,
        measured_speed_mps: float,
    ) -> float:
        """Shape normal tracking commands with an S-curve acceleration ramp."""

        dt = self._params.control_dt_s
        previous_speed = (
            self._last_speed
            if self._has_commanded
            else max(0.0, measured_speed_mps)
        )
        previous_acceleration = self._last_acceleration if self._has_commanded else 0.0
        desired_acceleration = min(
            self._params.max_acceleration_mps2,
            max(
                -self._params.max_deceleration_mps2,
                (target_speed_mps - previous_speed) / dt,
            ),
        )
        speed_delta = target_speed_mps - previous_speed
        acceleration_release_delta = (
            previous_acceleration * previous_acceleration
            / (2.0 * self._params.max_jerk_mps3)
        )
        if (
            previous_acceleration > 0.0
            and speed_delta <= acceleration_release_delta + 1e-12
        ):
            desired_acceleration = min(desired_acceleration, 0.0)
        elif (
            previous_acceleration < 0.0
            and -speed_delta <= acceleration_release_delta + 1e-12
        ):
            desired_acceleration = max(desired_acceleration, 0.0)
        jerk_step = self._params.max_jerk_mps3 * dt
        acceleration = min(
            self._params.max_acceleration_mps2,
            previous_acceleration + jerk_step,
            max(
                -self._params.max_deceleration_mps2,
                previous_acceleration - jerk_step,
                desired_acceleration,
            ),
        )
        speed = min(
            self._params.max_speed_mps,
            max(0.0, previous_speed + acceleration * dt),
        )
        self._last_acceleration = (speed - previous_speed) / dt
        self._has_commanded = True
        return speed

    def _nearest_index(self, state: VehicleState) -> int:
        start = max(0, self._progress_index - 3)
        end = min(
            len(self._xy),
            self._progress_index + max(1, self._params.progress_search_window_points) + 1,
        )
        offsets = self._xy[start:end] - np.array([state.x, state.y])
        nearest = start + int(np.argmin(np.sum(offsets * offsets, axis=1)))
        nearest = min(
            nearest,
            self._progress_index + max(1, self._params.max_progress_advance_points),
        )
        self._progress_index = max(self._progress_index, nearest)
        return self._progress_index

    def _reference_horizon(self, nearest: int) -> _ReferenceHorizon:
        timestamps = self._time[nearest] + self._params.control_dt_s * (
            np.arange(self._params.horizon_steps) + 1
        )
        upper = np.clip(
            np.searchsorted(self._time, timestamps, side="left"),
            0,
            len(self._time) - 1,
        )
        lower = np.maximum(0, upper - 1)
        lower_time = self._time[lower]
        upper_time = self._time[upper]
        duration = upper_time - lower_time
        alpha = np.divide(
            timestamps - lower_time,
            duration,
            out=np.zeros_like(timestamps),
            where=duration > 1e-12,
        )
        alpha = np.clip(alpha, 0.0, 1.0)
        yaw_delta = _wrap_angle_array(self._yaw[upper] - self._yaw[lower])
        return _ReferenceHorizon(
            x=self._xy[lower, 0] + alpha * (self._xy[upper, 0] - self._xy[lower, 0]),
            y=self._xy[lower, 1] + alpha * (self._xy[upper, 1] - self._xy[lower, 1]),
            yaw=_wrap_angle_array(self._yaw[lower] + alpha * yaw_delta),
            speed=self._speed[lower] + alpha * (self._speed[upper] - self._speed[lower]),
            curvature=(
                self._curvature[lower]
                + alpha * (self._curvature[upper] - self._curvature[lower])
            ),
            upper_indices=upper,
        )

    def _tracking_errors(self, state: VehicleState, index: int) -> tuple[float, float]:
        dx = state.x - self._xy[index, 0]
        dy = state.y - self._xy[index, 1]
        reference_yaw = self._yaw[index]
        lateral = -math.sin(reference_yaw) * dx + math.cos(reference_yaw) * dy
        heading = _wrap_angle(state.yaw - reference_yaw)
        return float(lateral), float(heading)

    def _warm_start(
        self,
        reference: _ReferenceHorizon,
        lateral_error: float,
        heading_error: float,
    ) -> None:
        if not np.any(self._nominal):
            self._nominal[:, 0] = reference.speed
            self._nominal[:, 1] = reference.curvature
        feedback_curvature = self._feedback_curvature(lateral_error, heading_error)
        max_curvature = 1.0 / self._params.min_turning_radius_m
        desired_curvature = np.clip(
            reference.curvature + feedback_curvature,
            -max_curvature,
            max_curvature,
        )
        blend = self._feedback_blend()
        self._nominal[:, 1] = (
            (1.0 - blend) * self._nominal[:, 1] + blend * desired_curvature
        )

    def _feedback_curvature(
        self,
        lateral_error: float,
        heading_error: float,
    ) -> float:
        return -(
            self._params.lateral_feedback_gain_1pm_per_m * lateral_error
            + self._params.heading_feedback_gain_1pm_per_rad * heading_error
        )

    def _feedback_blend(self) -> float:
        return min(1.0, max(0.0, self._params.feedback_blend))

    def _sample_noise(self) -> np.ndarray:
        noise = self._rng.normal(
            size=(self._params.rollout_count, self._params.horizon_steps, 2)
        )
        noise[:, :, 0] *= self._params.speed_noise_std_mps
        noise[:, :, 1] *= self._params.curvature_noise_std_1pm
        noise[0, :, :] = 0.0
        return noise

    def _enforce_controls(self, controls: np.ndarray, current_speed: float) -> np.ndarray:
        bounded = np.array(controls, copy=True)
        max_curvature = 1.0 / self._params.min_turning_radius_m
        bounded[:, :, 0] = np.clip(bounded[:, :, 0], 0.0, self._params.max_speed_mps)
        bounded[:, :, 1] = np.clip(bounded[:, :, 1], -max_curvature, max_curvature)
        remembered_speed = (
            min(
                self._last_speed,
                self._params.command_speed_memory_limit_mps,
            )
            if self._has_commanded
            else 0.0
        )
        previous_speed = np.full(
            bounded.shape[0],
            max(0.0, current_speed, remembered_speed),
        )
        previous_curvature = np.full(bounded.shape[0], self._last_curvature)
        accel_step = self._params.max_acceleration_mps2 * self._params.control_dt_s
        decel_step = self._params.max_deceleration_mps2 * self._params.control_dt_s
        curvature_step = self._params.max_curvature_rate_1pmps * self._params.control_dt_s
        for index in range(self._params.horizon_steps):
            bounded[:, index, 0] = np.minimum(
                previous_speed + accel_step,
                np.maximum(previous_speed - decel_step, bounded[:, index, 0]),
            )
            bounded[:, index, 1] = np.clip(
                bounded[:, index, 1],
                previous_curvature - curvature_step,
                previous_curvature + curvature_step,
            )
            previous_speed = bounded[:, index, 0]
            previous_curvature = bounded[:, index, 1]
        return bounded

    def _rollout_costs(
        self,
        state: VehicleState,
        controls: np.ndarray,
        reference: _ReferenceHorizon,
    ) -> np.ndarray:
        count = controls.shape[0]
        x = np.full(count, state.x)
        y = np.full(count, state.y)
        yaw = np.full(count, state.yaw)
        costs = np.zeros(count)
        previous_controls = np.column_stack(
            [
                np.full(count, max(0.0, state.linear_speed_mps)),
                np.full(count, self._last_curvature),
            ]
        )
        dt = self._params.control_dt_s
        for step in range(self._params.horizon_steps):
            speed = controls[:, step, 0]
            curvature = controls[:, step, 1]
            yaw_mid = yaw + 0.5 * speed * curvature * dt
            x += speed * np.cos(yaw_mid) * dt
            y += speed * np.sin(yaw_mid) * dt
            yaw = _wrap_angle_array(yaw + speed * curvature * dt)

            dx = x - reference.x[step]
            dy = y - reference.y[step]
            heading_error = _wrap_angle_array(yaw - reference.yaw[step])
            speed_error = speed - reference.speed[step]
            curvature_error = curvature - reference.curvature[step]
            delta_controls = controls[:, step, :] - previous_controls
            costs += (
                self._params.position_weight * (dx * dx + dy * dy)
                + self._params.heading_weight * heading_error * heading_error
                + self._params.speed_weight * speed_error * speed_error
                + self._params.curvature_weight * curvature_error * curvature_error
                + self._params.control_smoothness_weight
                * np.sum(delta_controls * delta_controls, axis=1)
            )
            previous_controls = controls[:, step, :]

        terminal_dx = x - reference.x[-1]
        terminal_dy = y - reference.y[-1]
        costs += self._params.terminal_weight * (
            terminal_dx * terminal_dx + terminal_dy * terminal_dy
        )
        return costs


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_angle_array(values: np.ndarray) -> np.ndarray:
    return (values + math.pi) % (2.0 * math.pi) - math.pi
