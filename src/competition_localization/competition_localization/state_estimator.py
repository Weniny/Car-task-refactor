"""Authoritative planar state with freshness and continuity guarantees."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Velocity2D:
    linear_x_mps: float
    yaw_rate_radps: float


@dataclass(frozen=True)
class StateObservation:
    pose: Pose2D
    velocity: Velocity2D
    pose_stamp_s: float
    velocity_stamp_s: float


@dataclass(frozen=True)
class StateEstimatorLimits:
    pose_timeout_s: float = 0.20
    velocity_timeout_s: float = 0.20
    future_tolerance_s: float = 0.05
    max_position_jump_m: float = 0.25
    max_heading_jump_rad: float = math.radians(20.0)
    max_linear_speed_mps: float = 2.0
    max_yaw_rate_radps: float = 3.259
    rate_margin_m: float = 0.03
    rate_margin_rad: float = math.radians(3.0)


@dataclass(frozen=True)
class StateEstimate:
    pose: Pose2D
    velocity: Velocity2D
    pose_stamp_s: float
    velocity_stamp_s: float
    valid: bool
    fresh: bool
    continuous: bool
    reasons: tuple[str, ...]


def predict_observation_to_time(
    observation: StateObservation,
    *,
    now_s: float,
    max_prediction_s: float,
) -> StateObservation:
    """Predict a delayed pose to ``now_s`` with current chassis velocity.

    FAST-LIO can publish geometrically valid poses with sensor-time stamps that
    lag wall time. The prediction is deliberately bounded; observations beyond
    ``max_prediction_s`` are left unchanged so the freshness guard still rejects
    them as stale.
    """

    pose_delay_s = now_s - observation.pose_stamp_s
    if max_prediction_s <= 0.0 or pose_delay_s <= 0.0:
        return observation
    if pose_delay_s > max_prediction_s:
        return observation

    predicted_pose = _integrate_unicycle(
        observation.pose,
        observation.velocity,
        pose_delay_s,
    )
    return StateObservation(
        pose=predicted_pose,
        velocity=observation.velocity,
        pose_stamp_s=now_s,
        velocity_stamp_s=observation.velocity_stamp_s,
    )


class StateEstimator:
    """Fuse one global pose and chassis velocity into a guarded state.

    Invalid observations never replace the last accepted state. Recovery after
    a persistent frame reset is explicit via :meth:`reset`, so a TF jump cannot
    silently become the new navigation truth on the next control cycle.
    """

    def __init__(self, limits: StateEstimatorLimits) -> None:
        self._limits = limits
        self._last_accepted: StateEstimate | None = None

    def reset(self) -> None:
        self._last_accepted = None

    def update(self, observation: StateObservation, *, now_s: float) -> StateEstimate:
        reasons: list[str] = []
        values = (
            observation.pose.x,
            observation.pose.y,
            observation.pose.yaw,
            observation.velocity.linear_x_mps,
            observation.velocity.yaw_rate_radps,
            observation.pose_stamp_s,
            observation.velocity_stamp_s,
            now_s,
        )
        if not all(math.isfinite(value) for value in values):
            reasons.append("non_finite_state")

        pose_age = now_s - observation.pose_stamp_s
        velocity_age = now_s - observation.velocity_stamp_s
        if pose_age > self._limits.pose_timeout_s:
            reasons.append("stale_pose")
        if velocity_age > self._limits.velocity_timeout_s:
            reasons.append("stale_velocity")
        if pose_age < -self._limits.future_tolerance_s:
            reasons.append("future_pose_stamp")
        if velocity_age < -self._limits.future_tolerance_s:
            reasons.append("future_velocity_stamp")

        previous = self._last_accepted
        if previous is not None:
            dt = observation.pose_stamp_s - previous.pose_stamp_s
            position_delta = math.hypot(
                observation.pose.x - previous.pose.x,
                observation.pose.y - previous.pose.y,
            )
            heading_delta = abs(_wrap_angle(observation.pose.yaw - previous.pose.yaw))
            if dt < -1e-9:
                reasons.append("pose_time_reversed")
            elif dt <= 1e-9:
                if position_delta > 1e-6 or heading_delta > 1e-6:
                    reasons.append("pose_changed_without_time_advance")
            else:
                position_limit = min(
                    self._limits.max_position_jump_m,
                    self._limits.max_linear_speed_mps * dt + self._limits.rate_margin_m,
                )
                heading_limit = min(
                    self._limits.max_heading_jump_rad,
                    self._limits.max_yaw_rate_radps * dt + self._limits.rate_margin_rad,
                )
                if position_delta > position_limit + 1e-9:
                    reasons.append("position_jump")
                if heading_delta > heading_limit + 1e-9:
                    reasons.append("heading_jump")

        fresh = not any(
            reason.startswith(("stale_", "future_")) for reason in reasons
        )
        continuous = not any(
            reason
            in {
                "pose_time_reversed",
                "pose_changed_without_time_advance",
                "position_jump",
                "heading_jump",
            }
            for reason in reasons
        )
        if reasons:
            if previous is not None:
                return StateEstimate(
                    pose=previous.pose,
                    velocity=previous.velocity,
                    pose_stamp_s=previous.pose_stamp_s,
                    velocity_stamp_s=previous.velocity_stamp_s,
                    valid=False,
                    fresh=fresh,
                    continuous=continuous,
                    reasons=tuple(reasons),
                )
            return StateEstimate(
                pose=observation.pose,
                velocity=observation.velocity,
                pose_stamp_s=observation.pose_stamp_s,
                velocity_stamp_s=observation.velocity_stamp_s,
                valid=False,
                fresh=fresh,
                continuous=continuous,
                reasons=tuple(reasons),
            )

        accepted = StateEstimate(
            pose=observation.pose,
            velocity=observation.velocity,
            pose_stamp_s=observation.pose_stamp_s,
            velocity_stamp_s=observation.velocity_stamp_s,
            valid=True,
            fresh=True,
            continuous=True,
            reasons=(),
        )
        self._last_accepted = accepted
        return accepted


def _integrate_unicycle(
    pose: Pose2D,
    velocity: Velocity2D,
    dt_s: float,
) -> Pose2D:
    if dt_s <= 0.0:
        return pose

    linear_x = velocity.linear_x_mps
    yaw_rate = velocity.yaw_rate_radps
    yaw0 = pose.yaw
    if abs(yaw_rate) < 1e-6:
        return Pose2D(
            x=pose.x + linear_x * dt_s * math.cos(yaw0),
            y=pose.y + linear_x * dt_s * math.sin(yaw0),
            yaw=_wrap_angle(yaw0),
        )

    yaw1 = yaw0 + yaw_rate * dt_s
    radius = linear_x / yaw_rate
    return Pose2D(
        x=pose.x + radius * (math.sin(yaw1) - math.sin(yaw0)),
        y=pose.y - radius * (math.cos(yaw1) - math.cos(yaw0)),
        yaw=_wrap_angle(yaw1),
    )


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
