"""Pure state machine for supervised execution of stop-bounded route segments."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


_RECOVERABLE_FRESHNESS_FAILURES = frozenset({"stale_pose", "stale_velocity"})


def state_failure_requires_fault_hold(reasons: tuple[str, ...]) -> bool:
    """Return whether an invalid estimate requires stable FAULT_HOLD recovery."""
    return not reasons or not set(reasons).issubset(
        _RECOVERABLE_FRESHNESS_FAILURES
    )


class SegmentedRoutePhase(str, Enum):
    DISARMED = "MISSION_DISARMED"
    TRACKING = "SEGMENT_TRACKING"
    DOCK_HOLD = "DOCK_HOLD"
    WAIT_RELEASE = "WAIT_RELEASE"
    SAFETY_HOLD = "SAFETY_HOLD"
    OVERSHOOT_HOLD = "DOCK_OVERSHOOT_HOLD"
    FAULT_HOLD = "FAULT_HOLD"
    COMPLETED = "ROUTE_COMPLETED"


@dataclass(frozen=True)
class SegmentedRouteConfig:
    goal_position_tolerance_m: float = 0.10
    goal_heading_tolerance_rad: float = math.radians(4.0)
    goal_overshoot_tolerance_m: float = 0.02
    goal_overshoot_activation_distance_m: float = 1.00
    stop_speed_tolerance_mps: float = 0.03
    dock_hold_s: float = 2.0
    fault_recovery_hold_s: float = 0.5

    def __post_init__(self) -> None:
        if self.goal_position_tolerance_m <= 0.0:
            raise ValueError("goal_position_tolerance_m must be positive")
        if self.goal_heading_tolerance_rad < 0.0:
            raise ValueError("goal_heading_tolerance_rad must be non-negative")
        if self.goal_overshoot_tolerance_m < 0.0:
            raise ValueError("goal_overshoot_tolerance_m must be non-negative")
        if (
            self.goal_overshoot_activation_distance_m
            < self.goal_position_tolerance_m
        ):
            raise ValueError(
                "goal_overshoot_activation_distance_m must be at least "
                "goal_position_tolerance_m"
            )
        if self.stop_speed_tolerance_mps < 0.0:
            raise ValueError("stop_speed_tolerance_mps must be non-negative")
        if self.dock_hold_s < 0.0:
            raise ValueError("dock_hold_s must be non-negative")
        if self.fault_recovery_hold_s < 0.0:
            raise ValueError("fault_recovery_hold_s must be non-negative")


@dataclass(frozen=True)
class SegmentedRouteObservation:
    now_s: float
    enabled: bool
    state_valid: bool
    stop_requested: bool
    position_error_m: float
    heading_error_rad: float
    speed_mps: float
    longitudinal_error_m: float = math.nan
    release_segment_index: int | None = None


@dataclass(frozen=True)
class SegmentedRouteDecision:
    phase: SegmentedRoutePhase
    active_segment_index: int
    allow_tracking: bool
    segment_changed: bool = False


class VisualStopConfirmationGate:
    """Confirm that a requested controller hold has reached stable zero speed."""

    def __init__(
        self,
        *,
        linear_speed_tolerance_mps: float = 0.03,
        yaw_rate_tolerance_radps: float = 0.05,
        settle_s: float = 0.15,
    ) -> None:
        values = (
            linear_speed_tolerance_mps,
            yaw_rate_tolerance_radps,
            settle_s,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError(
                "visual stop confirmation values must be finite and non-negative"
            )
        self._linear_speed_tolerance_mps = float(
            linear_speed_tolerance_mps
        )
        self._yaw_rate_tolerance_radps = float(yaw_rate_tolerance_radps)
        self._settle_s = float(settle_s)
        self._stable_since_s: float | None = None

    def update(
        self,
        *,
        now_s: float,
        stop_requested: bool,
        zero_commanded: bool,
        linear_speed_mps: float,
        yaw_rate_radps: float,
    ) -> bool:
        values = (now_s, linear_speed_mps, yaw_rate_radps)
        stable = (
            stop_requested
            and zero_commanded
            and all(math.isfinite(float(value)) for value in values)
            and abs(float(linear_speed_mps))
            <= self._linear_speed_tolerance_mps
            and abs(float(yaw_rate_radps))
            <= self._yaw_rate_tolerance_radps
        )
        if not stable:
            self._stable_since_s = None
            return False
        if self._stable_since_s is None:
            self._stable_since_s = float(now_s)
        return float(now_s) - self._stable_since_s + 1e-9 >= self._settle_s


class SegmentedRouteStateMachine:
    """Hide all segment switching and hold semantics behind one update method."""

    def __init__(
        self,
        segment_count: int,
        config: SegmentedRouteConfig,
    ) -> None:
        if segment_count < 1:
            raise ValueError("segment_count must be positive")
        self._segment_count = segment_count
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._phase = SegmentedRoutePhase.DISARMED
        self._active_segment_index = 0
        self._dock_hold_started_s: float | None = None
        self._fault_recovery_started_s: float | None = None

    def update(
        self,
        observation: SegmentedRouteObservation,
    ) -> SegmentedRouteDecision:
        if not math.isfinite(observation.now_s):
            raise ValueError("now_s must be finite")

        if not observation.enabled:
            if self._phase != SegmentedRoutePhase.COMPLETED:
                self._phase = SegmentedRoutePhase.DISARMED
                self._dock_hold_started_s = None
                self._fault_recovery_started_s = None
            return self._decision(allow_tracking=False)

        if self._phase == SegmentedRoutePhase.COMPLETED:
            return self._decision(allow_tracking=False)
        if self._phase == SegmentedRoutePhase.OVERSHOOT_HOLD:
            return self._decision(allow_tracking=False)
        if self._phase == SegmentedRoutePhase.FAULT_HOLD:
            if not observation.state_valid or observation.stop_requested:
                self._fault_recovery_started_s = None
                return self._decision(allow_tracking=False)
            if self._fault_recovery_started_s is None:
                self._fault_recovery_started_s = observation.now_s
            if (
                observation.now_s - self._fault_recovery_started_s
                < self._config.fault_recovery_hold_s
            ):
                return self._decision(allow_tracking=False)
            self._fault_recovery_started_s = None
            self._phase = SegmentedRoutePhase.TRACKING

        if not observation.state_valid:
            self._phase = SegmentedRoutePhase.FAULT_HOLD
            self._dock_hold_started_s = None
            self._fault_recovery_started_s = None
            return self._decision(allow_tracking=False)

        if self._phase == SegmentedRoutePhase.WAIT_RELEASE:
            if observation.stop_requested:
                return self._decision(allow_tracking=False)
            release_index = observation.release_segment_index
            if release_index is None:
                return self._decision(allow_tracking=False)
            if not (
                self._active_segment_index < release_index < self._segment_count
            ):
                return self._decision(allow_tracking=False)
            self._active_segment_index = release_index
            self._phase = SegmentedRoutePhase.TRACKING
            return self._decision(
                allow_tracking=False,
                segment_changed=True,
            )

        if observation.stop_requested:
            self._phase = SegmentedRoutePhase.SAFETY_HOLD
            self._dock_hold_started_s = None
            return self._decision(allow_tracking=False)

        if self._phase in (
            SegmentedRoutePhase.DISARMED,
            SegmentedRoutePhase.SAFETY_HOLD,
        ):
            self._phase = SegmentedRoutePhase.TRACKING

        release_index = observation.release_segment_index
        if (
            self._phase == SegmentedRoutePhase.TRACKING
            and release_index is not None
            and self._active_segment_index
            < release_index
            < self._segment_count
        ):
            self._active_segment_index = release_index
            self._dock_hold_started_s = None
            return self._decision(
                allow_tracking=True,
                segment_changed=True,
            )

        pose_at_goal = (
            math.isfinite(observation.position_error_m)
            and math.isfinite(observation.heading_error_rad)
            and observation.position_error_m
            <= self._config.goal_position_tolerance_m
            and abs(observation.heading_error_rad)
            <= self._config.goal_heading_tolerance_rad
        )
        stopped = (
            math.isfinite(observation.speed_mps)
            and abs(observation.speed_mps)
            <= self._config.stop_speed_tolerance_mps
        )
        overshot = (
            math.isfinite(observation.position_error_m)
            and observation.position_error_m
            <= self._config.goal_overshoot_activation_distance_m
            and math.isfinite(observation.longitudinal_error_m)
            and observation.longitudinal_error_m
            > self._config.goal_overshoot_tolerance_m
        )

        if self._phase == SegmentedRoutePhase.TRACKING:
            if pose_at_goal:
                self._phase = SegmentedRoutePhase.DOCK_HOLD
                self._dock_hold_started_s = observation.now_s if stopped else None
                return self._decision(allow_tracking=False)
            if overshot:
                self._phase = SegmentedRoutePhase.OVERSHOOT_HOLD
                self._dock_hold_started_s = None
                return self._decision(allow_tracking=False)
            return self._decision(allow_tracking=True)

        if self._phase == SegmentedRoutePhase.DOCK_HOLD:
            if not pose_at_goal:
                if overshot:
                    self._phase = SegmentedRoutePhase.OVERSHOOT_HOLD
                    self._dock_hold_started_s = None
                    return self._decision(allow_tracking=False)
                self._phase = SegmentedRoutePhase.TRACKING
                self._dock_hold_started_s = None
                return self._decision(allow_tracking=True)
            if not stopped:
                self._dock_hold_started_s = None
                return self._decision(allow_tracking=False)
            if self._dock_hold_started_s is None:
                self._dock_hold_started_s = observation.now_s
                return self._decision(allow_tracking=False)
            if (
                observation.now_s - self._dock_hold_started_s
                < self._config.dock_hold_s
            ):
                return self._decision(allow_tracking=False)
            self._dock_hold_started_s = None
            if self._active_segment_index >= self._segment_count - 1:
                self._phase = SegmentedRoutePhase.COMPLETED
                return self._decision(allow_tracking=False)
            self._phase = SegmentedRoutePhase.WAIT_RELEASE
            return self._decision(allow_tracking=False)

        raise RuntimeError(f"unsupported segmented route phase: {self._phase}")

    def _decision(
        self,
        *,
        allow_tracking: bool,
        segment_changed: bool = False,
    ) -> SegmentedRouteDecision:
        return SegmentedRouteDecision(
            phase=self._phase,
            active_segment_index=self._active_segment_index,
            allow_tracking=allow_tracking,
            segment_changed=segment_changed,
        )
