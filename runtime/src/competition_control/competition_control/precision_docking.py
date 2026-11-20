"""Field-independent precision docking policy for semantic checkpoints."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
from typing import Any, Sequence

from competition_control.mission_checkpoints import MissionCheckpoint
from competition_control.mppi_controller import (
    ControlTrajectory,
    ControlTrajectoryPoint,
    VehicleState,
)
from competition_control.shelf_alignment import ShelfAlignmentConfig, ShelfObservation


class RequestedMotionMode(str, Enum):
    DUAL_ACKERMANN = "dual_ackermann"
    SPIN = "spin"
    PARALLEL = "parallel"


class PrecisionDockingPhase(str, Enum):
    NORMAL_NAV = "NORMAL_NAV"
    PRECISION_APPROACH = "PRECISION_APPROACH"
    STOP_SETTLE = "STOP_SETTLE"
    HEADING_TRIM = "HEADING_TRIM"
    POSITION_TRIM = "POSITION_TRIM"
    DOCK_READY = "DOCK_READY"
    ALIGNMENT_HOLD = "ALIGNMENT_HOLD"
    SHELF_OBSERVATION_HOLD = "SHELF_OBSERVATION_HOLD"


@dataclass(frozen=True)
class PrecisionDockingConfig:
    activation_distance_m: float
    trim_entry_distance_m: float
    final_position_tolerance_m: float
    heading_realign_tolerance_rad: float
    position_trim_heading_tolerance_rad: float
    heading_trim_target_rad: float
    settle_time_s: float = 0.30
    stable_time_s: float = 0.50
    stopped_speed_tolerance_mps: float = 0.02
    stopped_yaw_rate_tolerance_radps: float = 0.03
    max_spin_correction_rad: float = math.radians(10.0)
    max_parallel_correction_m: float = 0.10
    spin_gain: float = 1.0
    spin_min_yaw_rate_radps: float = 0.06
    spin_max_yaw_rate_radps: float = 0.15
    parallel_gain: float = 0.8
    parallel_min_speed_mps: float = 0.04
    parallel_max_speed_mps: float = 0.06
    shelf_relative_enabled: bool = False
    shelf_side: str = "RIGHT"
    vehicle_half_width_m: float = 0.25
    target_side_clearance_m: float = 0.28
    minimum_side_clearance_m: float = 0.12
    shelf_capture_distance_m: float = 0.20
    shelf_observation_max_age_s: float = 0.25
    shelf_scan_fusion_window_s: float = 0.80
    shelf_scan_fusion_max_frames: int = 10
    shelf_alignment: ShelfAlignmentConfig = ShelfAlignmentConfig()

    def __post_init__(self) -> None:
        if not (
            self.activation_distance_m
            > self.trim_entry_distance_m
            > self.final_position_tolerance_m
            > 0.0
        ):
            raise ValueError(
                "precision distances must satisfy activation > trim entry > final > 0"
            )
        if not (
            0.0
            < self.heading_trim_target_rad
            < self.heading_realign_tolerance_rad
            <= self.position_trim_heading_tolerance_rad
            <= self.max_spin_correction_rad
        ):
            raise ValueError(
                "heading thresholds must satisfy target < realign "
                "<= position trim <= max correction"
            )
        if self.settle_time_s < 0.0 or self.stable_time_s < 0.0:
            raise ValueError("precision settle times must be non-negative")
        if self.max_parallel_correction_m < self.trim_entry_distance_m:
            raise ValueError(
                "precision max parallel correction must cover trim entry distance"
            )
        if not (0.0 < self.spin_min_yaw_rate_radps <= self.spin_max_yaw_rate_radps):
            raise ValueError("spin rates must satisfy 0 < min <= max")
        if not (0.0 < self.parallel_min_speed_mps <= self.parallel_max_speed_mps):
            raise ValueError("parallel speeds must satisfy 0 < min <= max")
        if self.shelf_relative_enabled:
            if self.shelf_side not in {"LEFT", "RIGHT"}:
                raise ValueError("precision shelf side must be LEFT or RIGHT")
            if self.shelf_alignment.side != self.shelf_side:
                raise ValueError("precision shelf side must match scan fit side")
            if not (
                self.vehicle_half_width_m > 0.0
                and self.target_side_clearance_m > self.minimum_side_clearance_m > 0.0
            ):
                raise ValueError(
                    "precision shelf clearances must satisfy target > minimum > 0"
                )
            if not (
                self.activation_distance_m
                >= self.shelf_capture_distance_m
                >= self.trim_entry_distance_m
            ):
                raise ValueError(
                    "precision shelf capture must be between trim entry and activation"
                )
            if (
                self.shelf_observation_max_age_s <= 0.0
                or self.shelf_scan_fusion_window_s <= 0.0
                or self.shelf_scan_fusion_max_frames < 1
            ):
                raise ValueError(
                    "precision shelf observation and fusion limits must be positive"
                )
            target_sensor_distance = (
                self.vehicle_half_width_m + self.target_side_clearance_m
            )
            if not (
                self.shelf_alignment.min_side_distance_m
                <= target_sensor_distance
                <= self.shelf_alignment.max_side_distance_m
            ):
                raise ValueError(
                    "precision shelf scan window must contain the target distance"
                )


@dataclass(frozen=True)
class PrecisionDockingDecision:
    phase: PrecisionDockingPhase
    motion_mode: RequestedMotionMode
    use_dynamic_replanning: bool
    use_fixed_reference: bool
    linear_x_mps: float = 0.0
    linear_y_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    pose_ready: bool = False
    position_error_m: float = math.inf
    heading_error_rad: float = math.inf
    shelf_relative_active: bool = False
    shelf_clearance_m: float | None = None


def precision_docking_configs_from_dict(
    route: dict[str, Any],
    dock_params: dict[str, Any],
) -> dict[str, PrecisionDockingConfig]:
    """Resolve semantic checkpoint assignments without embedding field geometry."""

    route_policy = route.get("precision_docking", {})
    if route_policy is None:
        return {}
    if not isinstance(route_policy, dict):
        raise ValueError("precision_docking route configuration must be a mapping")
    assignments = route_policy.get("checkpoints", {})
    profiles = dock_params.get("precision_profiles", {})
    if not isinstance(assignments, dict) or not isinstance(profiles, dict):
        raise ValueError(
            "precision checkpoint assignments and profiles must be mappings"
        )

    resolved: dict[str, PrecisionDockingConfig] = {}
    for raw_ref, raw_profile_name in assignments.items():
        ref = str(raw_ref).strip()
        profile_name = str(raw_profile_name).strip()
        if not ref or not profile_name:
            raise ValueError(
                "precision checkpoint refs and profile names must be non-empty"
            )
        raw = profiles.get(profile_name)
        if not isinstance(raw, dict):
            raise ValueError(
                f"unknown precision profile {profile_name!r} for checkpoint {ref!r}"
            )
        try:
            shelf = raw.get("shelf_relative", {})
            if shelf is None:
                shelf = {}
            if not isinstance(shelf, dict):
                raise ValueError("shelf_relative must be a mapping")
            scan_fit = shelf.get("scan_fit", {})
            if scan_fit is None:
                scan_fit = {}
            if not isinstance(scan_fit, dict):
                raise ValueError("shelf_relative.scan_fit must be a mapping")
            shelf_side = str(shelf.get("side", "RIGHT")).strip().upper()
            resolved[ref] = PrecisionDockingConfig(
                activation_distance_m=float(raw["activation_distance_m"]),
                trim_entry_distance_m=float(raw["trim_entry_distance_m"]),
                final_position_tolerance_m=float(raw["final_position_tolerance_m"]),
                heading_realign_tolerance_rad=math.radians(
                    float(raw["heading_realign_tolerance_deg"])
                ),
                position_trim_heading_tolerance_rad=math.radians(
                    float(
                        raw.get(
                            "position_trim_heading_tolerance_deg",
                            raw["heading_realign_tolerance_deg"],
                        )
                    )
                ),
                heading_trim_target_rad=math.radians(
                    float(raw["heading_trim_target_deg"])
                ),
                settle_time_s=float(raw.get("settle_time_s", 0.30)),
                stable_time_s=float(raw.get("stable_time_s", 0.50)),
                stopped_speed_tolerance_mps=float(
                    raw.get("stopped_speed_tolerance_mps", 0.02)
                ),
                stopped_yaw_rate_tolerance_radps=float(
                    raw.get("stopped_yaw_rate_tolerance_radps", 0.03)
                ),
                max_spin_correction_rad=math.radians(
                    float(raw.get("max_spin_correction_deg", 10.0))
                ),
                max_parallel_correction_m=float(
                    raw.get(
                        "max_parallel_correction_m",
                        raw["trim_entry_distance_m"],
                    )
                ),
                spin_gain=float(raw.get("spin_gain", 1.0)),
                spin_min_yaw_rate_radps=float(raw.get("spin_min_yaw_rate_radps", 0.06)),
                spin_max_yaw_rate_radps=float(raw.get("spin_max_yaw_rate_radps", 0.15)),
                parallel_gain=float(raw.get("parallel_gain", 0.8)),
                parallel_min_speed_mps=float(raw.get("parallel_min_speed_mps", 0.04)),
                parallel_max_speed_mps=float(raw.get("parallel_max_speed_mps", 0.06)),
                shelf_relative_enabled=_as_bool(shelf.get("enabled", False)),
                shelf_side=shelf_side,
                vehicle_half_width_m=float(shelf.get("vehicle_half_width_m", 0.25)),
                target_side_clearance_m=float(
                    shelf.get("target_side_clearance_m", 0.28)
                ),
                minimum_side_clearance_m=float(
                    shelf.get("minimum_side_clearance_m", 0.12)
                ),
                shelf_capture_distance_m=float(
                    shelf.get("capture_distance_m", 0.20)
                ),
                shelf_observation_max_age_s=float(
                    shelf.get("observation_max_age_s", 0.25)
                ),
                shelf_scan_fusion_window_s=float(
                    shelf.get("scan_fusion_window_s", 0.80)
                ),
                shelf_scan_fusion_max_frames=int(
                    shelf.get("scan_fusion_max_frames", 10)
                ),
                shelf_alignment=ShelfAlignmentConfig(
                    side=shelf_side,
                    min_range_m=float(scan_fit.get("min_range_m", 0.10)),
                    max_range_m=float(scan_fit.get("max_range_m", 1.50)),
                    min_longitudinal_m=float(
                        scan_fit.get("min_longitudinal_m", -0.60)
                    ),
                    max_longitudinal_m=float(
                        scan_fit.get("max_longitudinal_m", 0.80)
                    ),
                    min_side_distance_m=float(
                        scan_fit.get("min_side_distance_m", 0.20)
                    ),
                    max_side_distance_m=float(
                        scan_fit.get("max_side_distance_m", 1.00)
                    ),
                    min_points=int(scan_fit.get("min_points", 12)),
                    min_span_m=float(scan_fit.get("min_span_m", 0.30)),
                    max_residual_m=float(scan_fit.get("max_residual_m", 0.025)),
                    max_heading_error_rad=math.radians(
                        float(scan_fit.get("max_heading_error_deg", 15.0))
                    ),
                    max_candidate_points=int(
                        scan_fit.get("max_candidate_points", 40)
                    ),
                ),
            )
        except (KeyError, TypeError, ValueError) as exc:
            if isinstance(exc, ValueError) and str(exc).startswith("precision"):
                raise
            raise ValueError(
                f"invalid precision profile {profile_name!r} for checkpoint {ref!r}"
            ) from exc
    return resolved


def straight_followup_anchors_from_config(
    dock_params: dict[str, Any],
    checkpoint_refs: Sequence[str],
) -> dict[str, str]:
    """Resolve checkpoints that follow the preceding calibrated stop in a line."""

    raw_followups = dock_params.get("straight_followups", {})
    if raw_followups is None:
        return {}
    if not isinstance(raw_followups, dict):
        raise ValueError("precision straight_followups must be a mapping")

    ordered_refs = tuple(str(ref) for ref in checkpoint_refs)
    ref_indices = {ref: index for index, ref in enumerate(ordered_refs)}
    resolved: dict[str, str] = {}
    for raw_followup_ref, raw_anchor_ref in raw_followups.items():
        followup_ref = str(raw_followup_ref).strip()
        anchor_ref = str(raw_anchor_ref).strip()
        if not followup_ref or not anchor_ref:
            raise ValueError("precision straight followup refs must be non-empty")
        if followup_ref not in ref_indices or anchor_ref not in ref_indices:
            raise ValueError(
                "precision straight followup refs must be mission checkpoints"
            )
        if ref_indices[followup_ref] != ref_indices[anchor_ref] + 1:
            raise ValueError(
                "precision straight followup must immediately follow its anchor"
            )
        resolved[followup_ref] = anchor_ref
    return resolved


def straight_followup_extension_from_config(dock_params: dict[str, Any]) -> float:
    """Return the configured extra travel after each semantic rear checkpoint."""

    extension_m = float(dock_params.get("straight_followup_extension_m", 0.0))
    if not math.isfinite(extension_m) or extension_m < 0.0:
        raise ValueError("precision straight followup extension must be non-negative")
    return extension_m


def calibrated_straight_reference(
    anchor_state: VehicleState,
    anchor_checkpoint: MissionCheckpoint,
    followup_checkpoint: MissionCheckpoint,
    *,
    speed_mps: float,
    extension_m: float = 0.0,
    point_spacing_m: float = 0.05,
    max_lateral_offset_m: float = 0.05,
    max_heading_delta_rad: float = math.radians(5.0),
) -> tuple[MissionCheckpoint, ControlTrajectory]:
    """Build a straight followup from the actual calibrated vehicle pose."""

    if speed_mps <= 0.0 or point_spacing_m <= 0.0:
        raise ValueError("calibrated straight speed and spacing must be positive")
    if not math.isfinite(extension_m) or extension_m < 0.0:
        raise ValueError("calibrated straight extension must be non-negative")
    dx = followup_checkpoint.x - anchor_checkpoint.x
    dy = followup_checkpoint.y - anchor_checkpoint.y
    cos_yaw = math.cos(anchor_checkpoint.yaw)
    sin_yaw = math.sin(anchor_checkpoint.yaw)
    distance_m = cos_yaw * dx + sin_yaw * dy
    lateral_offset_m = -sin_yaw * dx + cos_yaw * dy
    heading_delta_rad = _wrap_angle(
        followup_checkpoint.yaw - anchor_checkpoint.yaw
    )
    if (
        distance_m <= 0.0
        or abs(lateral_offset_m) > max_lateral_offset_m
        or abs(heading_delta_rad) > max_heading_delta_rad
    ):
        raise ValueError(
            f"{anchor_checkpoint.ref_id}->{followup_checkpoint.ref_id} "
            "is not a straight forward pair"
        )
    distance_m += extension_m

    segment_count = max(1, math.ceil(distance_m / point_spacing_m))
    actual_cos_yaw = math.cos(anchor_state.yaw)
    actual_sin_yaw = math.sin(anchor_state.yaw)
    points = tuple(
        ControlTrajectoryPoint(
            x=anchor_state.x + distance_m * index / segment_count * actual_cos_yaw,
            y=anchor_state.y + distance_m * index / segment_count * actual_sin_yaw,
            yaw=anchor_state.yaw,
            s=distance_m * index / segment_count,
            curvature=0.0,
            v=0.0 if index == segment_count else speed_mps,
            t=distance_m * index / segment_count / speed_mps,
            ref_id=(followup_checkpoint.ref_id if index == segment_count else None),
        )
        for index in range(segment_count + 1)
    )
    target = MissionCheckpoint(
        ref_id=followup_checkpoint.ref_id,
        x=points[-1].x,
        y=points[-1].y,
        yaw=anchor_state.yaw,
    )
    return target, ControlTrajectory(
        frame_id="map",
        route_name=(
            f"calibrated_straight_{anchor_checkpoint.ref_id}_to_"
            f"{followup_checkpoint.ref_id}"
        ),
        points=points,
    )


def fixed_reference_to_checkpoint(
    trajectory: ControlTrajectory,
    state: VehicleState,
    checkpoint_ref: str,
    *,
    backtrack_points: int = 2,
) -> ControlTrajectory:
    """Slice the frozen global route from the current pose to one semantic stop."""

    if backtrack_points < 0:
        raise ValueError("backtrack_points must be non-negative")
    end_index = next(
        (
            index
            for index, point in enumerate(trajectory.points)
            if point.ref_id == checkpoint_ref
        ),
        None,
    )
    if end_index is None:
        raise ValueError(
            f"checkpoint ref not found in global trajectory: {checkpoint_ref}"
        )
    if end_index < 1:
        raise ValueError("precision checkpoint must not be the first trajectory point")
    nearest_index = min(
        range(end_index + 1),
        key=lambda index: (
            (trajectory.points[index].x - state.x) ** 2
            + (trajectory.points[index].y - state.y) ** 2
        ),
    )
    start_index = min(
        end_index - 1,
        max(0, nearest_index - backtrack_points),
    )
    selected = trajectory.points[start_index : end_index + 1]
    base_s = selected[0].s
    base_t = selected[0].t
    points = tuple(
        replace(point, s=point.s - base_s, t=point.t - base_t) for point in selected
    )
    return ControlTrajectory(
        frame_id=trajectory.frame_id,
        route_name=f"precision_to_{checkpoint_ref}",
        points=points,
    )


class PrecisionDockingController:
    """Select fixed-path approach, spin trim, and two-axis parallel trim."""

    def __init__(self, config: PrecisionDockingConfig) -> None:
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._phase = PrecisionDockingPhase.NORMAL_NAV
        self._settle_started_s: float | None = None
        self._stable_started_s: float | None = None
        self._shelf_relative_latched = False

    def update(
        self,
        *,
        now_s: float,
        state: VehicleState,
        checkpoint: MissionCheckpoint,
        yaw_rate_radps: float,
        shelf_observation: ShelfObservation | None = None,
    ) -> PrecisionDockingDecision:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        dx = checkpoint.x - state.x
        dy = checkpoint.y - state.y
        map_position_error = math.hypot(dx, dy)
        heading_error = _wrap_angle(checkpoint.yaw - state.yaw)
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        body_x_error = cos_yaw * dx + sin_yaw * dy
        body_y_error = -sin_yaw * dx + cos_yaw * dy
        shelf_clearance_m: float | None = None
        shelf_relative_active = False

        if map_position_error > self._config.activation_distance_m:
            self.reset()
            return self._decision(
                PrecisionDockingPhase.NORMAL_NAV,
                map_position_error,
                heading_error,
            )

        if self._config.shelf_relative_enabled:
            should_capture = (
                self._shelf_relative_latched
                or abs(body_x_error) <= self._config.shelf_capture_distance_m
            )
            if should_capture:
                self._shelf_relative_latched = True
                if shelf_observation is None:
                    self._phase = PrecisionDockingPhase.SHELF_OBSERVATION_HOLD
                    self._settle_started_s = None
                    self._stable_started_s = None
                    return self._decision(
                        self._phase,
                        map_position_error,
                        heading_error,
                        use_fixed_reference=True,
                        shelf_relative_active=True,
                    )
                shelf_relative_active = True
                shelf_clearance_m = (
                    shelf_observation.side_distance_m
                    - self._config.vehicle_half_width_m
                )
                clearance_error = (
                    shelf_clearance_m - self._config.target_side_clearance_m
                )
                side_sign = 1.0 if self._config.shelf_side == "LEFT" else -1.0
                body_y_error = side_sign * clearance_error
                position_error = math.hypot(body_x_error, clearance_error)
                heading_error = shelf_observation.heading_error_rad
            else:
                position_error = map_position_error
        else:
            position_error = map_position_error

        if (
            not shelf_relative_active
            and position_error > self._config.trim_entry_distance_m
        ):
            self._phase = PrecisionDockingPhase.PRECISION_APPROACH
            self._settle_started_s = None
            self._stable_started_s = None
            return self._decision(
                self._phase,
                position_error,
                heading_error,
                use_fixed_reference=True,
            )

        stopped = (
            abs(state.linear_speed_mps) <= self._config.stopped_speed_tolerance_mps
            and abs(yaw_rate_radps) <= self._config.stopped_yaw_rate_tolerance_radps
        )
        if self._phase in (
            PrecisionDockingPhase.NORMAL_NAV,
            PrecisionDockingPhase.PRECISION_APPROACH,
        ):
            return self._settle(
                now_s,
                stopped,
                position_error,
                heading_error,
                shelf_relative_active=shelf_relative_active,
                shelf_clearance_m=shelf_clearance_m,
            )
        if self._phase == PrecisionDockingPhase.STOP_SETTLE:
            if not stopped:
                self._settle_started_s = None
                return self._decision(
                    self._phase,
                    position_error,
                    heading_error,
                    use_fixed_reference=True,
                    shelf_relative_active=shelf_relative_active,
                    shelf_clearance_m=shelf_clearance_m,
                )
            if self._settle_started_s is None:
                self._settle_started_s = now_s
            if now_s - self._settle_started_s < self._config.settle_time_s:
                return self._decision(
                    self._phase,
                    position_error,
                    heading_error,
                    use_fixed_reference=True,
                    shelf_relative_active=shelf_relative_active,
                    shelf_clearance_m=shelf_clearance_m,
                )
            return self._select_trim(
                now_s,
                state,
                checkpoint,
                position_error,
                heading_error,
                body_x_error=body_x_error,
                body_y_error=body_y_error,
                shelf_relative_active=shelf_relative_active,
                shelf_clearance_m=shelf_clearance_m,
                shelf_observation=shelf_observation,
            )
        if self._phase == PrecisionDockingPhase.HEADING_TRIM:
            if (
                shelf_relative_active
                and shelf_clearance_m is not None
                and shelf_clearance_m < self._config.minimum_side_clearance_m
            ):
                return self._settle(
                    now_s,
                    stopped,
                    position_error,
                    heading_error,
                    shelf_relative_active=True,
                    shelf_clearance_m=shelf_clearance_m,
                )
            if abs(heading_error) <= self._config.heading_trim_target_rad:
                return self._settle(
                    now_s,
                    stopped,
                    position_error,
                    heading_error,
                    shelf_relative_active=shelf_relative_active,
                    shelf_clearance_m=shelf_clearance_m,
                )
            if abs(heading_error) > self._config.max_spin_correction_rad:
                return self._alignment_hold(position_error, heading_error)
            yaw_rate = _bounded_signed(
                self._config.spin_gain * heading_error,
                self._config.spin_min_yaw_rate_radps,
                self._config.spin_max_yaw_rate_radps,
            )
            return self._decision(
                self._phase,
                position_error,
                heading_error,
                mode=RequestedMotionMode.SPIN,
                yaw_rate_radps=yaw_rate,
                use_fixed_reference=True,
                shelf_relative_active=shelf_relative_active,
                shelf_clearance_m=shelf_clearance_m,
            )
        if self._phase == PrecisionDockingPhase.POSITION_TRIM:
            if (
                abs(heading_error)
                > self._config.position_trim_heading_tolerance_rad
            ):
                return self._settle(
                    now_s,
                    stopped,
                    position_error,
                    heading_error,
                    shelf_relative_active=shelf_relative_active,
                    shelf_clearance_m=shelf_clearance_m,
                )
            if position_error <= self._config.final_position_tolerance_m:
                return self._settle(
                    now_s,
                    stopped,
                    position_error,
                    heading_error,
                    shelf_relative_active=shelf_relative_active,
                    shelf_clearance_m=shelf_clearance_m,
                )
            if (
                shelf_relative_active
                and shelf_clearance_m is not None
                and shelf_clearance_m < self._config.minimum_side_clearance_m
            ):
                body_x_error = 0.0
            elif position_error > self._config.max_parallel_correction_m:
                return self._alignment_hold(position_error, heading_error)
            return self._parallel_command(
                state,
                checkpoint,
                position_error,
                heading_error,
                body_x_error=body_x_error,
                body_y_error=body_y_error,
                shelf_relative_active=shelf_relative_active,
                shelf_clearance_m=shelf_clearance_m,
            )
        if self._phase == PrecisionDockingPhase.DOCK_READY:
            if (
                position_error > self._config.final_position_tolerance_m
                or abs(heading_error) > self._config.heading_realign_tolerance_rad
                or not stopped
            ):
                self._stable_started_s = None
                return self._settle(
                    now_s,
                    stopped,
                    position_error,
                    heading_error,
                    shelf_relative_active=shelf_relative_active,
                    shelf_clearance_m=shelf_clearance_m,
                )
            if self._stable_started_s is None:
                self._stable_started_s = now_s
            return self._decision(
                self._phase,
                position_error,
                heading_error,
                use_fixed_reference=True,
                pose_ready=(
                    now_s - self._stable_started_s >= self._config.stable_time_s
                ),
                shelf_relative_active=shelf_relative_active,
                shelf_clearance_m=shelf_clearance_m,
            )
        if self._phase == PrecisionDockingPhase.ALIGNMENT_HOLD:
            if (
                abs(heading_error) <= self._config.max_spin_correction_rad
                and position_error <= self._config.max_parallel_correction_m
            ):
                return self._settle(
                    now_s,
                    stopped,
                    position_error,
                    heading_error,
                    shelf_relative_active=shelf_relative_active,
                    shelf_clearance_m=shelf_clearance_m,
                )
            return self._alignment_hold(position_error, heading_error)
        if self._phase == PrecisionDockingPhase.SHELF_OBSERVATION_HOLD:
            return self._settle(
                now_s,
                stopped,
                position_error,
                heading_error,
                shelf_relative_active=shelf_relative_active,
                shelf_clearance_m=shelf_clearance_m,
            )
        return self._alignment_hold(position_error, heading_error)

    def _settle(
        self,
        now_s: float,
        stopped: bool,
        position_error: float,
        heading_error: float,
        *,
        shelf_relative_active: bool = False,
        shelf_clearance_m: float | None = None,
    ) -> PrecisionDockingDecision:
        self._phase = PrecisionDockingPhase.STOP_SETTLE
        self._stable_started_s = None
        self._settle_started_s = now_s if stopped else None
        return self._decision(
            self._phase,
            position_error,
            heading_error,
            use_fixed_reference=True,
            shelf_relative_active=shelf_relative_active,
            shelf_clearance_m=shelf_clearance_m,
        )

    def _select_trim(
        self,
        now_s: float,
        state: VehicleState,
        checkpoint: MissionCheckpoint,
        position_error: float,
        heading_error: float,
        *,
        body_x_error: float,
        body_y_error: float,
        shelf_relative_active: bool,
        shelf_clearance_m: float | None,
        shelf_observation: ShelfObservation | None,
    ) -> PrecisionDockingDecision:
        self._settle_started_s = None
        if (
            shelf_relative_active
            and shelf_clearance_m is not None
            and shelf_clearance_m < self._config.minimum_side_clearance_m
        ):
            self._phase = PrecisionDockingPhase.POSITION_TRIM
            return self._parallel_command(
                state,
                checkpoint,
                position_error,
                heading_error,
                body_x_error=0.0,
                body_y_error=body_y_error,
                shelf_relative_active=True,
                shelf_clearance_m=shelf_clearance_m,
            )
        if abs(heading_error) > self._config.max_spin_correction_rad:
            return self._alignment_hold(position_error, heading_error)
        if abs(heading_error) > self._config.heading_realign_tolerance_rad:
            self._phase = PrecisionDockingPhase.HEADING_TRIM
            return self.update(
                now_s=now_s,
                state=state,
                checkpoint=checkpoint,
                yaw_rate_radps=0.0,
                shelf_observation=shelf_observation,
            )
        if position_error > self._config.max_parallel_correction_m:
            return self._alignment_hold(position_error, heading_error)
        if position_error > self._config.final_position_tolerance_m:
            self._phase = PrecisionDockingPhase.POSITION_TRIM
            return self._parallel_command(
                state,
                checkpoint,
                position_error,
                heading_error,
                body_x_error=body_x_error,
                body_y_error=body_y_error,
                shelf_relative_active=shelf_relative_active,
                shelf_clearance_m=shelf_clearance_m,
            )
        self._phase = PrecisionDockingPhase.DOCK_READY
        self._stable_started_s = now_s
        return self._decision(
            self._phase,
            position_error,
            heading_error,
            use_fixed_reference=True,
            shelf_relative_active=shelf_relative_active,
            shelf_clearance_m=shelf_clearance_m,
        )

    def _parallel_command(
        self,
        state: VehicleState,
        checkpoint: MissionCheckpoint,
        position_error: float,
        heading_error: float,
        *,
        body_x_error: float | None = None,
        body_y_error: float | None = None,
        shelf_relative_active: bool = False,
        shelf_clearance_m: float | None = None,
    ) -> PrecisionDockingDecision:
        if body_x_error is None or body_y_error is None:
            dx = checkpoint.x - state.x
            dy = checkpoint.y - state.y
            cos_yaw = math.cos(state.yaw)
            sin_yaw = math.sin(state.yaw)
            body_x_error = cos_yaw * dx + sin_yaw * dy
            body_y_error = -sin_yaw * dx + cos_yaw * dy
        raw_x = self._config.parallel_gain * body_x_error
        raw_y = self._config.parallel_gain * body_y_error
        magnitude = math.hypot(raw_x, raw_y)
        if magnitude <= 1e-12:
            return self._alignment_hold(position_error, heading_error)
        commanded = min(
            self._config.parallel_max_speed_mps,
            max(self._config.parallel_min_speed_mps, magnitude),
        )
        scale = commanded / magnitude
        return self._decision(
            PrecisionDockingPhase.POSITION_TRIM,
            position_error,
            heading_error,
            mode=RequestedMotionMode.PARALLEL,
            linear_x_mps=raw_x * scale,
            linear_y_mps=raw_y * scale,
            use_fixed_reference=True,
            shelf_relative_active=shelf_relative_active,
            shelf_clearance_m=shelf_clearance_m,
        )

    def _alignment_hold(
        self,
        position_error: float,
        heading_error: float,
    ) -> PrecisionDockingDecision:
        self._phase = PrecisionDockingPhase.ALIGNMENT_HOLD
        return self._decision(
            self._phase,
            position_error,
            heading_error,
            use_fixed_reference=True,
        )

    @staticmethod
    def _decision(
        phase: PrecisionDockingPhase,
        position_error: float,
        heading_error: float,
        *,
        mode: RequestedMotionMode = RequestedMotionMode.DUAL_ACKERMANN,
        linear_x_mps: float = 0.0,
        linear_y_mps: float = 0.0,
        yaw_rate_radps: float = 0.0,
        use_fixed_reference: bool = False,
        pose_ready: bool = False,
        shelf_relative_active: bool = False,
        shelf_clearance_m: float | None = None,
    ) -> PrecisionDockingDecision:
        return PrecisionDockingDecision(
            phase=phase,
            motion_mode=mode,
            use_dynamic_replanning=not use_fixed_reference,
            use_fixed_reference=use_fixed_reference,
            linear_x_mps=linear_x_mps,
            linear_y_mps=linear_y_mps,
            yaw_rate_radps=yaw_rate_radps,
            pose_ready=pose_ready,
            position_error_m=position_error,
            heading_error_rad=heading_error,
            shelf_relative_active=shelf_relative_active,
            shelf_clearance_m=shelf_clearance_m,
        )


def _bounded_signed(value: float, minimum: float, maximum: float) -> float:
    if abs(value) <= 1e-12:
        return 0.0
    return math.copysign(min(maximum, max(minimum, abs(value))), value)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "on", "1"}:
            return True
        if normalized in {"false", "no", "off", "0"}:
            return False
    raise ValueError(f"expected boolean, got {value!r}")
