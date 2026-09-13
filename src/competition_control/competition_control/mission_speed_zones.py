"""Route-progress speed limits that do not depend on mission-topic delivery."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence, TYPE_CHECKING

from competition_control.mission_markers import MissionMarker

if TYPE_CHECKING:
    from competition_control.mppi_controller import ControlTrajectory


@dataclass(frozen=True)
class MissionSpeedZone:
    ref_id: str
    start_index: int
    end_index: int
    max_speed_mps: float
    route_progress_owned: bool = False


@dataclass(frozen=True)
class RouteSpeedProfile:
    default_max_speed_mps: float | None
    zones: tuple[MissionSpeedZone, ...]


@dataclass(frozen=True)
class MissionSpeedLimitDecision:
    speed_limit_mps: float | None
    zone_ref: str | None
    phase: str
    progress_index: int


def mission_speed_zones_from_route(
    route: dict[str, Any],
    *,
    markers: Sequence[MissionMarker],
    trajectory: ControlTrajectory,
) -> tuple[MissionSpeedZone, ...]:
    """Resolve fixed marker-to-checkpoint speed zones on the global path."""

    raw_zones = route.get("mission_speed_zones", [])
    if raw_zones is None:
        return ()
    if not isinstance(raw_zones, list):
        raise ValueError("mission_speed_zones must be a list")
    marker_by_ref = {marker.ref_id: marker for marker in markers}
    point_indices_by_ref: dict[str, list[int]] = {}
    for index, point in enumerate(trajectory.points):
        if point.ref_id:
            point_indices_by_ref.setdefault(point.ref_id, []).append(index)

    zones: list[MissionSpeedZone] = []
    seen_refs: set[str] = set()
    for raw_zone in raw_zones:
        if not isinstance(raw_zone, dict):
            raise ValueError("mission speed zone entries must be mappings")
        ref_id = str(raw_zone.get("id", "")).strip()
        start_marker_ref = str(
            raw_zone.get("start_marker_ref", "")
        ).strip()
        end_checkpoint_ref = str(
            raw_zone.get("end_checkpoint_ref", "")
        ).strip()
        try:
            end_occurrence = int(raw_zone["end_checkpoint_occurrence"])
            max_speed_mps = float(raw_zone["max_speed_mps"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("mission speed zone values are invalid") from exc
        if not ref_id or not start_marker_ref or not end_checkpoint_ref:
            raise ValueError("mission speed zone refs must be non-empty")
        if ref_id in seen_refs:
            raise ValueError(f"duplicate mission speed zone {ref_id}")
        if end_occurrence <= 0:
            raise ValueError("speed zone checkpoint occurrence must be positive")
        if not math.isfinite(max_speed_mps) or max_speed_mps <= 0.0:
            raise ValueError("mission speed zone limit must be positive")

        marker = marker_by_ref.get(start_marker_ref)
        if marker is None:
            raise ValueError(
                f"speed zone {ref_id} references unknown marker "
                f"{start_marker_ref}"
            )
        start_occurrence = marker.checkpoint_occurrence or 1
        start_targets = point_indices_by_ref.get(
            marker.before_checkpoint_ref,
            [],
        )
        end_targets = point_indices_by_ref.get(end_checkpoint_ref, [])
        if len(start_targets) < start_occurrence:
            raise ValueError(
                f"speed zone {ref_id} start occurrence is not on trajectory"
            )
        if len(end_targets) < end_occurrence:
            raise ValueError(
                f"speed zone {ref_id} end occurrence is not on trajectory"
            )
        start_target_index = start_targets[start_occurrence - 1]
        start_target_s = trajectory.points[start_target_index].s
        start_s = start_target_s - marker.trigger_distance_m
        previous_target_index = (
            start_targets[start_occurrence - 2] + 1
            if start_occurrence > 1
            else 0
        )
        start_index = min(
            range(previous_target_index, start_target_index + 1),
            key=lambda index: abs(trajectory.points[index].s - start_s),
        )
        end_index = end_targets[end_occurrence - 1]
        if end_index <= start_index:
            raise ValueError(
                f"speed zone {ref_id} must end after its start marker"
            )
        zones.append(
            MissionSpeedZone(
                ref_id=ref_id,
                start_index=start_index,
                end_index=end_index,
                max_speed_mps=max_speed_mps,
            )
        )
        seen_refs.add(ref_id)

    zones.sort(key=lambda zone: zone.start_index)
    for previous, current in zip(zones, zones[1:]):
        if current.start_index <= previous.end_index:
            raise ValueError(
                f"mission speed zones overlap: {previous.ref_id}, "
                f"{current.ref_id}"
            )
    return tuple(zones)


def route_speed_profile_from_route(
    route: dict[str, Any],
    *,
    trajectory: ControlTrajectory,
) -> RouteSpeedProfile:
    """Resolve route-wide point-to-point speed limits on the global path."""

    raw_profile = route.get("route_speed_profile")
    if raw_profile is None:
        return RouteSpeedProfile(default_max_speed_mps=None, zones=())
    if not isinstance(raw_profile, dict):
        raise ValueError("route_speed_profile must be a mapping")
    try:
        default_max_speed_mps = float(raw_profile["default_max_speed_mps"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("route speed profile default is invalid") from exc
    if (
        not math.isfinite(default_max_speed_mps)
        or default_max_speed_mps <= 0.0
    ):
        raise ValueError("route speed profile default must be positive")

    raw_segments = raw_profile.get("segments", [])
    if not isinstance(raw_segments, list):
        raise ValueError("route speed profile segments must be a list")
    point_indices_by_ref: dict[str, list[int]] = {}
    for index, point in enumerate(trajectory.points):
        if point.ref_id:
            point_indices_by_ref.setdefault(point.ref_id, []).append(index)

    zones: list[MissionSpeedZone] = []
    seen_refs: set[str] = set()
    for raw_segment in raw_segments:
        if not isinstance(raw_segment, dict):
            raise ValueError("route speed profile entries must be mappings")
        ref_id = str(raw_segment.get("id", "")).strip()
        start_ref = str(raw_segment.get("start_ref", "")).strip()
        end_ref = str(raw_segment.get("end_ref", "")).strip()
        try:
            start_occurrence = int(raw_segment.get("start_occurrence", 1))
            end_occurrence = int(raw_segment.get("end_occurrence", 1))
            max_speed_mps = float(raw_segment["max_speed_mps"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("route speed profile values are invalid") from exc
        if not ref_id or not start_ref or not end_ref:
            raise ValueError("route speed profile refs must be non-empty")
        if ref_id in seen_refs:
            raise ValueError(f"duplicate route speed segment {ref_id}")
        if start_occurrence <= 0 or end_occurrence <= 0:
            raise ValueError("route speed profile occurrences must be positive")
        if not math.isfinite(max_speed_mps) or max_speed_mps <= 0.0:
            raise ValueError("route speed profile limit must be positive")
        start_indices = point_indices_by_ref.get(start_ref, [])
        end_indices = point_indices_by_ref.get(end_ref, [])
        if len(start_indices) < start_occurrence:
            raise ValueError(
                f"route speed segment {ref_id} start is not on trajectory"
            )
        if len(end_indices) < end_occurrence:
            raise ValueError(
                f"route speed segment {ref_id} end is not on trajectory"
            )
        start_index = start_indices[start_occurrence - 1]
        end_index = end_indices[end_occurrence - 1]
        if end_index <= start_index:
            raise ValueError(
                f"route speed segment {ref_id} must end after its start"
            )
        zones.append(
            MissionSpeedZone(
                ref_id=ref_id,
                start_index=start_index,
                end_index=end_index,
                max_speed_mps=max_speed_mps,
                route_progress_owned=True,
            )
        )
        seen_refs.add(ref_id)
    zones.sort(key=lambda zone: (zone.start_index, zone.end_index))
    return RouteSpeedProfile(
        default_max_speed_mps=default_max_speed_mps,
        zones=tuple(zones),
    )


class MissionSpeedZoneLimiter:
    """Track global progress and enforce fixed zones with braking lookahead."""

    def __init__(
        self,
        trajectory: ControlTrajectory,
        zones: Sequence[MissionSpeedZone],
        *,
        max_speed_mps: float,
        max_deceleration_mps2: float,
        default_speed_limit_mps: float | None = None,
        braking_margin_m: float = 0.15,
        progress_search_window_points: int = 80,
        max_progress_advance_points: int = 10,
    ) -> None:
        if not math.isfinite(max_speed_mps) or max_speed_mps <= 0.0:
            raise ValueError("maximum speed must be finite and positive")
        if (
            not math.isfinite(max_deceleration_mps2)
            or max_deceleration_mps2 <= 0.0
        ):
            raise ValueError("maximum deceleration must be finite and positive")
        if not math.isfinite(braking_margin_m) or braking_margin_m < 0.0:
            raise ValueError("braking margin must be finite and non-negative")
        if default_speed_limit_mps is not None and (
            not math.isfinite(default_speed_limit_mps)
            or default_speed_limit_mps <= 0.0
        ):
            raise ValueError("default speed limit must be finite and positive")
        if progress_search_window_points <= 0:
            raise ValueError("progress search window must be positive")
        if max_progress_advance_points <= 0:
            raise ValueError("maximum progress advance must be positive")
        self._points = trajectory.points
        self._zones = tuple(zones)
        self._max_speed_mps = max_speed_mps
        self._default_speed_limit_mps = default_speed_limit_mps
        self._max_deceleration_mps2 = max_deceleration_mps2
        self._braking_margin_m = braking_margin_m
        self._progress_search_window_points = progress_search_window_points
        self._max_progress_advance_points = max_progress_advance_points
        self.reset()

    def reset(self) -> None:
        self._progress_index = 0

    def update(
        self,
        *,
        x: float,
        y: float,
        route_target_index: int | None = None,
    ) -> MissionSpeedLimitDecision:
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("vehicle position must be finite")
        start = max(0, self._progress_index - 3)
        end = min(
            len(self._points),
            self._progress_index + self._progress_search_window_points + 1,
        )
        nearest = min(
            range(start, end),
            key=lambda index: (
                (self._points[index].x - x) ** 2
                + (self._points[index].y - y) ** 2
            ),
        )
        nearest = min(
            nearest,
            self._progress_index + self._max_progress_advance_points,
        )
        self._progress_index = max(self._progress_index, nearest)
        return self.decision_for_index(
            self._progress_index,
            route_target_index=route_target_index,
        )

    def decision_for_index(
        self,
        index: int,
        *,
        route_target_index: int | None = None,
    ) -> MissionSpeedLimitDecision:
        if not 0 <= index < len(self._points):
            raise IndexError("trajectory progress index is out of range")
        if route_target_index is not None and not (
            0 <= route_target_index < len(self._points)
        ):
            raise IndexError("route target index is out of range")
        eligible_zones = [
            zone
            for zone in self._zones
            if zone.route_progress_owned
            or route_target_index is None
            or route_target_index <= zone.end_index
        ]
        active_zones = [
            zone
            for zone in eligible_zones
            if zone.start_index <= index <= zone.end_index
        ]
        active_zone = min(
            active_zones,
            key=lambda zone: zone.max_speed_mps,
            default=None,
        )
        speed_limit_mps = (
            active_zone.max_speed_mps
            if active_zone is not None
            else self._default_speed_limit_mps
        )
        zone_ref = active_zone.ref_id if active_zone is not None else None
        phase = (
            "ACTIVE"
            if active_zone is not None
            else "DEFAULT"
            if speed_limit_mps is not None
            else "NONE"
        )

        braking_reference_mps = min(
            self._max_speed_mps,
            speed_limit_mps
            if speed_limit_mps is not None
            else self._max_speed_mps,
        )
        for zone in eligible_zones:
            if zone.start_index <= index:
                continue
            remaining_m = max(
                0.0,
                self._points[zone.start_index].s
                - self._points[index].s
                - self._braking_margin_m,
            )
            braking_limit_mps = math.sqrt(
                zone.max_speed_mps * zone.max_speed_mps
                + 2.0 * self._max_deceleration_mps2 * remaining_m
            )
            if braking_limit_mps < braking_reference_mps:
                braking_reference_mps = braking_limit_mps
                speed_limit_mps = braking_limit_mps
                zone_ref = zone.ref_id
                phase = "BRAKING"

        return MissionSpeedLimitDecision(
            speed_limit_mps=speed_limit_mps,
            zone_ref=zone_ref,
            phase=phase,
            progress_index=index,
        )
