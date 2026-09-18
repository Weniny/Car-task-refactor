"""Conservative path-aware alternative to the fixed forward stop box."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable, Sequence

from competition_safety.proximity_stop import (
    ProximityStopConfig,
    count_points_in_stop_box,
)


@dataclass(frozen=True)
class PathAwareConfig:
    slow_distance_m: float = 2.5
    slow_speed_mps: float = 0.25
    emergency_distance_m: float = 1.75
    minimum_emergency_distance_m: float = 0.65
    emergency_half_width_m: float = 0.35
    swept_radius_m: float = 0.65
    path_start_tolerance_m: float = 0.40
    max_path_segment_m: float = 0.35
    release_hold_s: float = 0.40
    clear_scan_count: int = 3
    max_speed_mps: float = 0.50
    min_deceleration_mps2: float = 0.18
    reaction_time_s: float = 0.90
    stopping_margin_m: float = 0.20
    front_overhang_m: float = 0.36

    def validate(self, fixed: ProximityStopConfig) -> None:
        if any(
            not math.isfinite(value)
            for value in (
                self.slow_distance_m,
                self.slow_speed_mps,
                self.emergency_distance_m,
                self.minimum_emergency_distance_m,
                self.emergency_half_width_m,
                self.swept_radius_m,
                self.path_start_tolerance_m,
                self.max_path_segment_m,
                self.release_hold_s,
                self.max_speed_mps,
                self.min_deceleration_mps2,
                self.reaction_time_s,
                self.stopping_margin_m,
                self.front_overhang_m,
            )
        ):
            raise ValueError("path-aware distances and speeds must be finite")
        if fixed.stop_distance_m <= 0.0:
            raise ValueError("path-aware mode requires an enabled fixed fallback")
        if not fixed.x_min_m < self.emergency_distance_m <= fixed.stop_distance_m:
            raise ValueError("emergency distance must fit inside the fixed stop box")
        if not fixed.x_min_m < self.minimum_emergency_distance_m <= self.emergency_distance_m:
            raise ValueError("minimum emergency distance is invalid")
        if self.slow_distance_m < fixed.stop_distance_m:
            raise ValueError("slow distance must cover the fixed stop box")
        if not 0.0 < self.slow_speed_mps < self.max_speed_mps:
            raise ValueError("slow speed must be between zero and maximum speed")
        if (
            self.min_deceleration_mps2 <= 0.0
            or self.reaction_time_s < 0.0
            or self.stopping_margin_m < 0.0
            or self.front_overhang_m < 0.0
        ):
            raise ValueError("braking parameters are invalid")
        braking_distance = (
            self.max_speed_mps**2 / (2.0 * self.min_deceleration_mps2)
            + self.max_speed_mps * self.reaction_time_s
            + self.stopping_margin_m
            + self.front_overhang_m
        )
        if self.emergency_distance_m < braking_distance:
            raise ValueError("emergency distance is shorter than the braking bound")
        if not 0.0 < self.emergency_half_width_m <= self.swept_radius_m:
            raise ValueError("emergency half-width must fit inside swept radius")
        if self.path_start_tolerance_m <= 0.0 or self.max_path_segment_m <= 0.0:
            raise ValueError("path validation distances must be positive")
        if self.release_hold_s < 0.0 or self.clear_scan_count < 1:
            raise ValueError("release hysteresis is invalid")


@dataclass(frozen=True)
class PathAwareDecision:
    stop: bool
    reason: str
    path_valid: bool
    fixed_count: int
    path_collision_count: int
    emergency_count: int
    slow_count: int
    speed_limit_mps: float | None
    emergency_distance_m: float | None = None


def evaluate_path_aware_stop(
    points: Iterable[tuple[float, float, float]],
    path_body: Sequence[tuple[float, float]] | None,
    fixed: ProximityStopConfig,
    config: PathAwareConfig,
    *,
    measured_speed_mps: float | None = None,
) -> PathAwareDecision:
    """A missing/invalid path retains the original fixed-box stop behavior."""

    config.validate(fixed)
    observed = tuple(
        (x, y, z)
        for x, y, z in points
        if math.isfinite(x)
        and math.isfinite(y)
        and math.isfinite(z)
        and fixed.z_min_m <= z <= fixed.z_max_m
    )
    fixed_count = count_points_in_stop_box(observed, fixed)
    slow_config = replace(fixed, stop_distance_m=config.slow_distance_m)
    slow_count = count_points_in_stop_box(observed, slow_config)
    speed_limit = config.slow_speed_mps if slow_count >= fixed.min_points else None

    path_valid = (
        _valid_path(path_body, config)
        and sum(math.dist(a, b) for a, b in zip(path_body, path_body[1:]))
        >= fixed.stop_distance_m
        and measured_speed_mps is not None
        and math.isfinite(measured_speed_mps)
        and 0.0 <= measured_speed_mps <= config.max_speed_mps
    )
    if not path_valid:
        return PathAwareDecision(
            stop=fixed_count >= fixed.min_points,
            reason="fixed_fallback" if fixed_count >= fixed.min_points else "clear_no_path",
            path_valid=False,
            fixed_count=fixed_count,
            path_collision_count=0,
            emergency_count=0,
            slow_count=slow_count,
            speed_limit_mps=speed_limit,
        )

    prospective_speed = speed_limit if speed_limit is not None else config.max_speed_mps
    braking_speed = max(measured_speed_mps, prospective_speed)
    emergency_distance = max(
        config.minimum_emergency_distance_m,
        braking_speed**2 / (2.0 * config.min_deceleration_mps2)
        + braking_speed * config.reaction_time_s
        + config.stopping_margin_m
        + config.front_overhang_m,
    )
    emergency_count = sum(
        fixed.x_min_m <= x <= emergency_distance
        and abs(y) <= config.emergency_half_width_m
        for x, y, _ in observed
    )
    segments = _lookahead_segments(path_body, fixed.stop_distance_m)
    path_count = sum(
        _near_segments(x, y, segments, config.swept_radius_m)
        for x, y, _ in observed
    )
    emergency_stop = emergency_count >= fixed.min_points
    path_stop = path_count >= fixed.min_points
    return PathAwareDecision(
        stop=emergency_stop or path_stop,
        reason=(
            "emergency_corridor"
            if emergency_stop
            else "obstacle_on_local_path"
            if path_stop
            else "clear_path"
        ),
        path_valid=True,
        fixed_count=fixed_count,
        path_collision_count=path_count,
        emergency_count=emergency_count,
        slow_count=slow_count,
        speed_limit_mps=speed_limit,
        emergency_distance_m=emergency_distance,
    )


class StopReleaseLatch:
    def __init__(self, hold_s: float, clear_scan_count: int) -> None:
        if hold_s < 0.0 or clear_scan_count < 1:
            raise ValueError("release hysteresis is invalid")
        self._hold_s = hold_s
        self._clear_scan_count = clear_scan_count
        self._last_stop_s: float | None = None
        self._clear_count = 0

    def update(self, raw_stop: bool, now_s: float) -> bool:
        if raw_stop:
            self._last_stop_s = now_s
            self._clear_count = 0
            return True
        if self._last_stop_s is None:
            return False
        self._clear_count += 1
        if (
            now_s - self._last_stop_s < self._hold_s
            or self._clear_count < self._clear_scan_count
        ):
            return True
        self._last_stop_s = None
        self._clear_count = 0
        return False


def _valid_path(path: Sequence[tuple[float, float]] | None, config: PathAwareConfig) -> bool:
    if path is None or len(path) < 2:
        return False
    if any(not math.isfinite(x) or not math.isfinite(y) for x, y in path):
        return False
    if math.hypot(*path[0]) > config.path_start_tolerance_m:
        return False
    first_dx = path[1][0] - path[0][0]
    if first_dx < 0.5 * math.dist(path[0], path[1]):
        return False
    return all(
        0.0 < math.dist(start, end) <= config.max_path_segment_m
        for start, end in zip(path, path[1:])
    )


def _lookahead_segments(
    path: Sequence[tuple[float, float]], distance_m: float
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    segments = []
    progress = 0.0
    for start, end in zip(path, path[1:]):
        segment_length = math.dist(start, end)
        remaining = distance_m - progress
        if remaining <= 0.0:
            break
        if segment_length > remaining:
            fraction = remaining / segment_length
            end = (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )
        segments.append((start, end))
        progress += segment_length
    return tuple(segments)


def _near_segments(
    x: float,
    y: float,
    segments: Sequence[tuple[tuple[float, float], tuple[float, float]]],
    radius_m: float,
) -> bool:
    radius_squared = radius_m * radius_m
    for (ax, ay), (bx, by) in segments:
        dx, dy = bx - ax, by - ay
        length_squared = dx * dx + dy * dy
        fraction = min(
            1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / length_squared)
        )
        nearest_x, nearest_y = ax + fraction * dx, ay + fraction * dy
        if (x - nearest_x) ** 2 + (y - nearest_y) ** 2 <= radius_squared:
            return True
    return False
