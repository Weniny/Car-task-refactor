"""Geometry-only continuity checks for online local trajectories."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any, Protocol, Sequence


class PathPose(Protocol):
    x: float
    y: float
    yaw: float


def stop_line_lengths_excluding_docks(
    semantic_map: Mapping[str, Any],
) -> dict[str, float]:
    """Return stop-line semantics that are not precision docking poses."""

    dock_refs = {
        str(record["point_ref"])
        for record in semantic_map.get("dock_poses", [])
        if isinstance(record, Mapping) and record.get("point_ref")
    }
    result: dict[str, float] = {}
    for record in semantic_map.get("stop_lines", []):
        if not isinstance(record, Mapping) or not record.get("point_ref"):
            continue
        point_ref = str(record["point_ref"])
        if point_ref in dock_refs:
            continue
        length_m = float(record.get("length_m", 0.0))
        if length_m <= 0.0:
            raise ValueError(f"stop line {point_ref} requires positive length_m")
        result[point_ref] = length_m
    return result


def local_paths_are_equivalent(
    previous: Sequence[PathPose],
    current: Sequence[PathPose],
    *,
    position_tolerance_m: float = 0.05,
    heading_tolerance_rad: float = math.radians(5.0),
) -> bool:
    """Return whether ``current`` keeps a geometrically identical future path.

    Local replans prepend the latest vehicle pose, so multi-point paths compare
    their future suffix.  A two-point checkpoint path is reused only when both
    poses still match.  Real bypass, rejoin, or extension changes therefore
    continue to replace the controller trajectory.
    """

    if position_tolerance_m < 0.0 or heading_tolerance_rad < 0.0:
        raise ValueError("path equivalence tolerances must be non-negative")
    if len(previous) == 2 or len(current) == 2:
        return (
            len(previous) == len(current) == 2
            and _finite_path(previous)
            and _finite_path(current)
            and all(
                _poses_match(
                    old,
                    new,
                    position_tolerance_m=position_tolerance_m,
                    heading_tolerance_rad=heading_tolerance_rad,
                )
                for old, new in zip(previous, current)
            )
        )
    if len(previous) < 3 or len(current) < 3:
        return False

    current_future = current[1:]
    if len(current_future) < 2:
        return False
    if not _finite_path(current_future):
        return False

    for start_index in range(1, len(previous)):
        previous_future = previous[start_index:]
        if len(previous_future) != len(current_future):
            continue
        if not _finite_path(previous_future):
            continue
        if all(
            _poses_match(
                old,
                new,
                position_tolerance_m=position_tolerance_m,
                heading_tolerance_rad=heading_tolerance_rad,
            )
            for old, new in zip(previous_future, current_future)
        ):
            return True
    return False


def nearest_path_point_index(
    path: Sequence[PathPose],
    checkpoint: PathPose,
    *,
    max_distance_m: float,
) -> int | None:
    """Find a checkpoint on local geometry without snapping from far away."""

    if max_distance_m < 0.0:
        raise ValueError("max_distance_m must be non-negative")
    if not path or not _finite_path(path):
        return None
    distances = tuple(
        math.hypot(
            float(point.x) - float(checkpoint.x),
            float(point.y) - float(checkpoint.y),
        )
        for point in path
    )
    index = min(range(len(path)), key=distances.__getitem__)
    return index if distances[index] <= max_distance_m else None


def nearest_stop_line_path_point_index(
    path: Sequence[PathPose],
    checkpoint: PathPose,
    *,
    line_length_m: float,
    max_longitudinal_distance_m: float,
) -> int | None:
    """Find where local geometry crosses a finite semantic stop line."""

    if line_length_m <= 0.0:
        raise ValueError("line_length_m must be positive")
    if max_longitudinal_distance_m < 0.0:
        raise ValueError("max_longitudinal_distance_m must be non-negative")
    if not path or not _finite_path(path) or not _finite_path((checkpoint,)):
        return None
    candidates = []
    for index, point in enumerate(path):
        longitudinal_m, lateral_m = _checkpoint_frame_offsets(point, checkpoint)
        if abs(lateral_m) <= 0.5 * line_length_m:
            candidates.append((abs(longitudinal_m), index))
    if not candidates:
        return None
    longitudinal_error_m, index = min(candidates)
    return (
        index
        if longitudinal_error_m <= max_longitudinal_distance_m
        else None
    )


def checkpoint_errors(
    pose: PathPose,
    checkpoint: PathPose,
    *,
    stop_line_length_m: float | None = None,
) -> tuple[float, float]:
    """Return state-machine errors for an exact pose or finite stop line."""

    if not _finite_path((pose, checkpoint)):
        return math.inf, math.inf
    if stop_line_length_m is not None:
        if stop_line_length_m <= 0.0:
            raise ValueError("stop_line_length_m must be positive")
        longitudinal_m, lateral_m = _checkpoint_frame_offsets(pose, checkpoint)
        if abs(lateral_m) > 0.5 * stop_line_length_m:
            return math.inf, 0.0
        return abs(longitudinal_m), 0.0
    return (
        math.hypot(
            float(pose.x) - float(checkpoint.x),
            float(pose.y) - float(checkpoint.y),
        ),
        math.atan2(
            math.sin(float(pose.yaw) - float(checkpoint.yaw)),
            math.cos(float(pose.yaw) - float(checkpoint.yaw)),
        ),
    )


def checkpoint_longitudinal_error(
    pose: PathPose,
    checkpoint: PathPose,
) -> float:
    """Return signed checkpoint-axis error; positive means the goal was passed."""

    if not _finite_path((pose, checkpoint)):
        return math.nan
    longitudinal_m, _ = _checkpoint_frame_offsets(pose, checkpoint)
    return longitudinal_m


def _checkpoint_frame_offsets(
    pose: PathPose,
    checkpoint: PathPose,
) -> tuple[float, float]:
    delta_x_m = float(pose.x) - float(checkpoint.x)
    delta_y_m = float(pose.y) - float(checkpoint.y)
    cos_yaw = math.cos(float(checkpoint.yaw))
    sin_yaw = math.sin(float(checkpoint.yaw))
    return (
        cos_yaw * delta_x_m + sin_yaw * delta_y_m,
        -sin_yaw * delta_x_m + cos_yaw * delta_y_m,
    )


def _finite_path(path: Sequence[PathPose]) -> bool:
    return all(
        math.isfinite(float(value))
        for point in path
        for value in (point.x, point.y, point.yaw)
    )


def _poses_match(
    first: PathPose,
    second: PathPose,
    *,
    position_tolerance_m: float,
    heading_tolerance_rad: float,
) -> bool:
    position_error_m = math.hypot(
        float(first.x) - float(second.x),
        float(first.y) - float(second.y),
    )
    heading_error_rad = abs(
        math.atan2(
            math.sin(float(first.yaw) - float(second.yaw)),
            math.cos(float(first.yaw) - float(second.yaw)),
        )
    )
    return (
        position_error_m <= position_tolerance_m
        and heading_error_rad <= heading_tolerance_rad
        and getattr(first, "ref_id", None) == getattr(second, "ref_id", None)
    )
