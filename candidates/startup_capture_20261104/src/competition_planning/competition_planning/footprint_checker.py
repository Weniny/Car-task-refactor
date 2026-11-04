#!/usr/bin/env python3
"""Offline rectangular footprint sweep checks for frozen control trajectories."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any, Sequence

import yaml

from competition_planning.occupancy_grid_planner import OccupancyGridMap


@dataclass(frozen=True)
class FootprintConfig:
    vehicle_length_m: float
    vehicle_width_m: float
    clearance_m: float = 0.0
    sample_spacing_m: float = 0.03
    body_offset_x_m: float = 0.0
    body_offset_y_m: float = 0.0


@dataclass(frozen=True)
class TrajectoryPose:
    x: float
    y: float
    yaw: float
    s: float | None = None
    t: float | None = None
    ref_id: str | None = None


@dataclass(frozen=True)
class FootprintViolation:
    pose_index: int
    x: float
    y: float
    yaw: float
    s: float | None
    t: float | None
    ref_id: str | None
    occupied_cell: tuple[int, int]
    occupied_world: tuple[float, float]


@dataclass(frozen=True)
class FootprintCheckResult:
    ok: bool
    checked_pose_count: int
    collision_pose_count: int
    first_violation: FootprintViolation | None

    def to_dict(self) -> dict[str, Any]:
        first = None
        if self.first_violation is not None:
            first = {
                "pose_index": self.first_violation.pose_index,
                "x": round(self.first_violation.x, 4),
                "y": round(self.first_violation.y, 4),
                "yaw": round(self.first_violation.yaw, 4),
                "s": (
                    None
                    if self.first_violation.s is None
                    else round(self.first_violation.s, 4)
                ),
                "t": (
                    None
                    if self.first_violation.t is None
                    else round(self.first_violation.t, 4)
                ),
                "ref_id": self.first_violation.ref_id,
                "occupied_cell": list(self.first_violation.occupied_cell),
                "occupied_world": [
                    round(self.first_violation.occupied_world[0], 4),
                    round(self.first_violation.occupied_world[1], 4),
                ],
            }
        return {
            "ok": self.ok,
            "checked_pose_count": self.checked_pose_count,
            "collision_pose_count": self.collision_pose_count,
            "first_violation": first,
        }


def poses_from_trajectory_artifact(path: str | Path) -> tuple[TrajectoryPose, ...]:
    with Path(path).open("r", encoding="utf-8") as stream:
        artifact = yaml.safe_load(stream)
    if not isinstance(artifact, dict):
        raise ValueError(f"{path} is not a YAML mapping")
    raw_points = artifact.get("points", [])
    if not isinstance(raw_points, list):
        raise ValueError(f"{path} has no trajectory points")
    return tuple(_pose_from_mapping(point) for point in raw_points)


def check_trajectory_footprint(
    poses: Sequence[TrajectoryPose],
    grid_map: OccupancyGridMap,
    config: FootprintConfig,
) -> FootprintCheckResult:
    if config.vehicle_length_m <= 0.0:
        raise ValueError("vehicle_length_m must be positive")
    if config.vehicle_width_m <= 0.0:
        raise ValueError("vehicle_width_m must be positive")
    if config.clearance_m < 0.0:
        raise ValueError("clearance_m must be non-negative")
    if config.sample_spacing_m <= 0.0:
        raise ValueError("sample_spacing_m must be positive")

    sampled = _sample_poses(poses, max(config.sample_spacing_m, grid_map.resolution))
    collision_pose_count = 0
    first_violation: FootprintViolation | None = None
    for pose_index, pose in enumerate(sampled):
        violation = _first_pose_violation(pose_index, pose, grid_map, config)
        if violation is None:
            continue
        collision_pose_count += 1
        if first_violation is None:
            first_violation = violation
    return FootprintCheckResult(
        ok=collision_pose_count == 0,
        checked_pose_count=len(sampled),
        collision_pose_count=collision_pose_count,
        first_violation=first_violation,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--vehicle-length-m", type=float, required=True)
    parser.add_argument("--vehicle-width-m", type=float, required=True)
    parser.add_argument("--clearance-m", type=float, default=0.0)
    parser.add_argument("--sample-spacing-m", type=float, default=0.03)
    parser.add_argument("--body-offset-x-m", type=float, default=0.0)
    parser.add_argument("--body-offset-y-m", type=float, default=0.0)
    parser.add_argument("--report")
    args = parser.parse_args(argv)

    result = check_trajectory_footprint(
        poses_from_trajectory_artifact(args.trajectory),
        OccupancyGridMap.from_yaml(args.map),
        FootprintConfig(
            vehicle_length_m=args.vehicle_length_m,
            vehicle_width_m=args.vehicle_width_m,
            clearance_m=args.clearance_m,
            sample_spacing_m=args.sample_spacing_m,
            body_offset_x_m=args.body_offset_x_m,
            body_offset_y_m=args.body_offset_y_m,
        ),
    )
    payload = result.to_dict()
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if args.report:
        _write_report(Path(args.report), args, payload)
    return 0 if result.ok else 2


def _pose_from_mapping(raw: Any) -> TrajectoryPose:
    if not isinstance(raw, dict):
        raise ValueError("trajectory point is not a mapping")
    return TrajectoryPose(
        x=float(raw["x"]),
        y=float(raw["y"]),
        yaw=float(raw["yaw"]),
        s=float(raw["s"]) if raw.get("s") is not None else None,
        t=float(raw["t"]) if raw.get("t") is not None else None,
        ref_id=str(raw["ref_id"]) if raw.get("ref_id") else None,
    )


def _sample_poses(
    poses: Sequence[TrajectoryPose],
    spacing_m: float,
) -> tuple[TrajectoryPose, ...]:
    if len(poses) < 2:
        return tuple(poses)
    sampled: list[TrajectoryPose] = []
    for start, end in zip(poses, poses[1:]):
        distance = math.hypot(end.x - start.x, end.y - start.y)
        count = max(1, math.ceil(distance / spacing_m))
        for index in range(count):
            if sampled and index == 0:
                continue
            ratio = index / count
            sampled.append(_interpolate_pose(start, end, ratio))
    sampled.append(poses[-1])
    return tuple(sampled)


def _interpolate_pose(
    start: TrajectoryPose,
    end: TrajectoryPose,
    ratio: float,
) -> TrajectoryPose:
    yaw_delta = _wrap_angle(end.yaw - start.yaw)
    return TrajectoryPose(
        x=start.x + (end.x - start.x) * ratio,
        y=start.y + (end.y - start.y) * ratio,
        yaw=_wrap_angle(start.yaw + yaw_delta * ratio),
        s=_interpolate_optional(start.s, end.s, ratio),
        t=_interpolate_optional(start.t, end.t, ratio),
        ref_id=start.ref_id if ratio == 0.0 else None,
    )


def _interpolate_optional(
    start: float | None,
    end: float | None,
    ratio: float,
) -> float | None:
    if start is None or end is None:
        return None
    return start + (end - start) * ratio


def _first_pose_violation(
    pose_index: int,
    pose: TrajectoryPose,
    grid_map: OccupancyGridMap,
    config: FootprintConfig,
) -> FootprintViolation | None:
    yaw_cos = math.cos(pose.yaw)
    yaw_sin = math.sin(pose.yaw)
    center_x = pose.x + yaw_cos * config.body_offset_x_m - yaw_sin * config.body_offset_y_m
    center_y = pose.y + yaw_sin * config.body_offset_x_m + yaw_cos * config.body_offset_y_m
    half_length = 0.5 * config.vehicle_length_m + config.clearance_m
    half_width = 0.5 * config.vehicle_width_m + config.clearance_m
    radius_cells = int(
        math.ceil(math.hypot(half_length, half_width) / grid_map.resolution)
    ) + 2
    center_col, center_row = grid_map.world_to_cell(center_x, center_y)
    min_col = max(0, center_col - radius_cells)
    max_col = min(grid_map.width - 1, center_col + radius_cells)
    min_row = max(0, center_row - radius_cells)
    max_row = min(grid_map.height - 1, center_row + radius_cells)

    for row in range(min_row, max_row + 1):
        for col in range(min_col, max_col + 1):
            if not grid_map.occupied[grid_map.index((col, row))]:
                continue
            cell_x, cell_y = grid_map.cell_to_world((col, row))
            dx = cell_x - center_x
            dy = cell_y - center_y
            body_x = yaw_cos * dx + yaw_sin * dy
            body_y = -yaw_sin * dx + yaw_cos * dy
            if abs(body_x) <= half_length and abs(body_y) <= half_width:
                return FootprintViolation(
                    pose_index=pose_index,
                    x=pose.x,
                    y=pose.y,
                    yaw=pose.yaw,
                    s=pose.s,
                    t=pose.t,
                    ref_id=pose.ref_id,
                    occupied_cell=(col, row),
                    occupied_world=(cell_x, cell_y),
                )
    return None


def _write_report(path: Path, args: argparse.Namespace, payload: dict[str, Any]) -> None:
    status = "PASS" if payload["ok"] else "FAIL"
    lines = [
        "# Footprint sweep check",
        "",
        f"- status: {status}",
        f"- trajectory: `{args.trajectory}`",
        f"- map: `{args.map}`",
        f"- vehicle_length_m: {args.vehicle_length_m:.3f}",
        f"- vehicle_width_m: {args.vehicle_width_m:.3f}",
        f"- clearance_m: {args.clearance_m:.3f}",
        f"- checked_pose_count: {payload['checked_pose_count']}",
        f"- collision_pose_count: {payload['collision_pose_count']}",
    ]
    if payload["first_violation"] is not None:
        lines.extend(["", "## First violation", ""])
        for key, value in payload["first_violation"].items():
            lines.append(f"- {key}: {value}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


if __name__ == "__main__":
    sys.exit(main())
