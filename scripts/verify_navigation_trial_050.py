#!/usr/bin/env python3
"""Check the offline artifacts for the unvalidated 0.5 m/s trial."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

import yaml

from competition_planning.artifact_provenance import (
    resolve_trajectory_source_paths,
    validate_source_manifest,
)
from competition_planning.semantic_planner import PathPoint
from competition_planning.trajectory_parameterizer import parameterize_local_path


WS = Path(__file__).resolve().parents[1]


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def check_pair(
    segment: str, source_paths: dict[str, Path], *, profile_suffix: str = ""
) -> None:
    old = (
        WS / "artifacts"
        / f"indoor_manual_{segment}_radius120_turnrate250_020mps_trajectory.yaml"
    )
    new = (
        WS / "artifacts"
        / f"indoor_manual_{segment}_radius120_turnrate250_050mps{profile_suffix}_trajectory.yaml"
    )
    original = load_yaml(old)
    artifact = load_yaml(new)
    validate_source_manifest(artifact, source_paths)
    reference = artifact["source_manifest"].get("reference_trajectory", {})
    require(
        reference.get("sha256") == hashlib.sha256(old.read_bytes()).hexdigest(),
        f"{segment}: reference trajectory hash mismatch",
    )

    points = artifact["points"]
    original_points = original["points"]
    require(
        artifact["ok"] is True and len(points) == len(original_points) >= 2,
        f"{segment}: invalid point count or failed trajectory",
    )
    geometry_keys = ("x", "y", "yaw", "s", "curvature")
    require(
        all(
            all(point[key] == previous[key] for key in geometry_keys)
            for point, previous in zip(points, original_points)
        ),
        f"{segment}: retiming changed route geometry",
    )
    if profile_suffix:
        previous_trial = load_yaml(
            WS
            / "artifacts"
            / f"indoor_manual_{segment}_radius120_turnrate250_050mps_trajectory.yaml"
        )
        require(
            points == previous_trial["points"],
            f"{segment}: local-only profile changed the global trajectory",
        )
    require(points[0]["v"] == points[-1]["v"] == 0.0, f"{segment}: no endpoint stop")
    require(
        all(0.0 <= point["v"] <= 0.5001 for point in points),
        f"{segment}: speed exceeds limit",
    )
    require(
        all(math.isfinite(point["t"]) for point in points),
        f"{segment}: nonfinite time",
    )
    require(
        all(
            later["s"] > earlier["s"] and later["t"] > earlier["t"]
            for earlier, later in zip(points, points[1:])
        ),
        f"{segment}: distance or time is not increasing",
    )
    require(max(point["a"] for point in points) <= 0.121, f"{segment}: acceleration")
    require(min(point["a"] for point in points) >= -0.181, f"{segment}: deceleration")
    require(
        max(abs(point["jerk"]) for point in points) <= 0.201,
        f"{segment}: jerk",
    )
    print(
        f"{segment}: {len(points)} points, {points[-1]['s']:.3f} m, "
        f"max {max(point['v'] for point in points):.3f} m/s, "
        f"duration {artifact['duration_s']:.3f} s"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("original", "localcap"), default="original")
    args = parser.parse_args()
    control = load_yaml(WS / "config/control/control_params_rebuild_050mps.yaml")
    safety = load_yaml(WS / "config/safety/safety_params_rebuild_050mps_inflation035.yaml")
    profile_suffix = "_localcap" if args.profile == "localcap" else ""
    optimizer_path = (
        WS
        / "config/planning"
        / f"continuous_trajectory_radius090_turnrate250_050mps{profile_suffix}.yaml"
    )
    optimizer = load_yaml(optimizer_path)
    require(control["motion"]["max_speed_mps"] == 0.5, "controller speed mismatch")
    require(safety["safety"]["max_speed_mps"] == 0.5, "Safety speed mismatch")
    require(
        safety["proximity_stop"]["grid_inflation_radius_m"] == 0.35,
        "inflation radius mismatch",
    )
    require(
        optimizer["continuous_trajectory_optimizer"]["max_speed_mps"] == 0.5,
        "optimizer speed mismatch",
    )
    if args.profile == "localcap":
        local = optimizer.get("trajectory_optimizer", {})
        require(local.get("max_speed_mps") == 0.5, "local speed mismatch")
        require(
            local.get("max_lateral_acceleration_mps2") == 0.12,
            "local lateral acceleration mismatch",
        )
        require(
            local.get("high_curvature_threshold_1pm") == 0.5
            and local.get("high_curvature_speed_limit_mps") == 0.25,
            "local turn cap mismatch",
        )

    source_paths = resolve_trajectory_source_paths(
        route_file=WS / "routes/indoor_manual_six_anchor_v3_radius090_loop.yaml",
        semantic_map_file=(
            WS / "maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml"
        ),
        planning_params_file=(
            WS / "config/planning/semantic_lidar_guarded_six_anchor_radius120.yaml"
        ),
        optimizer_params_file=optimizer_path,
    )
    check_pair("first_segment_30m", source_paths, profile_suffix=profile_suffix)
    check_pair("six_anchor_v3", source_paths, profile_suffix=profile_suffix)

    if args.profile == "localcap":
        artifact = load_yaml(
            WS
            / "artifacts"
            / "indoor_manual_first_segment_30m_radius120_turnrate250_050mps_localcap_trajectory.yaml"
        )
        geometry = tuple(
            PathPoint(x=point["x"], y=point["y"], yaw=point["yaw"])
            for point in artifact["points"]
            if 21.0 <= point["s"] <= 26.0
        )
        semantic_map = load_yaml(source_paths["semantic_map"])
        local_plan = parameterize_local_path(geometry, semantic_map, optimizer)
        turn_speeds = [
            point.v for point in local_plan.points if abs(point.curvature) >= 0.5
        ]
        require(turn_speeds, "local test path has no turn")
        require(max(turn_speeds) <= 0.250001, "online turn speed is not capped")
        print(
            f"online turn sample: {len(turn_speeds)} curved points, "
            f"max {max(turn_speeds):.3f} m/s"
        )


if __name__ == "__main__":
    main()
