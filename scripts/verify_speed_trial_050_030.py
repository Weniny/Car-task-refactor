#!/usr/bin/env python3
"""Read-only checks for the unverified 0.50 straight / 0.30 turn candidate."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import yaml

from competition_control.mission_speed_zones import (
    MissionSpeedZoneLimiter,
    route_speed_profile_from_route,
)
from competition_control.mppi_controller import control_trajectories_from_dict
from competition_planning.artifact_provenance import (
    resolve_trajectory_source_paths,
    validate_source_manifest,
)
from competition_planning.semantic_planner import PathPoint
from competition_planning.trajectory_parameterizer import parameterize_local_path


def main() -> None:
    ws = Path(__file__).resolve().parent.parent

    def load(relative: str):
        return yaml.safe_load((ws / relative).read_text(encoding="utf-8"))

    reference_path = ws / (
        "artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml"
    )
    baseline = yaml.safe_load(reference_path.read_text(encoding="utf-8"))
    artifact = load(
        "artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml"
    )
    route = load("routes/indoor_manual_six_anchor_v3_radius120_050straight_030turn_loop.yaml")
    optimizer = load("config/planning/continuous_trajectory_radius120_050straight_030turn.yaml")
    safety = load("config/safety/safety_params_rebuild_050straight_030turn.yaml")
    old_safety = load("config/safety/safety_params_rebuild_020mps_turn_recovery.yaml")
    old_safety["safety"]["max_speed_mps"] = 0.5
    assert safety == old_safety, "Candidate changed a protection beyond normal speed limit"

    sources = resolve_trajectory_source_paths(
        route_file=ws / "routes/indoor_manual_six_anchor_v3_radius120_050straight_030turn_loop.yaml",
        semantic_map_file=ws / "maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml",
        planning_params_file=ws / "config/planning/semantic_lidar_guarded_six_anchor_radius120.yaml",
        optimizer_params_file=ws / "config/planning/continuous_trajectory_radius120_050straight_030turn.yaml",
    )
    validate_source_manifest(artifact, sources)
    assert artifact["source_manifest"]["reference_trajectory"]["sha256"] == (
        hashlib.sha256(reference_path.read_bytes()).hexdigest()
    )
    assert artifact["ok"] is True
    assert artifact["point_count"] == len(artifact["points"]) == len(baseline["points"]) == 3109
    geometry_fields = ("x", "y", "yaw", "s", "curvature", "ref_id")
    for original, candidate in zip(baseline["points"], artifact["points"]):
        assert all(original.get(key) == candidate.get(key) for key in geometry_fields)
    points = artifact["points"]
    assert max(point["v"] for point in points) <= 0.500001
    assert max(point["v"] for point in points) >= 0.49
    assert points[0]["v"] == points[-1]["v"] == 0.0
    assert max(point["a"] for point in points) <= 0.1201
    assert min(point["a"] for point in points) >= -0.1801
    assert max(abs(point["jerk"]) for point in points) <= 0.2001

    trajectory = control_trajectories_from_dict(artifact)[0]
    profile = route_speed_profile_from_route(route, trajectory=trajectory)
    assert len(profile.zones) == 4
    limiter = MissionSpeedZoneLimiter(
        trajectory,
        profile.zones,
        max_speed_mps=0.5,
        max_deceleration_mps2=0.18,
        default_speed_limit_mps=profile.default_max_speed_mps,
    )
    zones = []
    for zone in profile.zones:
        segment = points[zone.start_index:zone.end_index + 1]
        assert max(point["v"] for point in segment) <= 0.300001
        for index in range(zone.start_index, zone.end_index + 1):
            assert limiter.decision_for_index(index).speed_limit_mps <= 0.300001
        assert limiter.decision_for_index(zone.start_index - 1).phase == "BRAKING"
        zones.append({"id": zone.ref_id, "start": zone.start_index,
                      "end": zone.end_index, "max_reference_v": max(p["v"] for p in segment)})

    # A curved online detour must also respect the local parameterizer's cap.
    arc = tuple(
        PathPoint(1.2 * math.sin(t), 1.2 * (1.0 - math.cos(t)), t)
        for t in (i * math.pi / 80 for i in range(41))
    )
    local = parameterize_local_path(arc, {}, optimizer)
    curved = [point for point in local.points if abs(point.curvature) >= 0.5]
    assert curved and max(point.v for point in curved) <= 0.300001
    assert max(point.v for point in curved) > 0.25

    print(json.dumps({
        "status": "PASS",
        "point_count": len(points),
        "geometry": "identical to verified 0.20 baseline",
        "path_length_m": artifact["path_length_m"],
        "reference_duration_s": artifact["duration_s"],
        "max_reference_speed_mps": max(point["v"] for point in points),
        "turn_zones": zones,
        "online_curved_path_max_speed_mps": max(point.v for point in curved),
        "protections": "unchanged except normal speed limit 0.20 -> 0.50",
        "hardware": "not started by this verifier",
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
