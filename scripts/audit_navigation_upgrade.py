#!/usr/bin/env python3
"""Read the pinned candidate and quantify upgrade constraints; never import ROS.

Requires PyYAML. Results describe local files, not running parameters or vehicle
acceptance. Constant-deceleration distances exclude jerk and hardware latency.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected a mapping: {path}")
    return data


def launch_literals(path: Path, name: str) -> dict:
    """Extract literal launch entries without executing launch or ROS code."""
    result = {}
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == name and isinstance(node.value, ast.Dict):
                for key, value in zip(node.value.keys, node.value.values):
                    try:
                        result[ast.literal_eval(key)] = ast.literal_eval(value)
                    except (ValueError, TypeError):
                        pass  # YAML references/substitutions must be resolved separately.
            if isinstance(target, ast.Subscript) and isinstance(target.value, ast.Name) and target.value.id == name:
                result[ast.literal_eval(target.slice)] = ast.literal_eval(node.value)
    return result


def launch_default(path: Path, argument: str) -> float:
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument":
            if node.args and ast.literal_eval(node.args[0]) == argument:
                return float(next(ast.literal_eval(k.value) for k in node.keywords if k.arg == "default_value"))
    raise ValueError(f"Missing launch argument: {argument}")


def braking_distance(speed: float, deceleration: float, latency: float = 0.0) -> float:
    if not all(math.isfinite(v) for v in (speed, deceleration, latency)) or speed < 0 or latency < 0 or deceleration <= 0:
        raise ValueError("Invalid braking inputs")
    return speed * latency + speed * speed / (2 * deceleration)


def audit(candidate: Path) -> dict:
    candidate = candidate.resolve()
    profile = json.loads((candidate / "CURRENT_RUNTIME_PROFILE.json").read_text(encoding="utf-8"))
    manifest = json.loads((candidate / "CANDIDATE_MANIFEST.json").read_text(encoding="utf-8"))
    mismatches = []
    for relative, expected in manifest["files"].items():
        file = (candidate / relative).resolve()
        if not file.is_relative_to(candidate):
            raise ValueError(f"Manifest escapes candidate: {relative}")
        actual = hashlib.sha256(file.read_bytes()).hexdigest() if file.is_file() else None
        if actual != expected:
            mismatches.append(relative)
    launch = candidate / "src/forward_rejoin_bringup/launch" / profile["launch"]
    launch_hash = hashlib.sha256(launch.read_bytes()).hexdigest()
    if launch_hash != profile["launch_sha256"]:
        mismatches.append("CURRENT_RUNTIME_PROFILE.json:launch_sha256")
    config = candidate / "config"
    control = load_yaml(config / "control/control_params_rebuild_050straight_030turn.yaml")
    safety_all = load_yaml(config / "safety/safety_params_rebuild_050straight_030turn.yaml")
    optimizers = load_yaml(config / "planning/continuous_trajectory_radius120_050straight_030turn.yaml")
    motion, safety = control["motion"], safety_all["safety"]
    scan, proximity = safety_all["pointcloud_to_laserscan"], dict(safety_all["proximity_stop"])
    proximity.update(launch_literals(launch, "proximity"))
    planner = launch_literals(launch, "replanner_parameters")
    frequency = launch_default(launch, "local_replanner_frequency_hz")
    route = load_yaml(candidate / "routes/indoor_manual_six_anchor_v3_radius120_050straight_030turn_loop.yaml")
    trajectory = load_yaml(candidate / "artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml")
    points = trajectory["points"]
    refs = {p["ref_id"]: i for i, p in enumerate(points) if p.get("ref_id")}
    zones = []
    for segment in route["route_speed_profile"]["segments"]:
        start, end = refs[segment["start_ref"]], refs[segment["end_ref"]]
        zones.append({"id": segment["id"], "start": start, "end": end,
                      "route_cap_mps": segment["max_speed_mps"],
                      "actual_reference_max_mps": max(p["v"] for p in points[start:end + 1])})
    accel = min(motion["max_acceleration_mps2"], safety["max_acceleration_mps2"])
    decel = min(motion["max_deceleration_mps2"], safety["max_deceleration_mps2"])
    caps = {
        "controller": motion["max_speed_mps"], "safety": safety["max_speed_mps"],
        "offline_optimizer": optimizers["continuous_trajectory_optimizer"]["max_speed_mps"],
        "online_optimizer": optimizers["trajectory_optimizer"]["max_speed_mps"],
        "route_default": route["route_speed_profile"]["default_max_speed_mps"],
        "frozen_reference_max": max(p["v"] for p in points),
        "path_guard_launch_source": motion["max_speed_mps"],
    }
    lateral = optimizers["continuous_trajectory_optimizer"]["max_lateral_acceleration_mps2"]
    # This sum is a design scenario, not a measured end-to-end latency or bound:
    # one planner period + configured search budget + command timeout. Sensor age,
    # callback queueing, network/actuator delay, jerk and braking uncertainty remain.
    latency_scenario = 1 / frequency + planner["planning_timeout_s"] + safety["command_timeout_s"]
    return {
        "status": "LOCAL_DESIGN_AUDIT_ONLY_FIELD_PENDING",
        "baseline": {"id": profile["id"], "launch": profile["launch"],
                     "manifest_file_count": len(manifest["files"]),
                     "manifest_matches": not mismatches, "mismatches": mismatches},
        "scope": "Pinned local candidate; runtime parameters and hardware limits not verified. No motion or runtime writes.",
        "speed_caps_mps": caps,
        "reference": {"point_count": len(points), "path_length_m": trajectory["path_length_m"],
                      "duration_s": trajectory["duration_s"], "turn_zones": zones},
        "response": {"acceleration_mps2": accel, "deceleration_mps2": decel,
                     "jerk_mps3": motion["max_jerk_mps3"],
                     "recovery_speed_mps": safety["recovery_speed_mps"],
                     "ideal_zero_to_1mps_s_excluding_jerk": 1 / accel},
        "braking_scenarios": {
            "latency_scenario_s": latency_scenario,
            "latency_terms": "1/planner_hz + planning_timeout_s + command_timeout_s; NOT measured and NOT a complete bound",
            "lookahead_m": planner["lookahead_distance_m"],
            "rows": [{"speed_mps": v, "ideal_braking_m": braking_distance(v, decel),
                      "with_latency_scenario_m": braking_distance(v, decel, latency_scenario),
                      "exceeds_nominal_lookahead": braking_distance(v, decel, latency_scenario) > planner["lookahead_distance_m"]}
                     for v in (0.5, 0.6, 0.8, 1.0)],
            "limitations": "Available collision-checked distance may be shorter than nominal lookahead; use measured stopping envelope and swept vehicle footprint.",
        },
        "turn_design_at_radius_1p2m": {
            "radius_is_design_example_not_entire_path": True,
            "current_lateral_acceleration_limit_mps2": lateral,
            "lateral_limit_only_speed_mps": math.sqrt(lateral * 1.2),
            "rows": [{"speed_mps": v, "required_lateral_acceleration_mps2": v * v / 1.2}
                     for v in (0.3, 0.5, 0.6, 1.0)],
        },
        "obstacle_model": {
            "grid_inflation_m": proximity["grid_inflation_radius_m"],
            "planner_additional_inflation_m": planner["inflation_radius_m"],
            "tracking_reserve_m": planner["tracking_clearance_m"],
            "configured_body_length_m": proximity["vehicle_length_m"],
            "configured_body_width_m": proximity["vehicle_width_m"],
            "centered_rectangle_corner_radius_m": math.hypot(proximity["vehicle_length_m"] / 2, proximity["vehicle_width_m"] / 2),
            "body_reference_and_real_outline_pending_measurement": True,
            "scan_angle_bin_rad": scan["angle_increment"],
            "bin_arc_spacing_m": {str(r): r * scan["angle_increment"] for r in (2, 3, 4)},
            "hard_stop_min_points": proximity["min_points"],
            "hard_stop_forward_x_max_m": proximity["stop_distance_m"],
            "hard_stop_geometry": "union of front sector and body corridor; not a uniform radial threshold",
            "grid_single_valid_return_can_mark_occupied": True,
            "perception_cause_of_close_stool_pass": "unconfirmed; no contact per latest user clarification",
        },
        "required_before_1mps_field_acceptance": [
            "Re-time reference and align all speed/acceleration/jerk/route/guard limits; changing controller max alone is insufficient.",
            "Measure end-to-end command/steering response, stopping distance and latency with the actual load and floor.",
            "Apply swept footprint and speed-dependent stopping budget to every accepted path, including normal complete paths.",
            "Measure stool detections and physical clearance; minimum-point hard stop is separate from grid occupancy.",
            "Separate pose/TF drift, actual vehicle error and path replacement effects in synchronized records.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, default=ROOT / "candidates/startup_capture_20261104")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    # Reject output anywhere inside the input baseline, even when explicitly set.
    if args.output and args.output.resolve().is_relative_to(args.candidate.resolve()):
        parser.error("Output must be outside the pinned candidate")
    result = audit(args.candidate)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result["baseline"]["manifest_matches"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
