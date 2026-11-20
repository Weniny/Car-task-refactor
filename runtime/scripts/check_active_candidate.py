#!/usr/bin/env python3
"""Read ROS parameters to verify that the running navigation uses this workspace."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess

import yaml


def expected_node_parameters(root: Path) -> dict:
    control = yaml.safe_load((root / "config/control/control_params_rebuild_050straight_030turn.yaml").read_text())
    safety = yaml.safe_load((root / "config/safety/safety_params_rebuild_050straight_030turn.yaml").read_text())["safety"]
    motion = control["motion"]
    trajectory = str(root / "artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml")
    return {
        "/rebuild_replan_mppi": {
            "trajectory_file": trajectory,
            "route_file": str(root / "routes/indoor_manual_six_anchor_v3_radius120_050straight_030turn_loop.yaml"),
            "optimizer_params_file": str(root / "config/planning/continuous_trajectory_radius120_050straight_030turn.yaml"),
            "max_speed_mps": motion["max_speed_mps"],
            "max_acceleration_mps2": motion["max_acceleration_mps2"],
            "max_deceleration_mps2": motion["max_deceleration_mps2"],
            "max_jerk_mps3": motion["max_jerk_mps3"],
        },
        "/rebuild_local_replanner": {
            "trajectory_file": trajectory,
            "map_file": str(root / "maps/indoor_manual_final/map.yaml"),
            "guard_max_speed_mps": motion["max_speed_mps"],
            "guard_deceleration_mps2": min(motion["max_deceleration_mps2"], safety["max_deceleration_mps2"]),
        },
        "/rebuild_replan_safety": {
            name: safety[name]
            for name in (
                "max_speed_mps", "max_acceleration_mps2", "max_deceleration_mps2",
                "recovery_speed_mps", "recovery_lateral_error_m", "recovery_heading_error_deg",
            )
        },
    }


def parameter_differences(expected: dict, actual: dict) -> list[dict]:
    return [
        {"node": node, "parameter": name, "expected": value, "actual": actual.get(node, {}).get(name)}
        for node, values in expected.items()
        for name, value in values.items()
        if actual.get(node, {}).get(name) != value
    ]


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    expected = expected_node_parameters(root)
    actual = {}
    for node in expected:
        try:
            result = subprocess.run(
                ["ros2", "param", "dump", node],
                check=True, capture_output=True, text=True, timeout=20,
            )
            payload = yaml.safe_load(result.stdout)
            actual[node] = payload[node]["ros__parameters"]
        except (subprocess.SubprocessError, OSError, ValueError, KeyError, TypeError) as error:
            raise SystemExit(f"ACTIVE CANDIDATE FAILED: cannot read {node}: {error}") from error
    differences = parameter_differences(expected, actual)
    if differences:
        print(json.dumps({"status": "ACTIVE CANDIDATE FAILED", "differences": differences}, indent=2))
        raise SystemExit(1)
    print(json.dumps({"status": "ACTIVE CANDIDATE PASSED", "workspace": str(root)}))


if __name__ == "__main__":
    main()
