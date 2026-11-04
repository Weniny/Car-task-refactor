#!/usr/bin/env python3
"""Generate the Day 5 whole-line control trajectory without moving the car."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Sequence

import yaml

from competition_planning.artifact_provenance import (
    build_source_manifest,
    resolve_trajectory_source_paths,
)
from competition_planning.semantic_planner import load_yaml_file
from competition_planning.trajectory_parameterizer import (
    ContinuousRouteTrajectory,
    optimize_continuous_route_trajectory,
)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True)
    parser.add_argument("--semantic-map", required=True)
    parser.add_argument("--planning-params", required=True)
    parser.add_argument("--optimizer-params", required=True)
    parser.add_argument("--end-ref")
    parser.add_argument("--end-ref-occurrence", type=int, default=1)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report")
    args = parser.parse_args(argv)

    result = optimize_continuous_route_trajectory(
        load_yaml_file(args.route),
        load_yaml_file(args.semantic_map),
        load_yaml_file(args.planning_params),
        load_yaml_file(args.optimizer_params),
        end_ref=args.end_ref,
        end_ref_occurrence=args.end_ref_occurrence,
    )
    artifact = result.to_dict()
    artifact["source_manifest"] = build_source_manifest(
        resolve_trajectory_source_paths(
            route_file=args.route,
            semantic_map_file=args.semantic_map,
            planning_params_file=args.planning_params,
            optimizer_params_file=args.optimizer_params,
        )
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(artifact, stream, sort_keys=False, allow_unicode=True)
    if args.report:
        write_continuous_trajectory_report(result, Path(args.report))

    print(
        f"route={result.route_name} ok={result.ok} points={len(result.points)} "
        f"length_m={result.path_length_m:.3f} duration_s={result.duration_s:.3f}"
    )
    for failure in result.failures:
        print(f"FAIL {failure.reason}: {failure.detail}")
    return 0 if result.ok else 2


def write_continuous_trajectory_report(
    result: ContinuousRouteTrajectory,
    path: Path,
) -> None:
    max_speed = max((point.v for point in result.points), default=0.0)
    max_acceleration = max((abs(point.a) for point in result.points), default=0.0)
    max_jerk = max((abs(point.jerk) for point in result.points), default=0.0)
    max_curvature = max((abs(point.curvature) for point in result.points), default=0.0)
    max_curvature_rate = max(
        (
            abs(current.curvature - previous.curvature) / (current.t - previous.t)
            for previous, current in zip(result.points, result.points[1:])
        ),
        default=0.0,
    )
    stopped_refs = [point.ref_id for point in result.points[1:] if point.v == 0.0]
    lines = [
        "# Day 5 连续控制轨迹摘要",
        "",
        "## 事实",
        "",
        f"- route_name: {result.route_name}",
        f"- frame_id: {result.frame_id}",
        f"- ok: {result.ok}",
        f"- planner_plugin: {result.planner_plugin}",
        f"- optimizer_plugin: {result.optimizer_plugin}",
        f"- point_count: {len(result.points)}",
        f"- path_length_m: {result.path_length_m:.3f}",
        f"- duration_s: {result.duration_s:.3f}",
        f"- max_speed_mps: {max_speed:.3f}",
        f"- max_abs_acceleration_mps2: {max_acceleration:.3f}",
        f"- max_abs_jerk_mps3: {max_jerk:.3f}",
        f"- max_abs_curvature_1pm: {max_curvature:.6f}",
        f"- max_abs_curvature_rate_1pmps: {max_curvature_rate:.6f}",
        f"- stopped_refs_after_start: {stopped_refs}",
        "",
        "## 未验证",
        "",
        "- 该报告是离线生成结果，不代表 ROS2 在线跟踪或实车运动已通过。",
    ]
    if result.failures:
        lines.extend(["", "## 失败", ""])
        lines.extend(f"- {failure.reason}: {failure.detail}" for failure in result.failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
