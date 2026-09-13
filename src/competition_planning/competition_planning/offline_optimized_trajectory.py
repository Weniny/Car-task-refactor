#!/usr/bin/env python3
"""Offline entry point for Day 4 optimized trajectory generation."""

from __future__ import annotations

import argparse
import csv
import html
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
    OptimizedRouteTrajectory,
    optimize_route_trajectory,
)


CSV_FIELDS = ("x", "y", "yaw", "s", "curvature", "v", "yaw_rate", "t", "ref_id")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True, help="Route YAML file.")
    parser.add_argument("--semantic-map", required=True, help="Semantic map YAML file.")
    parser.add_argument("--planning-params", required=True, help="Planning params YAML file.")
    parser.add_argument("--optimizer-params", required=True, help="Optimizer params YAML file.")
    parser.add_argument("--output", help="Optional output YAML artifact.")
    parser.add_argument("--csv-dir", help="Optional directory for per-step CSV files.")
    parser.add_argument("--report", help="Optional Markdown summary artifact.")
    parser.add_argument("--svg", help="Optional SVG overview artifact.")
    parser.add_argument("--step-id", action="append", help="Emit only the selected step id.")
    args = parser.parse_args(argv)

    route = load_yaml_file(args.route)
    semantic_map = load_yaml_file(args.semantic_map)
    planning_params = load_yaml_file(args.planning_params)
    optimizer_params = load_yaml_file(args.optimizer_params)

    result = optimize_route_trajectory(route, semantic_map, planning_params, optimizer_params)
    selected_result = _select_steps(result, args.step_id)
    output = selected_result.to_dict()
    output["source_manifest"] = build_source_manifest(
        resolve_trajectory_source_paths(
            route_file=args.route,
            semantic_map_file=args.semantic_map,
            planning_params_file=args.planning_params,
            optimizer_params_file=args.optimizer_params,
        )
    )

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False, allow_unicode=True)

    if args.csv_dir:
        _write_csv_files(selected_result, Path(args.csv_dir))

    if args.report:
        _write_report(selected_result, optimizer_params, Path(args.report))

    if args.svg:
        _write_svg(selected_result, Path(args.svg))

    print(
        f"route={output['route_name']} ok={output['ok']} "
        f"trajectories={len(output['trajectories'])} failures={len(output['failures'])}"
    )
    for trajectory in output["trajectories"]:
        print(
            f"TRAJECTORY {trajectory['step_id']} target={trajectory['target_ref']} "
            f"points={trajectory['point_count']} length_m={trajectory['path_length_m']} "
            f"duration_s={trajectory['duration_s']}"
        )
    for failure in output["failures"]:
        print(
            f"FAIL {failure['step_id']} reason={failure['reason']} "
            f"detail={failure['detail']}"
        )

    return 0 if output["ok"] else 2


def _select_steps(
    result: OptimizedRouteTrajectory,
    selected_step_ids: Sequence[str] | None,
) -> OptimizedRouteTrajectory:
    if not selected_step_ids:
        return result
    selected = set(selected_step_ids)
    return OptimizedRouteTrajectory(
        frame_id=result.frame_id,
        route_name=result.route_name,
        trajectories=tuple(
            trajectory for trajectory in result.trajectories if trajectory.step_id in selected
        ),
        failures=tuple(failure for failure in result.failures if failure.step_id in selected),
    )


def _write_csv_files(result: OptimizedRouteTrajectory, csv_dir: Path) -> None:
    csv_dir.mkdir(parents=True, exist_ok=True)
    for trajectory in result.trajectories:
        csv_path = csv_dir / f"{trajectory.step_id}.csv"
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator="\n")
            writer.writeheader()
            for point in trajectory.points:
                row = point.to_dict()
                row["ref_id"] = row.get("ref_id", "")
                writer.writerow(row)


def _write_report(
    result: OptimizedRouteTrajectory,
    optimizer_params: dict,
    report_path: Path,
) -> None:
    params = optimizer_params.get("trajectory_optimizer", {})
    lines = [
        "# Day 4 优化轨迹摘要",
        "",
        "## 事实",
        "",
        f"- route_name: {result.route_name}",
        f"- frame_id: {result.frame_id}",
        f"- ok: {result.ok}",
        f"- trajectory_count: {len(result.trajectories)}",
        f"- failure_count: {len(result.failures)}",
        "",
        "## 配置限值",
        "",
        f"- max_speed_mps: {params.get('max_speed_mps')}",
        f"- max_acceleration_mps2: {params.get('max_acceleration_mps2')}",
        f"- max_deceleration_mps2: {params.get('max_deceleration_mps2')}",
        f"- max_lateral_acceleration_mps2: {params.get('max_lateral_acceleration_mps2')}",
        f"- obstacle_zone_speed_limits_mps: {params.get('obstacle_zone_speed_limits_mps')}",
        "",
        "## 分段摘要",
        "",
        "| step_id | points | length_m | duration_s | max_v_mps | max_abs_curvature |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for trajectory in result.trajectories:
        max_v = max((point.v for point in trajectory.points), default=0.0)
        max_abs_curvature = max(
            (abs(point.curvature) for point in trajectory.points),
            default=0.0,
        )
        lines.append(
            f"| {trajectory.step_id} | {len(trajectory.points)} | "
            f"{trajectory.path_length_m:.3f} | {trajectory.duration_s:.3f} | "
            f"{max_v:.3f} | {max_abs_curvature:.6f} |"
        )
    lines.extend(
        [
            "",
            "## 未验证",
            "",
            "- 当前只是离线 artifact；ROS2 发布、tracker 行为和实车运动尚未验证。",
        ]
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_svg(result: OptimizedRouteTrajectory, svg_path: Path) -> None:
    points = [point for trajectory in result.trajectories for point in trajectory.points]
    if not points:
        svg_path.parent.mkdir(parents=True, exist_ok=True)
        svg_path.write_text('<svg xmlns="http://www.w3.org/2000/svg"></svg>\n', encoding="utf-8")
        return

    width = 900.0
    height = 600.0
    margin = 40.0
    min_x = min(point.x for point in points)
    max_x = max(point.x for point in points)
    min_y = min(point.y for point in points)
    max_y = max(point.y for point in points)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    scale = min((width - margin * 2.0) / span_x, (height - margin * 2.0) / span_y)

    def project(x: float, y: float) -> tuple[float, float]:
        return (
            margin + (x - min_x) * scale,
            height - margin - (y - min_y) * scale,
        )

    colors = ["#2563eb", "#16a34a", "#dc2626", "#9333ea", "#ea580c", "#0891b2"]
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(width)}" height="{int(height)}" viewBox="0 0 {int(width)} {int(height)}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
    ]
    for index, trajectory in enumerate(result.trajectories):
        color = colors[index % len(colors)]
        polyline = " ".join(
            f"{project(point.x, point.y)[0]:.1f},{project(point.x, point.y)[1]:.1f}"
            for point in trajectory.points
        )
        lines.append(
            f'<polyline points="{polyline}" fill="none" stroke="{color}" '
            'stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>'
        )
        for point in trajectory.points:
            if not point.ref_id:
                continue
            x, y = project(point.x, point.y)
            label = html.escape(point.ref_id)
            lines.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{color}"/>')
            lines.append(
                f'<text x="{x + 6.0:.1f}" y="{y - 6.0:.1f}" '
                'font-family="Arial, sans-serif" font-size="11" fill="#111827">'
                f"{label}</text>"
            )
    lines.append("</svg>")
    svg_path.parent.mkdir(parents=True, exist_ok=True)
    svg_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
