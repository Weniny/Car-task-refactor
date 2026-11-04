#!/usr/bin/env python3
"""Retime a frozen Day 5 route without replanning its geometry."""

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
from competition_planning.offline_continuous_trajectory import (
    write_continuous_trajectory_report,
)
from competition_planning.semantic_planner import load_yaml_file
from competition_planning.trajectory_parameterizer import (
    retime_continuous_trajectory,
)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-trajectory", required=True)
    parser.add_argument("--route", required=True)
    parser.add_argument("--semantic-map", required=True)
    parser.add_argument("--planning-params", required=True)
    parser.add_argument("--optimizer-params", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report")
    args = parser.parse_args(argv)

    reference_path = Path(args.reference_trajectory)
    result = retime_continuous_trajectory(
        load_yaml_file(reference_path),
        load_yaml_file(args.optimizer_params),
    )
    artifact = result.to_dict()
    sources = resolve_trajectory_source_paths(
        route_file=args.route,
        semantic_map_file=args.semantic_map,
        planning_params_file=args.planning_params,
        optimizer_params_file=args.optimizer_params,
    )
    sources["reference_trajectory"] = reference_path.resolve()
    artifact["source_manifest"] = build_source_manifest(sources)

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
    return 0


if __name__ == "__main__":
    sys.exit(main())
