#!/usr/bin/env python3
"""Offline entry point for semantic corridor global planning."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

from competition_planning.semantic_planner import load_yaml_file, plan_route


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True, help="Route YAML file.")
    parser.add_argument("--semantic-map", required=True, help="Semantic map YAML file.")
    parser.add_argument("--params", help="Planning params YAML file.")
    parser.add_argument("--output", help="Optional output YAML artifact.")
    parser.add_argument("--step-id", action="append", help="Print only the selected step id.")
    args = parser.parse_args()

    route = load_yaml_file(args.route)
    semantic_map = load_yaml_file(args.semantic_map)
    params = load_yaml_file(args.params) if args.params else {}
    result = plan_route(route, semantic_map, params)
    output = result.to_dict()

    if args.step_id:
        selected = set(args.step_id)
        output["plans"] = [
            plan for plan in output["plans"] if str(plan.get("step_id")) in selected
        ]
        output["failures"] = [
            failure
            for failure in output["failures"]
            if str(failure.get("step_id")) in selected
        ]
        output["ok"] = not output["failures"]

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False, allow_unicode=True)

    print(
        f"route={output['route_name']} ok={output['ok']} "
        f"plans={len(output['plans'])} failures={len(output['failures'])}"
    )
    for plan in output["plans"]:
        print(
            f"PLAN {plan['step_id']} target={plan['target_ref']} "
            f"points={plan['point_count']} length_m={plan['path_length_m']} "
            f"time_ms={plan['planning_time_ms']}"
        )
    for failure in output["failures"]:
        print(
            f"FAIL {failure['step_id']} reason={failure['reason']} "
            f"detail={failure['detail']}"
        )

    return 0 if output["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
