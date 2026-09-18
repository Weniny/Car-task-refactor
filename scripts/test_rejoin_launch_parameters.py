"""Evaluate launch wiring without executing any nodes or connecting hardware."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[1] / "src/rebuild_bringup/launch"


def description(filename):
    spec = importlib.util.spec_from_file_location("launch_under_test", ROOT / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def defaults(context, launch):
    for entity in launch.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name not in context.launch_configurations:
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value
            )


def main():
    full = description("rebuild_full_navigation_050_trial.launch.py")
    preset = description("rebuild_full_navigation_050_rejoin4m_candidate.launch.py")
    competition = description(
        "rebuild_full_navigation_050_competition_stop_candidate.launch.py"
    )
    for mode in ("legacy", "path_aware", "rejoin4m", "competition_stop"):
        guarded = description("rebuild_navigation_replan_radius120_050mps_guarded.launch.py")
        context = LaunchContext()
        if mode in ("rejoin4m", "competition_stop"):
            candidate = preset if mode == "rejoin4m" else competition
            defaults(context, candidate)
            include = next(
                e for e in candidate.entities if isinstance(e, IncludeLaunchDescription)
            )
            for name, value in include.launch_arguments:
                context.launch_configurations[name] = perform_substitutions(
                    context, normalize_to_list_of_substitutions(value)
                )
        elif mode == "path_aware":
            context.launch_configurations["path_aware_stop_enabled"] = "true"
        defaults(context, full)
        include = next(e for e in full.entities if isinstance(e, IncludeLaunchDescription))
        for name, value in include.launch_arguments:
            context.launch_configurations[name] = perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )
        defaults(context, guarded)
        parameters = [
            p for e in guarded.entities
            for p in evaluate_parameters(context, getattr(e, "_Node__parameters", []))
            if isinstance(p, dict)
        ]
        planner = next(p for p in parameters if "lookahead_distance_m" in p)
        enabled = mode in ("path_aware", "rejoin4m")
        assert planner["costmap_occupancy_threshold"] == (100 if enabled else 50)
        assert planner["inflation_radius_m"] == (0.75 if enabled else 0.04)
        assert planner["minimum_short_rejoin_distance_m"] == (1.8 if enabled else 0.0)
        assert planner["lookahead_distance_m"] == (
            4.0 if mode in ("rejoin4m", "competition_stop") else 3.0
        )
        assert planner["obstacle_aware_heuristic"] == (
            mode in ("rejoin4m", "competition_stop")
        )
        if mode in ("rejoin4m", "competition_stop"):
            assert planner["obstacle_x_max_m"] == 5.5
            assert planner["planning_timeout_s"] == 0.6
            assert planner["empty_history_planning_timeout_s"] == 0.6
            assert planner["max_obstacle_age_s"] == 0.9
            assert float(planner["frequency_hz"]) == 2.0
            assert context.launch_configurations["start_chassis_adapter"] == "false"
        if mode == "competition_stop":
            proximity = next(p for p in parameters if "stop_distance_m" in p)
            safety = next(p for p in parameters if "require_avoidance_speed_limit" in p)
            assert proximity["stop_distance_m"] == 0.0
            assert proximity["path_aware_stop_enabled"] is False
            assert safety["require_avoidance_speed_limit"] is False
        print(f"PASS: {mode} launch parameter wiring (no nodes executed)")


if __name__ == "__main__":
    main()
