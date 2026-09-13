"""Deterministic global planning inside semantic route corridors."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any

import yaml


PLANNABLE_STEP_TYPES = {"RUN_SEGMENT", "CONE_LANE_CHANGE", "FINISH_PARK"}


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    yaw: float
    ref_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 4),
        }
        if self.ref_id:
            result["ref_id"] = self.ref_id
        return result


@dataclass(frozen=True)
class StepPlan:
    step_id: str
    step_type: str
    corridor_ref: str
    target_ref: str
    target_source: str
    path: tuple[PathPoint, ...]
    planning_time_ms: float
    planner_plugin: str = "semantic_corridor"
    smoother_plugin: str = "none"

    @property
    def path_length_m(self) -> float:
        total = 0.0
        for previous, current in zip(self.path, self.path[1:]):
            total += math.hypot(current.x - previous.x, current.y - previous.y)
        return total

    def to_dict(self) -> dict[str, Any]:
        return {
            "step_id": self.step_id,
            "type": self.step_type,
            "corridor_ref": self.corridor_ref,
            "target_ref": self.target_ref,
            "target_source": self.target_source,
            "planner_plugin": self.planner_plugin,
            "smoother_plugin": self.smoother_plugin,
            "point_count": len(self.path),
            "path_length_m": round(self.path_length_m, 3),
            "planning_time_ms": round(self.planning_time_ms, 3),
            "points": [point.to_dict() for point in self.path],
        }


@dataclass(frozen=True)
class PlanFailure:
    step_id: str
    step_type: str
    reason: str
    detail: str

    def to_dict(self) -> dict[str, str]:
        return {
            "step_id": self.step_id,
            "type": self.step_type,
            "reason": self.reason,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class RoutePlan:
    frame_id: str
    route_name: str
    plans: tuple[StepPlan, ...]
    failures: tuple[PlanFailure, ...]

    @property
    def ok(self) -> bool:
        return not self.failures

    def to_dict(self) -> dict[str, Any]:
        return {
            "route_name": self.route_name,
            "frame_id": self.frame_id,
            "ok": self.ok,
            "plans": [plan.to_dict() for plan in self.plans],
            "failures": [failure.to_dict() for failure in self.failures],
        }


@dataclass(frozen=True)
class ContinuousRoutePlan:
    frame_id: str
    route_name: str
    path: tuple[PathPoint, ...]
    planner_plugin: str
    planning_time_ms: float
    failures: tuple[PlanFailure, ...]

    @property
    def ok(self) -> bool:
        return not self.failures

    def to_dict(self) -> dict[str, Any]:
        return {
            "route_name": self.route_name,
            "frame_id": self.frame_id,
            "ok": self.ok,
            "planner_plugin": self.planner_plugin,
            "planning_time_ms": round(self.planning_time_ms, 3),
            "point_count": len(self.path),
            "points": [point.to_dict() for point in self.path],
            "failures": [failure.to_dict() for failure in self.failures],
        }


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data


def plan_route(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    planning_config: dict[str, Any] | None = None,
) -> RoutePlan:
    """Plan every semantic route step that owns a corridor.

    The interface intentionally accepts parsed dictionaries so tests, CLIs, and
    ROS adapters can all use the same implementation.
    """

    planning_config = planning_config or {}
    params = planning_config.get("global_planner", {})
    timeout_ms = float(params.get("planning_timeout_ms", 500.0))
    margin_m = _corridor_margin_m(params)
    min_turning_radius_m = float(params.get("min_turning_radius_m", 0.0))
    wide_turn_min_turning_radius_m = float(
        params.get("wide_turn_min_turning_radius_m", min_turning_radius_m)
    )
    dock_approach_min_turning_radius_m = float(
        params.get("dock_approach_min_turning_radius_m", min_turning_radius_m)
    )
    sample_spacing_m = float(params.get("path_sample_spacing_m", 0.25))
    planner_plugin = str(params.get("plugin", "semantic_corridor"))
    fallback_plugin = str(params.get("fallback_plugin", ""))
    smoother = _load_smoother(planning_config)

    map_model = _SemanticMap(semantic_map)
    grid_planner = None
    wide_turn_grid_planner = None
    dock_approach_grid_planner = None
    precision_goal_grid_planner = None
    grid_planner_failure: str | None = None
    if planner_plugin in {"occupancy_grid_astar", "hybrid_astar"}:
        try:
            grid_planner = (
                _load_hybrid_planner(semantic_map, params)
                if planner_plugin == "hybrid_astar"
                else _load_grid_planner(semantic_map, params)
            )
            if (
                planner_plugin == "hybrid_astar"
                and "dock_approach_min_turning_radius_m" in params
            ):
                dock_params = dict(params)
                dock_params["min_turning_radius_m"] = params[
                    "dock_approach_min_turning_radius_m"
                ]
                if "dock_approach_goal_lateral_tolerance_m" in params:
                    dock_params["hybrid_goal_lateral_tolerance_m"] = params[
                        "dock_approach_goal_lateral_tolerance_m"
                    ]
                if "dock_approach_exact_goal_connection" in params:
                    dock_params["hybrid_exact_goal_connection"] = params[
                        "dock_approach_exact_goal_connection"
                    ]
                dock_approach_grid_planner = _load_hybrid_planner(
                    semantic_map,
                    dock_params,
                )
            if (
                planner_plugin == "hybrid_astar"
                and wide_turn_min_turning_radius_m > min_turning_radius_m
            ):
                wide_turn_params = dict(params)
                wide_turn_params["min_turning_radius_m"] = (
                    wide_turn_min_turning_radius_m
                )
                wide_turn_grid_planner = _load_hybrid_planner(
                    semantic_map,
                    wide_turn_params,
                )
            if planner_plugin == "hybrid_astar":
                precision_params = dict(params)
                precision_params["hybrid_goal_lateral_tolerance_m"] = params.get(
                    "precision_goal_lateral_tolerance_m",
                    params.get("dock_approach_goal_lateral_tolerance_m", 0.03),
                )
                precision_params[
                    "hybrid_alignment_waypoint_position_tolerance_m"
                ] = params.get(
                    "precision_alignment_waypoint_position_tolerance_m",
                    params.get("dock_approach_goal_lateral_tolerance_m", 0.03),
                )
                precision_params["hybrid_goal_position_tolerance_m"] = params.get(
                    "precision_alignment_waypoint_position_tolerance_m",
                    params.get("dock_approach_goal_lateral_tolerance_m", 0.03),
                )
                precision_params["hybrid_exact_goal_connection"] = params.get(
                    "precision_goal_exact_connection",
                    params.get("dock_approach_exact_goal_connection", True),
                )
                precision_goal_grid_planner = _load_hybrid_planner(
                    semantic_map,
                    precision_params,
                )
        except Exception as exc:
            grid_planner_failure = str(exc)
    current_ref = _initial_current_ref(route, map_model)
    plans: list[StepPlan] = []
    failures: list[PlanFailure] = []

    for step in route.get("steps", []):
        if not isinstance(step, dict):
            continue

        step_id = str(step.get("id", ""))
        step_type = str(step.get("type", ""))
        if step_type not in PLANNABLE_STEP_TYPES:
            current_ref = _pose_ref_from_step(step) or current_ref
            continue

        started_at = time.perf_counter()
        plan, failure, next_ref = _plan_step(
            step=step,
            current_ref=current_ref,
            map_model=map_model,
            margin_m=margin_m,
            min_turning_radius_m=min_turning_radius_m,
            wide_turn_min_turning_radius_m=wide_turn_min_turning_radius_m,
            dock_approach_min_turning_radius_m=(
                dock_approach_min_turning_radius_m
            ),
            sample_spacing_m=sample_spacing_m,
            planner_plugin=planner_plugin,
            fallback_plugin=fallback_plugin,
            grid_planner=grid_planner,
            wide_turn_grid_planner=wide_turn_grid_planner,
            dock_approach_grid_planner=dock_approach_grid_planner,
            precision_goal_grid_planner=precision_goal_grid_planner,
            grid_planner_failure=grid_planner_failure,
            smoother=smoother,
        )
        elapsed_ms = (time.perf_counter() - started_at) * 1000.0

        if plan is not None and elapsed_ms > timeout_ms:
            failure = PlanFailure(
                step_id=plan.step_id,
                step_type=plan.step_type,
                reason="planning_timeout",
                detail=f"planning took {elapsed_ms:.3f} ms; limit is {timeout_ms:.3f} ms",
            )
            plan = None
            next_ref = current_ref

        if plan is not None:
            plans.append(
                StepPlan(
                    step_id=plan.step_id,
                    step_type=plan.step_type,
                    corridor_ref=plan.corridor_ref,
                    target_ref=plan.target_ref,
                    target_source=plan.target_source,
                    path=plan.path,
                    planning_time_ms=elapsed_ms,
                    planner_plugin=plan.planner_plugin,
                    smoother_plugin=plan.smoother_plugin,
                )
            )
            current_ref = next_ref
        elif failure is not None:
            failures.append(failure)

    return RoutePlan(
        frame_id=str(semantic_map.get("frame_id", "map")),
        route_name=str(route.get("route_name", "")),
        plans=tuple(plans),
        failures=tuple(failures),
    )


def plan_continuous_route(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    planning_config: dict[str, Any] | None = None,
) -> ContinuousRoutePlan:
    """Plan every motion step as one continuous pose-constrained path.

    This seam is for whole-line control validation. It deliberately ignores
    non-motion task stops while preserving the ordered semantic waypoints. The
    mission planner continues to use :func:`plan_route` for per-step plans.
    """

    planning_config = planning_config or {}
    params = planning_config.get("global_planner", {})
    planner_plugin = str(params.get("plugin", ""))
    frame_id = str(semantic_map.get("frame_id", "map"))
    route_name = str(route.get("route_name", ""))
    if planner_plugin not in {"hybrid_astar", "semantic_corridor"}:
        failure = PlanFailure(
            step_id="continuous_route",
            step_type="CONTROL_ROUTE",
            reason="continuous_route_requires_hybrid_astar",
            detail=f"configured global planner is {planner_plugin or 'unset'}",
        )
        return ContinuousRoutePlan(
            frame_id=frame_id,
            route_name=route_name,
            path=(),
            planner_plugin=planner_plugin,
            planning_time_ms=0.0,
            failures=(failure,),
        )

    map_model = _SemanticMap(semantic_map)
    refs, failures = _continuous_route_refs(route, map_model)
    if failures:
        return ContinuousRoutePlan(
            frame_id=frame_id,
            route_name=route_name,
            path=(),
            planner_plugin=planner_plugin,
            planning_time_ms=0.0,
            failures=tuple(failures),
        )
    waypoints = [map_model.point(ref) for ref in refs]
    if any(point is None for point in waypoints):
        failure = PlanFailure(
            step_id="continuous_route",
            step_type="CONTROL_ROUTE",
            reason="unknown_point_ref",
            detail="continuous route contains an unknown semantic point",
        )
        return ContinuousRoutePlan(
            frame_id=frame_id,
            route_name=route_name,
            path=(),
            planner_plugin=planner_plugin,
            planning_time_ms=0.0,
            failures=(failure,),
        )

    started_at = time.perf_counter()
    if planner_plugin == "semantic_corridor":
        semantic_route = plan_route(route, semantic_map, planning_config)
        if semantic_route.failures:
            return ContinuousRoutePlan(
                frame_id=frame_id,
                route_name=route_name,
                path=(),
                planner_plugin=planner_plugin,
                planning_time_ms=(time.perf_counter() - started_at) * 1000.0,
                failures=tuple(semantic_route.failures),
            )

        path = []
        for step_plan in semantic_route.plans:
            step_path = list(step_plan.path)
            if path and step_path:
                step_path = step_path[1:]
            path.extend(step_path)
    else:
        try:
            planner = _load_hybrid_planner(semantic_map, params)
            path = planner.plan([point for point in waypoints if point is not None])
        except Exception as exc:
            failure = PlanFailure(
                step_id="continuous_route",
                step_type="CONTROL_ROUTE",
                reason="hybrid_planning_failed",
                detail=str(exc),
            )
            return ContinuousRoutePlan(
                frame_id=frame_id,
                route_name=route_name,
                path=(),
                planner_plugin=planner_plugin,
                planning_time_ms=(time.perf_counter() - started_at) * 1000.0,
                failures=(failure,),
            )

    elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    timeout_ms = float(params.get("planning_timeout_ms", 500.0))
    validation_failure = _validate_path(
        {"id": "continuous_route", "type": "CONTROL_ROUTE"},
        path,
        {"width_m": 1_000_000.0, "points": refs},
        map_model,
        margin_m=0.0,
        min_turning_radius_m=float(params.get("min_turning_radius_m", 0.0)),
        check_corridor_distance=False,
    )
    failures_out: tuple[PlanFailure, ...] = ()
    if elapsed_ms > timeout_ms:
        failures_out = (
            PlanFailure(
                step_id="continuous_route",
                step_type="CONTROL_ROUTE",
                reason="planning_timeout",
                detail=f"planning took {elapsed_ms:.3f} ms; limit is {timeout_ms:.3f} ms",
            ),
        )
    elif validation_failure is not None:
        failures_out = (validation_failure,)

    return ContinuousRoutePlan(
        frame_id=frame_id,
        route_name=route_name,
        path=tuple(path),
        planner_plugin=planner_plugin,
        planning_time_ms=elapsed_ms,
        failures=failures_out,
    )


def _continuous_route_refs(
    route: dict[str, Any],
    map_model: "_SemanticMap",
) -> tuple[list[str], list[PlanFailure]]:
    current_ref = _initial_current_ref(route, map_model)
    refs = [current_ref] if current_ref else []
    failures: list[PlanFailure] = []
    for step in route.get("steps", []):
        if not isinstance(step, dict):
            continue
        if str(step.get("type", "")) not in PLANNABLE_STEP_TYPES:
            current_ref = _pose_ref_from_step(step) or current_ref
            continue
        corridor_ref = str(step.get("corridor_ref", ""))
        corridor = map_model.corridors.get(corridor_ref)
        if corridor is None:
            failures.append(_failure(step, "unknown_corridor", f"unknown corridor {corridor_ref}"))
            continue
        allowed_steps = corridor.get("allowed_steps", [])
        if allowed_steps and str(step.get("id", "")) not in allowed_steps:
            failures.append(
                _failure(
                    step,
                    "step_not_allowed_in_corridor",
                    f"{step.get('id', '')} is not listed in corridor {corridor_ref} allowed_steps",
                )
            )
            continue
        centerline_ref = str(corridor.get("centerline_ref", ""))
        centerline = map_model.centerlines.get(centerline_ref)
        if centerline is None:
            failures.append(
                _failure(step, "unknown_centerline", f"unknown centerline {centerline_ref}")
            )
            continue
        centerline_refs = [str(ref) for ref in centerline.get("points", [])]
        target_ref, _ = _target_ref_for_step(step, centerline_refs)
        if target_ref is None:
            failures.append(_failure(step, "missing_target_ref", "step has no usable target"))
            continue
        segment_refs = _segment_refs_between(centerline_refs, current_ref, target_ref)
        if not segment_refs:
            failures.append(
                _failure(
                    step,
                    "route_ref_not_on_centerline",
                    f"cannot connect {current_ref} to {target_ref} on {centerline_ref}",
                )
            )
            continue
        refs.extend(segment_refs[1:] if refs and refs[-1] == segment_refs[0] else segment_refs)
        current_ref = target_ref
    return [ref for ref in refs if ref is not None], failures


class _SemanticMap:
    def __init__(self, data: dict[str, Any]) -> None:
        self.points = data.get("points", {})
        self.effective_area = data.get("effective_area", {})
        self.no_go_zones = data.get("no_go_zones", [])
        self.centerlines = {
            str(item.get("id")): item for item in data.get("lane_centerlines", [])
        }
        self.corridors = {
            str(item.get("id")): item for item in data.get("route_corridors", [])
        }

    def point(self, ref: str) -> PathPoint | None:
        raw = self.points.get(ref)
        if not isinstance(raw, dict):
            return None
        try:
            return PathPoint(
                x=float(raw["x"]),
                y=float(raw["y"]),
                yaw=float(raw.get("yaw", 0.0)),
                ref_id=ref,
            )
        except (KeyError, TypeError, ValueError):
            return None

    def polygon(self, raw: Any) -> list[tuple[float, float]]:
        if not isinstance(raw, list):
            return []
        vertices: list[tuple[float, float]] = []
        for vertex in raw:
            if not isinstance(vertex, list | tuple) or len(vertex) < 2:
                return []
            vertices.append((float(vertex[0]), float(vertex[1])))
        return vertices


def _plan_step(
    step: dict[str, Any],
    current_ref: str | None,
    map_model: _SemanticMap,
    margin_m: float,
    min_turning_radius_m: float,
    wide_turn_min_turning_radius_m: float,
    dock_approach_min_turning_radius_m: float,
    sample_spacing_m: float,
    planner_plugin: str,
    fallback_plugin: str,
    grid_planner: Any,
    wide_turn_grid_planner: Any,
    dock_approach_grid_planner: Any,
    precision_goal_grid_planner: Any,
    grid_planner_failure: str | None,
    smoother: Any,
) -> tuple[StepPlan | None, PlanFailure | None, str | None]:
    step_id = str(step.get("id", ""))
    step_type = str(step.get("type", ""))
    corridor_ref = step.get("corridor_ref")
    if not corridor_ref:
        return None, _failure(step, "missing_corridor_ref", "step has no corridor_ref"), current_ref

    corridor = map_model.corridors.get(str(corridor_ref))
    if corridor is None:
        return None, _failure(step, "unknown_corridor", f"unknown corridor {corridor_ref}"), current_ref

    allowed_steps = corridor.get("allowed_steps", [])
    if allowed_steps and step_id not in allowed_steps:
        return (
            None,
            _failure(
                step,
                "step_not_allowed_in_corridor",
                f"{step_id} is not listed in corridor {corridor_ref} allowed_steps",
            ),
            current_ref,
        )

    centerline_ref = str(corridor.get("centerline_ref", ""))
    centerline = map_model.centerlines.get(centerline_ref)
    if centerline is None:
        return (
            None,
            _failure(step, "unknown_centerline", f"unknown centerline {centerline_ref}"),
            current_ref,
        )

    centerline_refs = [str(ref) for ref in centerline.get("points", [])]
    centerline_points = [map_model.point(ref) for ref in centerline_refs]
    if any(point is None for point in centerline_points):
        return (
            None,
            _failure(
                step,
                "unknown_point_ref",
                f"centerline {centerline_ref} contains an unknown point reference",
            ),
            current_ref,
        )

    start_ref = current_ref or (centerline_refs[0] if centerline_refs else None)
    target_ref, target_source = _target_ref_for_step(step, centerline_refs)
    if target_ref is None:
        return None, _failure(step, "missing_target_ref", "step has no usable target"), current_ref

    segment_refs = _segment_refs_between(centerline_refs, start_ref, target_ref)
    if not segment_refs:
        return (
            None,
            _failure(
                step,
                "route_ref_not_on_centerline",
                f"cannot connect {start_ref} to {target_ref} on {centerline_ref}",
            ),
            current_ref,
        )

    cone_ref_failure = _validate_cone_refs(step, segment_refs)
    if cone_ref_failure is not None:
        return None, cone_ref_failure, current_ref

    points = [map_model.point(ref) for ref in segment_refs]
    if any(point is None for point in points):
        return None, _failure(step, "unknown_point_ref", "segment contains unknown point"), current_ref

    is_dock_approach = bool(step.get("docking_approach", False))
    is_wide_turn = bool(step.get("wide_turn", False))
    semantic_points = [point for point in points if point]
    semantic_path = tuple(_interpolate_path(semantic_points, sample_spacing_m))
    path = semantic_path
    used_plugin = "semantic_corridor"
    if planner_plugin in {"occupancy_grid_astar", "hybrid_astar"}:
        if bool(step.get("precision_goal_connection", False)):
            active_grid_planner = precision_goal_grid_planner or grid_planner
        elif wide_turn_grid_planner is not None and is_wide_turn:
            active_grid_planner = wide_turn_grid_planner
        elif dock_approach_grid_planner is not None and is_dock_approach:
            active_grid_planner = dock_approach_grid_planner
        else:
            active_grid_planner = grid_planner
        if active_grid_planner is None:
            if fallback_plugin != "semantic_corridor":
                return (
                    None,
                    _failure(
                        step,
                        "grid_planner_unavailable",
                        grid_planner_failure or "occupancy-grid planner is unavailable",
                    ),
                    current_ref,
                )
        else:
            try:
                if (
                    planner_plugin == "hybrid_astar"
                    and bool(step.get("soft_intermediate_refs", False))
                ):
                    corridor_half_width_m = max(
                        sample_spacing_m,
                        0.5 * float(centerline.get("width_m", 0.0)),
                    )
                    path = active_grid_planner.plan(
                        semantic_points,
                        reference_path=semantic_path,
                        corridor_half_width_m=corridor_half_width_m,
                        soft_intermediate_waypoints=True,
                    )
                else:
                    path = active_grid_planner.plan(semantic_points)
                used_plugin = planner_plugin
            except Exception as exc:
                if fallback_plugin != "semantic_corridor":
                    return (
                        None,
                        _failure(step, "grid_planning_failed", str(exc)),
                        current_ref,
                    )

    smoother_plugin = "none"
    if smoother is not None:
        smoothing_input = _with_semantic_anchor_yaws(path, points)
        smoothed_path = smoother.smooth(smoothing_input)
        if smoothed_path == smoothing_input:
            smoother_plugin = "none_insufficient_anchors"
        elif grid_planner is not None and not grid_planner.path_is_navigable(smoothed_path):
            smoother_plugin = "none_collision_fallback"
        else:
            path = smoothed_path
            smoother_plugin = "cubic_bezier"

    validation_failure = _validate_path(
        step,
        path,
        centerline,
        map_model,
        margin_m,
        (
            wide_turn_min_turning_radius_m
            if is_wide_turn
            else (
                dock_approach_min_turning_radius_m
                if is_dock_approach
                else min_turning_radius_m
            )
        ),
        check_corridor_distance=used_plugin == "semantic_corridor",
        corridor_reference=(
            path if used_plugin == "semantic_corridor" else None
        ),
    )
    if validation_failure is not None:
        return None, validation_failure, current_ref

    plan = StepPlan(
        step_id=step_id,
        step_type=step_type,
        corridor_ref=str(corridor_ref),
        target_ref=target_ref,
        target_source=target_source,
        path=path,
        planning_time_ms=0.0,
        planner_plugin=used_plugin,
        smoother_plugin=smoother_plugin,
    )
    return plan, None, target_ref


def _target_ref_for_step(
    step: dict[str, Any],
    centerline_refs: list[str],
) -> tuple[str | None, str]:
    if step.get("target_ref"):
        return str(step["target_ref"]), "target_ref"
    if step.get("exit_ref"):
        candidate = str(step["exit_ref"])
        if candidate == centerline_refs[-1]:
            return candidate, "exit_ref"
        return centerline_refs[-1], "corridor_end"
    return None, "missing"


def _segment_refs_between(
    centerline_refs: list[str],
    start_ref: str | None,
    target_ref: str,
) -> list[str]:
    if start_ref not in centerline_refs or target_ref not in centerline_refs:
        return []
    start_index = centerline_refs.index(str(start_ref))
    target_index = centerline_refs.index(target_ref)
    if start_index <= target_index:
        return centerline_refs[start_index : target_index + 1]
    return list(reversed(centerline_refs[target_index : start_index + 1]))


def _validate_cone_refs(
    step: dict[str, Any],
    segment_refs: list[str],
) -> PlanFailure | None:
    if step.get("type") != "CONE_LANE_CHANGE":
        return None
    for key in ("entry_ref", "exit_ref"):
        ref = step.get(key)
        if ref and str(ref) not in segment_refs:
            return _failure(
                step,
                "cone_ref_not_on_segment",
                f"{key}={ref} is not on the planned corridor segment",
            )
    return None


def _interpolate_path(points: list[PathPoint], sample_spacing_m: float) -> list[PathPoint]:
    if len(points) < 2:
        return points

    spacing = max(0.05, sample_spacing_m)
    path: list[PathPoint] = []
    for start, end in zip(points, points[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if length > 0.0 else start.yaw
        samples = max(1, math.ceil(length / spacing))
        for index in range(samples + 1):
            if path and index == 0:
                continue
            ratio = index / samples
            ref_id = start.ref_id if index == 0 else end.ref_id if index == samples else None
            path.append(
                PathPoint(
                    x=start.x + dx * ratio,
                    y=start.y + dy * ratio,
                    yaw=yaw,
                    ref_id=ref_id,
                )
            )
    target = points[-1]
    path[-1] = PathPoint(x=target.x, y=target.y, yaw=target.yaw, ref_id=target.ref_id)
    return path


def _validate_path(
    step: dict[str, Any],
    path: tuple[PathPoint, ...],
    centerline: dict[str, Any],
    map_model: _SemanticMap,
    margin_m: float,
    min_turning_radius_m: float,
    check_corridor_distance: bool = True,
    corridor_reference: tuple[PathPoint, ...] | None = None,
) -> PlanFailure | None:
    width_m = float(centerline.get("width_m", 0.0))
    max_distance_m = width_m / 2.0 - margin_m
    if max_distance_m <= 0.0:
        return _failure(
            step,
            "corridor_too_narrow",
            f"centerline width {width_m:.3f} leaves no room after margin {margin_m:.3f}",
        )

    if corridor_reference is not None:
        centerline_points = list(corridor_reference)
    else:
        centerline_points = []
        for ref in centerline.get("points", []):
            point = map_model.point(str(ref))
            if point is not None:
                centerline_points.append(point)

    effective_area = map_model.polygon(map_model.effective_area.get("vertices"))
    for point in path:
        if effective_area and not _point_in_polygon(point.x, point.y, effective_area):
            return _failure(
                step,
                "outside_effective_area",
                f"path point ({point.x:.3f}, {point.y:.3f}) is outside effective_area",
            )
        if _inside_no_go_zone(point, map_model):
            return _failure(
                step,
                "inside_no_go_zone",
                f"path point ({point.x:.3f}, {point.y:.3f}) is inside a no_go_zone",
            )
        if (
            check_corridor_distance
            and _distance_to_polyline(point, centerline_points) > max_distance_m + 1e-9
        ):
            return _failure(
                step,
                "outside_corridor",
                f"path point ({point.x:.3f}, {point.y:.3f}) exceeds corridor margin",
            )

    tightest_radius_m = _minimum_turning_radius(path)
    if (
        min_turning_radius_m > 0.0
        and tightest_radius_m is not None
        and tightest_radius_m + 1e-9 < min_turning_radius_m
    ):
        return _failure(
            step,
            "curvature_exceeded",
            f"minimum turn radius {tightest_radius_m:.3f} m is below "
            f"limit {min_turning_radius_m:.3f} m",
        )
    return None


def _corridor_margin_m(params: dict[str, Any]) -> float:
    configured_margin_m = float(params.get("corridor_margin_m", 0.20))
    footprint_radius_m = float(params.get("footprint_radius_m", 0.0))
    clearance_m = float(params.get("clearance_m", 0.0))
    return max(configured_margin_m, footprint_radius_m + clearance_m)


def _load_grid_planner(semantic_map: dict[str, Any], params: dict[str, Any]) -> Any:
    from competition_planning.occupancy_grid_planner import (
        GridAStarPlanner,
        GridPlanningError,
        OccupancyGridMap,
    )

    configured_map_file = params.get("map_file") or semantic_map.get("source_map")
    if not configured_map_file:
        raise GridPlanningError("global_planner.map_file or semantic_map.source_map is required")
    map_file = Path(str(configured_map_file))
    if not map_file.exists():
        raise GridPlanningError(f"occupancy map file not found: {map_file}")

    footprint_radius_m = float(params.get("footprint_radius_m", 0.0))
    clearance_m = float(params.get("clearance_m", 0.0))
    inflation_radius_m = float(
        params.get("grid_inflation_radius_m", footprint_radius_m + clearance_m)
    )
    search_padding_m = float(params.get("grid_search_padding_m", 3.0))
    sample_spacing_m = float(params.get("path_sample_spacing_m", 0.25))
    simplify_path = bool(params.get("grid_simplify_path", True))
    return GridAStarPlanner(
        OccupancyGridMap.from_yaml(map_file),
        inflation_radius_m=inflation_radius_m,
        search_padding_m=search_padding_m,
        sample_spacing_m=sample_spacing_m,
        simplify_path=simplify_path,
    )


def _load_hybrid_planner(semantic_map: dict[str, Any], params: dict[str, Any]) -> Any:
    from competition_planning.hybrid_astar_planner import HybridAStarPlanner
    from competition_planning.occupancy_grid_planner import (
        GridPlanningError,
        OccupancyGridMap,
    )

    configured_map_file = params.get("map_file") or semantic_map.get("source_map")
    if not configured_map_file:
        raise GridPlanningError("global_planner.map_file or semantic_map.source_map is required")
    map_file = Path(str(configured_map_file))
    if not map_file.exists():
        raise GridPlanningError(f"occupancy map file not found: {map_file}")

    footprint_radius_m = float(params.get("footprint_radius_m", 0.0))
    clearance_m = float(params.get("clearance_m", 0.0))
    inflation_radius_m = float(
        params.get("grid_inflation_radius_m", footprint_radius_m + clearance_m)
    )
    return HybridAStarPlanner(
        OccupancyGridMap.from_yaml(map_file),
        inflation_radius_m=inflation_radius_m,
        search_padding_m=float(params.get("grid_search_padding_m", 3.0)),
        sample_spacing_m=float(params.get("path_sample_spacing_m", 0.10)),
        min_turning_radius_m=float(params.get("min_turning_radius_m", 0.0)),
        step_length_m=float(params.get("hybrid_step_length_m", 0.20)),
        curvature_bins=int(params.get("hybrid_curvature_bins", 9)),
        heading_bins=int(params.get("hybrid_heading_bins", 72)),
        goal_position_tolerance_m=float(
            params.get("hybrid_goal_position_tolerance_m", 0.15)
        ),
        goal_heading_tolerance_rad=math.radians(
            float(params.get("hybrid_goal_heading_tolerance_deg", 8.0))
        ),
        goal_lateral_tolerance_m=(
            float(params["hybrid_goal_lateral_tolerance_m"])
            if "hybrid_goal_lateral_tolerance_m" in params
            else None
        ),
        exact_goal_connection=bool(
            params.get("hybrid_exact_goal_connection", False)
        ),
        soft_waypoint_position_tolerance_m=float(
            params.get("hybrid_soft_waypoint_position_tolerance_m", 0.50)
        ),
        soft_waypoint_heading_tolerance_rad=math.radians(
            float(params.get("hybrid_soft_waypoint_heading_tolerance_deg", 45.0))
        ),
        alignment_waypoint_position_tolerance_m=float(
            params.get("hybrid_alignment_waypoint_position_tolerance_m", 0.25)
        ),
        alignment_waypoint_heading_tolerance_rad=math.radians(
            float(params.get("hybrid_alignment_waypoint_heading_tolerance_deg", 15.0))
        ),
        max_expansions=int(params.get("hybrid_max_expansions", 250_000)),
        reference_deviation_weight=float(
            params.get("hybrid_reference_deviation_weight", 0.0)
        ),
    )


def _load_smoother(planning_config: dict[str, Any]) -> Any:
    params = planning_config.get("trajectory_smoother", {})
    if not bool(params.get("enabled", False)):
        return None

    plugin = str(params.get("plugin", "cubic_bezier"))
    from competition_planning.trajectory_smoother import (
        CircularArcSmoother,
        CubicBezierSmoother,
    )

    if plugin == "cubic_bezier":
        return CubicBezierSmoother(
            sample_spacing_m=float(params.get("sample_spacing_m", 0.20)),
            tangent_scale=float(params.get("tangent_scale", 0.75)),
        )
    if plugin == "circular_arc":
        return CircularArcSmoother(
            sample_spacing_m=float(params.get("sample_spacing_m", 0.05)),
        )
    return None


def _with_semantic_anchor_yaws(
    path: tuple[PathPoint, ...],
    semantic_points: list[PathPoint | None],
) -> tuple[PathPoint, ...]:
    anchor_yaws = {
        point.ref_id: point.yaw
        for point in semantic_points
        if point is not None and point.ref_id is not None
    }
    return tuple(
        PathPoint(x=point.x, y=point.y, yaw=anchor_yaws[point.ref_id], ref_id=point.ref_id)
        if point.ref_id in anchor_yaws
        else point
        for point in path
    )


def _inside_no_go_zone(point: PathPoint, map_model: _SemanticMap) -> bool:
    for zone in map_model.no_go_zones:
        if not isinstance(zone, dict):
            continue
        if zone.get("geometry") == "outside_polygon":
            continue
        polygon = map_model.polygon(zone.get("boundary"))
        if polygon and _point_in_polygon(point.x, point.y, polygon):
            return True
    return False


def _distance_to_polyline(point: PathPoint, polyline: list[PathPoint]) -> float:
    if len(polyline) < 2:
        return math.inf
    return min(
        _distance_to_segment(point, start, end)
        for start, end in zip(polyline, polyline[1:])
    )


def _distance_to_segment(point: PathPoint, start: PathPoint, end: PathPoint) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(point.x - start.x, point.y - start.y)
    ratio = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq
    ratio = min(1.0, max(0.0, ratio))
    projection_x = start.x + ratio * dx
    projection_y = start.y + ratio * dy
    return math.hypot(point.x - projection_x, point.y - projection_y)


def _minimum_turning_radius(path: tuple[PathPoint, ...]) -> float | None:
    radii: list[float] = []
    for first, second, third in zip(path, path[1:], path[2:]):
        radius = _turning_radius(first, second, third)
        if radius is not None:
            radii.append(radius)
    return min(radii) if radii else None


def _turning_radius(
    first: PathPoint,
    second: PathPoint,
    third: PathPoint,
) -> float | None:
    ab = math.hypot(second.x - first.x, second.y - first.y)
    bc = math.hypot(third.x - second.x, third.y - second.y)
    ca = math.hypot(first.x - third.x, first.y - third.y)
    area = abs(
        (second.x - first.x) * (third.y - first.y)
        - (second.y - first.y) * (third.x - first.x)
    ) / 2.0
    if area <= 1e-9:
        return None
    return ab * bc * ca / (4.0 * area)


def _point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        if _point_on_segment_xy(x, y, x1, y1, x2, y2):
            return True
        crosses = (y1 > y) != (y2 > y)
        if crosses:
            intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersection_x:
                inside = not inside
    return inside


def _point_on_segment_xy(
    x: float,
    y: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> bool:
    cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
    if abs(cross) > 1e-9:
        return False
    dot = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)
    if dot < -1e-9:
        return False
    length_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    return dot <= length_sq + 1e-9


def _initial_current_ref(route: dict[str, Any], map_model: _SemanticMap) -> str | None:
    configured = route.get("start_ref")
    if configured and map_model.point(str(configured)) is not None:
        return str(configured)
    if map_model.point("start") is not None:
        return "start"
    return None


def _pose_ref_from_step(step: dict[str, Any]) -> str | None:
    for key in ("dock_pose_ref", "target_ref"):
        if step.get(key):
            return str(step[key])
    return None


def _failure(step: dict[str, Any], reason: str, detail: str) -> PlanFailure:
    return PlanFailure(
        step_id=str(step.get("id", "")),
        step_type=str(step.get("type", "")),
        reason=reason,
        detail=detail,
    )
