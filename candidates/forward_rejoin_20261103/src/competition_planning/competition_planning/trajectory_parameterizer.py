"""Offline speed and time parameterization for semantic global paths."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Any, Mapping, Sequence

from competition_planning.local_trajectory_planner import concatenate_reference_paths
from competition_planning.semantic_planner import (
    ContinuousRoutePlan,
    PathPoint,
    PlanFailure,
    RoutePlan,
    StepPlan,
    plan_continuous_route,
    plan_route,
)


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    s: float
    curvature: float
    v: float
    yaw_rate: float
    t: float
    ref_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 4),
            "s": round(self.s, 4),
            "curvature": round(self.curvature, 6),
            "v": round(self.v, 4),
            "yaw_rate": round(self.yaw_rate, 4),
            "t": round(self.t, 4),
        }
        if self.ref_id:
            result["ref_id"] = self.ref_id
        return result


@dataclass(frozen=True)
class OptimizedStepTrajectory:
    step_id: str
    step_type: str
    corridor_ref: str
    target_ref: str
    target_source: str
    planner_plugin: str
    smoother_plugin: str
    points: tuple[TrajectoryPoint, ...]

    @property
    def duration_s(self) -> float:
        return self.points[-1].t if self.points else 0.0

    @property
    def path_length_m(self) -> float:
        return self.points[-1].s if self.points else 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "step_id": self.step_id,
            "type": self.step_type,
            "corridor_ref": self.corridor_ref,
            "target_ref": self.target_ref,
            "target_source": self.target_source,
            "planner_plugin": self.planner_plugin,
            "smoother_plugin": self.smoother_plugin,
            "point_count": len(self.points),
            "path_length_m": round(self.path_length_m, 3),
            "duration_s": round(self.duration_s, 3),
            "points": [point.to_dict() for point in self.points],
        }


@dataclass(frozen=True)
class OptimizedRouteTrajectory:
    frame_id: str
    route_name: str
    trajectories: tuple[OptimizedStepTrajectory, ...]
    failures: tuple[PlanFailure, ...]

    @property
    def ok(self) -> bool:
        return not self.failures

    def to_dict(self) -> dict[str, Any]:
        return {
            "route_name": self.route_name,
            "frame_id": self.frame_id,
            "ok": self.ok,
            "trajectories": [trajectory.to_dict() for trajectory in self.trajectories],
            "failures": [failure.to_dict() for failure in self.failures],
        }


@dataclass(frozen=True)
class ContinuousTrajectoryPoint:
    x: float
    y: float
    yaw: float
    s: float
    curvature: float
    v: float
    a: float
    jerk: float
    yaw_rate: float
    t: float
    ref_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 4),
            "s": round(self.s, 4),
            "curvature": round(self.curvature, 6),
            "v": round(self.v, 4),
            "a": round(self.a, 4),
            "jerk": round(self.jerk, 4),
            "yaw_rate": round(self.yaw_rate, 4),
            "t": round(self.t, 4),
        }
        if self.ref_id:
            result["ref_id"] = self.ref_id
        return result


@dataclass(frozen=True)
class ContinuousRouteTrajectory:
    frame_id: str
    route_name: str
    planner_plugin: str
    optimizer_plugin: str
    points: tuple[ContinuousTrajectoryPoint, ...]
    failures: tuple[PlanFailure, ...]

    @property
    def ok(self) -> bool:
        return not self.failures

    @property
    def duration_s(self) -> float:
        return self.points[-1].t if self.points else 0.0

    @property
    def path_length_m(self) -> float:
        return self.points[-1].s if self.points else 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "route_name": self.route_name,
            "frame_id": self.frame_id,
            "ok": self.ok,
            "planner_plugin": self.planner_plugin,
            "optimizer_plugin": self.optimizer_plugin,
            "point_count": len(self.points),
            "path_length_m": round(self.path_length_m, 3),
            "duration_s": round(self.duration_s, 3),
            "points": [point.to_dict() for point in self.points],
            "failures": [failure.to_dict() for failure in self.failures],
        }


def optimize_route_trajectory(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    planning_config: dict[str, Any] | None = None,
    optimizer_config: dict[str, Any] | None = None,
) -> OptimizedRouteTrajectory:
    """Plan a semantic route and return its optimized trajectory artifact.

    This is the external seam for offline tools and future ROS adapters. Callers
    should not need to orchestrate global planning and speed/time
    parameterization separately.
    """

    route_plan = plan_route(route, semantic_map, planning_config)
    return parameterize_route_plan(route_plan, semantic_map, optimizer_config)


def optimize_continuous_route_trajectory(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    planning_config: dict[str, Any] | None = None,
    optimizer_config: dict[str, Any] | None = None,
    *,
    end_ref: str | None = None,
    end_ref_occurrence: int = 1,
) -> ContinuousRouteTrajectory:
    """Return one whole-line path with a bounded S-curve speed profile."""

    planning_mode = str(route.get("continuous_planning_mode", "single_search"))
    if planning_mode == "single_search":
        route_plan = plan_continuous_route(route, semantic_map, planning_config)
    elif planning_mode == "joined_step_paths":
        route_plan = _plan_joined_step_paths(
            route,
            semantic_map,
            planning_config,
        )
    else:
        route_plan = ContinuousRoutePlan(
            frame_id=str(semantic_map.get("frame_id", "map")),
            route_name=str(route.get("route_name", "")),
            path=(),
            planner_plugin="",
            planning_time_ms=0.0,
            failures=(
                PlanFailure(
                    step_id="continuous_route",
                    step_type="CONTROL_ROUTE",
                    reason="unsupported_continuous_planning_mode",
                    detail=f"unsupported continuous_planning_mode {planning_mode}",
                ),
            ),
        )
    params = (optimizer_config or {}).get("continuous_trajectory_optimizer", {})
    optimizer_plugin = str(params.get("plugin", "jerk_limited_s_curve"))
    if not route_plan.ok:
        return _empty_continuous_trajectory(route_plan, optimizer_plugin)
    if end_ref:
        try:
            route_plan = _continuous_plan_through_ref(
                route_plan,
                end_ref,
                occurrence=end_ref_occurrence,
            )
        except ValueError as exc:
            failure = PlanFailure(
                step_id="continuous_route",
                step_type="CONTROL_ROUTE",
                reason="invalid_staged_endpoint",
                detail=str(exc),
            )
            return ContinuousRouteTrajectory(
                frame_id=route_plan.frame_id,
                route_name=route_plan.route_name,
                planner_plugin=route_plan.planner_plugin,
                optimizer_plugin=optimizer_plugin,
                points=(),
                failures=(failure,),
            )
    if optimizer_plugin != "jerk_limited_s_curve":
        failure = PlanFailure(
            step_id="continuous_route",
            step_type="CONTROL_ROUTE",
            reason="unsupported_continuous_optimizer",
            detail=f"unsupported optimizer plugin {optimizer_plugin}",
        )
        return ContinuousRouteTrajectory(
            frame_id=route_plan.frame_id,
            route_name=route_plan.route_name,
            planner_plugin=route_plan.planner_plugin,
            optimizer_plugin=optimizer_plugin,
            points=(),
            failures=(failure,),
        )
    try:
        points = _parameterize_continuous_path(
            route_plan.path,
            params,
            route_speed_profile=route.get("route_speed_profile"),
        )
    except ValueError as exc:
        failure = PlanFailure(
            step_id="continuous_route",
            step_type="CONTROL_ROUTE",
            reason="continuous_parameterization_failed",
            detail=str(exc),
        )
        return ContinuousRouteTrajectory(
            frame_id=route_plan.frame_id,
            route_name=route_plan.route_name,
            planner_plugin=route_plan.planner_plugin,
            optimizer_plugin=optimizer_plugin,
            points=(),
            failures=(failure,),
        )
    return ContinuousRouteTrajectory(
        frame_id=route_plan.frame_id,
        route_name=route_plan.route_name,
        planner_plugin=route_plan.planner_plugin,
        optimizer_plugin=optimizer_plugin,
        points=points,
        failures=(),
    )


def retime_continuous_trajectory(
    artifact: Mapping[str, Any],
    optimizer_config: dict[str, Any] | None = None,
) -> ContinuousRouteTrajectory:
    """Apply a new speed profile while preserving frozen route geometry."""

    raw_points = artifact.get("points")
    if artifact.get("ok") is not True or not isinstance(raw_points, list):
        raise ValueError("reference trajectory must be a successful continuous artifact")
    if len(raw_points) < 2 or not all(isinstance(point, dict) for point in raw_points):
        raise ValueError("reference trajectory requires at least two point mappings")
    path = tuple(
        PathPoint(
            x=float(point["x"]),
            y=float(point["y"]),
            yaw=float(point["yaw"]),
            ref_id=str(point["ref_id"]) if point.get("ref_id") else None,
        )
        for point in raw_points
    )
    distances = [float(point["s"]) for point in raw_points]
    curvatures = [float(point["curvature"]) for point in raw_points]
    params = (optimizer_config or {}).get("continuous_trajectory_optimizer", {})
    optimizer_plugin = str(params.get("plugin", "jerk_limited_s_curve"))
    if optimizer_plugin != "jerk_limited_s_curve":
        raise ValueError(f"unsupported optimizer plugin {optimizer_plugin}")
    points = _parameterize_continuous_path(
        path,
        params,
        distances=distances,
        curvatures=curvatures,
    )
    return ContinuousRouteTrajectory(
        frame_id=str(artifact.get("frame_id", "map")),
        route_name=str(artifact.get("route_name", "")),
        planner_plugin=str(artifact.get("planner_plugin", "")),
        optimizer_plugin=optimizer_plugin,
        points=points,
        failures=(),
    )


def _plan_joined_step_paths(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    planning_config: dict[str, Any] | None,
) -> ContinuousRoutePlan:
    """Join globally planned task intervals before one speed parameterization.

    Precision docking poses are exact interval anchors.  Joining the planned
    intervals avoids carrying Hybrid A* endpoint tolerance into the next
    pose-constrained search, while still exposing one continuous reference to
    control and local replanning.
    """

    step_plan = plan_route(route, semantic_map, planning_config)
    planner_plugin = next(
        (plan.planner_plugin for plan in step_plan.plans if plan.planner_plugin),
        "",
    )
    if not step_plan.ok:
        return ContinuousRoutePlan(
            frame_id=step_plan.frame_id,
            route_name=step_plan.route_name,
            path=(),
            planner_plugin=planner_plugin,
            planning_time_ms=sum(plan.planning_time_ms for plan in step_plan.plans),
            failures=step_plan.failures,
        )
    try:
        path = concatenate_reference_paths(
            tuple(plan.path for plan in step_plan.plans),
            preserve_next_path=True,
        )
    except ValueError as exc:
        return ContinuousRoutePlan(
            frame_id=step_plan.frame_id,
            route_name=step_plan.route_name,
            path=(),
            planner_plugin=planner_plugin,
            planning_time_ms=sum(plan.planning_time_ms for plan in step_plan.plans),
            failures=(
                PlanFailure(
                    step_id="continuous_route",
                    step_type="CONTROL_ROUTE",
                    reason="continuous_path_join_failed",
                    detail=str(exc),
                ),
            ),
        )
    return ContinuousRoutePlan(
        frame_id=step_plan.frame_id,
        route_name=step_plan.route_name,
        path=path,
        planner_plugin=planner_plugin,
        planning_time_ms=sum(plan.planning_time_ms for plan in step_plan.plans),
        failures=(),
    )


def _continuous_plan_through_ref(
    route_plan: ContinuousRoutePlan,
    end_ref: str,
    *,
    occurrence: int,
) -> ContinuousRoutePlan:
    if occurrence <= 0:
        raise ValueError("end_ref_occurrence must be positive")
    matches = [
        index for index, point in enumerate(route_plan.path) if point.ref_id == end_ref
    ]
    if len(matches) < occurrence:
        raise ValueError(
            f"continuous route contains {len(matches)} occurrence(s) of "
            f"end_ref={end_ref}; requested {occurrence}"
        )
    end_index = matches[occurrence - 1]
    if end_index < 1:
        raise ValueError("staged route must contain at least two path points")
    return replace(
        route_plan,
        route_name=f"{route_plan.route_name}:through:{end_ref}:{occurrence}",
        path=route_plan.path[: end_index + 1],
    )


def _empty_continuous_trajectory(
    route_plan: ContinuousRoutePlan,
    optimizer_plugin: str,
) -> ContinuousRouteTrajectory:
    return ContinuousRouteTrajectory(
        frame_id=route_plan.frame_id,
        route_name=route_plan.route_name,
        planner_plugin=route_plan.planner_plugin,
        optimizer_plugin=optimizer_plugin,
        points=(),
        failures=route_plan.failures,
    )


def _parameterize_continuous_path(
    path: Sequence[PathPoint],
    params: dict[str, Any],
    *,
    distances: Sequence[float] | None = None,
    curvatures: Sequence[float] | None = None,
    route_speed_profile: Mapping[str, Any] | None = None,
) -> tuple[ContinuousTrajectoryPoint, ...]:
    if len(path) < 2:
        raise ValueError("continuous route requires at least two path points")
    max_speed_mps = _positive_float(params, "max_speed_mps", 0.20)
    max_acceleration_mps2 = _positive_float(
        params,
        "max_acceleration_mps2",
        0.20,
    )
    max_deceleration_mps2 = _positive_float(
        params,
        "max_deceleration_mps2",
        0.30,
    )
    max_jerk_mps3 = _positive_float(params, "max_jerk_mps3", 0.40)
    max_lateral_acceleration_mps2 = _positive_float(
        params,
        "max_lateral_acceleration_mps2",
        0.20,
    )
    max_curvature_rate_1pmps = _positive_float(
        params,
        "max_curvature_rate_1pmps",
        0.80,
    )

    distances = (
        list(distances) if distances is not None else _cumulative_distances(path)
    )
    if len(distances) != len(path):
        raise ValueError("continuous route distance count does not match path points")
    if any(current <= previous for previous, current in zip(distances, distances[1:])):
        raise ValueError("continuous route contains duplicate or reversed path samples")
    curvatures = (
        list(curvatures) if curvatures is not None else _path_curvatures(path)
    )
    if len(curvatures) != len(path):
        raise ValueError("continuous route curvature count does not match path points")
    if bool(params.get("spatial_speed_envelope", False)):
        return _parameterize_spatial_speed_envelope(
            path,
            distances,
            curvatures,
            max_speed_mps=max_speed_mps,
            max_acceleration_mps2=max_acceleration_mps2,
            max_deceleration_mps2=max_deceleration_mps2,
            max_jerk_mps3=max_jerk_mps3,
            max_lateral_acceleration_mps2=max_lateral_acceleration_mps2,
            max_curvature_rate_1pmps=max_curvature_rate_1pmps,
            route_speed_profile=route_speed_profile,
        )
    if route_speed_profile is not None:
        raise ValueError(
            "route speed profile requires spatial_speed_envelope"
        )

    curve_caps = [
        math.sqrt(max_lateral_acceleration_mps2 / abs(curvature))
        for curvature in curvatures
        if abs(curvature) > 1e-9
    ]
    curvature_rate_caps = [
        max_curvature_rate_1pmps
        * (current_distance - previous_distance)
        / abs(current_curvature - previous_curvature)
        for previous_distance, current_distance, previous_curvature, current_curvature
        in zip(distances, distances[1:], curvatures, curvatures[1:])
        if abs(current_curvature - previous_curvature) > 1e-9
    ]
    profile_speed_mps = min([max_speed_mps, *curve_caps, *curvature_rate_caps])
    profile = _JerkLimitedProfile(
        distance_m=distances[-1],
        max_speed_mps=profile_speed_mps,
        max_acceleration_mps2=min(max_acceleration_mps2, max_deceleration_mps2),
        max_jerk_mps3=max_jerk_mps3,
    )

    output: list[ContinuousTrajectoryPoint] = []
    for point, distance, curvature in zip(path, distances, curvatures):
        timestamp, speed, acceleration, jerk = profile.state_at_distance(distance)
        output.append(
            ContinuousTrajectoryPoint(
                x=point.x,
                y=point.y,
                yaw=point.yaw,
                s=distance,
                curvature=curvature,
                v=speed,
                a=acceleration,
                jerk=jerk,
                yaw_rate=speed * curvature,
                t=timestamp,
                ref_id=point.ref_id,
            )
        )
    result = tuple(output)
    for previous, current in zip(result, result[1:]):
        curvature_rate = abs(current.curvature - previous.curvature) / (
            current.t - previous.t
        )
        if curvature_rate > max_curvature_rate_1pmps + 1e-9:
            raise ValueError(
                "continuous route curvature rate "
                f"{curvature_rate:.6f} 1/m/s exceeds "
                f"{max_curvature_rate_1pmps:.6f} 1/m/s"
            )
    return result


def _parameterize_spatial_speed_envelope(
    path: Sequence[PathPoint],
    distances: Sequence[float],
    curvatures: Sequence[float],
    *,
    max_speed_mps: float,
    max_acceleration_mps2: float,
    max_deceleration_mps2: float,
    max_jerk_mps3: float,
    max_lateral_acceleration_mps2: float,
    max_curvature_rate_1pmps: float,
    route_speed_profile: Mapping[str, Any] | None = None,
) -> tuple[ContinuousTrajectoryPoint, ...]:
    """Keep straight-line speed local instead of throttling the whole route."""

    speed_caps = [
        min(
            max_speed_mps,
            math.sqrt(max_lateral_acceleration_mps2 / abs(curvature))
            if abs(curvature) > 1e-9
            else max_speed_mps,
        )
        for curvature in curvatures
    ]
    route_speed_caps = _route_speed_caps(
        path,
        max_speed_mps=max_speed_mps,
        route_speed_profile=route_speed_profile,
    )
    speed_caps = [
        min(curvature_cap, route_cap)
        for curvature_cap, route_cap in zip(speed_caps, route_speed_caps)
    ]
    segments = zip(distances, distances[1:], curvatures, curvatures[1:])
    for index, (
        previous_distance,
        current_distance,
        previous_curvature,
        current_curvature,
    ) in enumerate(segments):
        curvature_delta = abs(current_curvature - previous_curvature)
        if curvature_delta <= 1e-9:
            continue
        segment_cap = (
            max_curvature_rate_1pmps
            * (current_distance - previous_distance)
            / curvature_delta
        )
        speed_caps[index] = min(speed_caps[index], segment_cap)
        speed_caps[index + 1] = min(speed_caps[index + 1], segment_cap)
    speed_caps[0] = 0.0
    speed_caps[-1] = 0.0

    speeds = _apply_acceleration_limits(
        speed_caps,
        distances,
        max_acceleration_mps2,
        max_deceleration_mps2,
    )
    speeds = _smooth_speed_profile_for_jerk(
        speeds,
        distances,
        max_acceleration_mps2=max_acceleration_mps2,
        max_deceleration_mps2=max_deceleration_mps2,
        max_jerk_mps3=max_jerk_mps3,
    )
    times, accelerations, jerks = _speed_profile_kinematics(distances, speeds)

    if max(accelerations) > max_acceleration_mps2 + 1e-9:
        raise ValueError("continuous route exceeds the acceleration limit")
    if min(accelerations) < -max_deceleration_mps2 - 1e-9:
        raise ValueError("continuous route exceeds the deceleration limit")
    if max(abs(jerk) for jerk in jerks) > max_jerk_mps3 + 1e-9:
        raise ValueError(
            "continuous route spatial speed envelope exceeds the jerk limit"
        )

    result = tuple(
        ContinuousTrajectoryPoint(
            x=point.x,
            y=point.y,
            yaw=point.yaw,
            s=distance,
            curvature=curvature,
            v=speed,
            a=acceleration,
            jerk=jerk,
            yaw_rate=speed * curvature,
            t=timestamp,
            ref_id=point.ref_id,
        )
        for point, distance, curvature, speed, acceleration, jerk, timestamp in zip(
            path,
            distances,
            curvatures,
            speeds,
            accelerations,
            jerks,
            times,
        )
    )
    for previous, current in zip(result, result[1:]):
        curvature_rate = abs(current.curvature - previous.curvature) / (
            current.t - previous.t
        )
        if curvature_rate > max_curvature_rate_1pmps + 1e-9:
            raise ValueError(
                "continuous route curvature rate "
                f"{curvature_rate:.6f} 1/m/s exceeds "
                f"{max_curvature_rate_1pmps:.6f} 1/m/s"
            )
    return result


def _route_speed_caps(
    path: Sequence[PathPoint],
    *,
    max_speed_mps: float,
    route_speed_profile: Mapping[str, Any] | None,
) -> list[float]:
    if route_speed_profile is None:
        return [max_speed_mps] * len(path)
    try:
        default_max_speed_mps = float(
            route_speed_profile["default_max_speed_mps"]
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("route speed profile default is invalid") from exc
    if (
        not math.isfinite(default_max_speed_mps)
        or default_max_speed_mps <= 0.0
    ):
        raise ValueError("route speed profile default must be positive")
    raw_segments = route_speed_profile.get("segments", [])
    if not isinstance(raw_segments, list):
        raise ValueError("route speed profile segments must be a list")

    point_indices_by_ref: dict[str, list[int]] = {}
    for index, point in enumerate(path):
        if point.ref_id:
            point_indices_by_ref.setdefault(point.ref_id, []).append(index)
    speed_caps = [min(max_speed_mps, default_max_speed_mps)] * len(path)
    seen_refs: set[str] = set()
    for raw_segment in raw_segments:
        if not isinstance(raw_segment, Mapping):
            raise ValueError("route speed profile entries must be mappings")
        ref_id = str(raw_segment.get("id", "")).strip()
        start_ref = str(raw_segment.get("start_ref", "")).strip()
        end_ref = str(raw_segment.get("end_ref", "")).strip()
        try:
            start_occurrence = int(raw_segment.get("start_occurrence", 1))
            end_occurrence = int(raw_segment.get("end_occurrence", 1))
            segment_max_speed_mps = float(raw_segment["max_speed_mps"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("route speed profile values are invalid") from exc
        if not ref_id or not start_ref or not end_ref:
            raise ValueError("route speed profile refs must be non-empty")
        if ref_id in seen_refs:
            raise ValueError(f"duplicate route speed segment {ref_id}")
        if start_occurrence <= 0 or end_occurrence <= 0:
            raise ValueError("route speed profile occurrences must be positive")
        if (
            not math.isfinite(segment_max_speed_mps)
            or segment_max_speed_mps <= 0.0
        ):
            raise ValueError("route speed profile limit must be positive")
        start_indices = point_indices_by_ref.get(start_ref, [])
        end_indices = point_indices_by_ref.get(end_ref, [])
        if len(start_indices) < start_occurrence:
            raise ValueError(
                f"route speed segment {ref_id} start is not on path"
            )
        if len(end_indices) < end_occurrence:
            raise ValueError(
                f"route speed segment {ref_id} end is not on path"
            )
        start_index = start_indices[start_occurrence - 1]
        end_index = end_indices[end_occurrence - 1]
        if end_index <= start_index:
            raise ValueError(
                f"route speed segment {ref_id} must end after its start"
            )
        bounded_speed_mps = min(max_speed_mps, segment_max_speed_mps)
        for index in range(start_index, end_index + 1):
            if index in (start_index, end_index):
                speed_caps[index] = min(speed_caps[index], bounded_speed_mps)
            else:
                speed_caps[index] = bounded_speed_mps
        seen_refs.add(ref_id)
    return speed_caps


@dataclass(frozen=True)
class _ProfilePhase:
    start_t: float
    duration: float
    start_s: float
    start_v: float
    start_a: float
    jerk: float

    def state(self, elapsed: float) -> tuple[float, float, float]:
        elapsed = min(self.duration, max(0.0, elapsed))
        acceleration = self.start_a + self.jerk * elapsed
        speed = self.start_v + self.start_a * elapsed + 0.5 * self.jerk * elapsed**2
        distance = (
            self.start_s
            + self.start_v * elapsed
            + 0.5 * self.start_a * elapsed**2
            + self.jerk * elapsed**3 / 6.0
        )
        return distance, speed, acceleration


class _JerkLimitedProfile:
    def __init__(
        self,
        *,
        distance_m: float,
        max_speed_mps: float,
        max_acceleration_mps2: float,
        max_jerk_mps3: float,
    ) -> None:
        if distance_m <= 0.0:
            raise ValueError("continuous route has zero path length")
        peak_speed = self._peak_speed(
            distance_m,
            max_speed_mps,
            max_acceleration_mps2,
            max_jerk_mps3,
        )
        jerk_time, constant_acceleration_time = self._acceleration_times(
            peak_speed,
            max_acceleration_mps2,
            max_jerk_mps3,
        )
        acceleration_time = 2.0 * jerk_time + constant_acceleration_time
        acceleration_distance = 0.5 * peak_speed * acceleration_time
        cruise_distance = max(0.0, distance_m - 2.0 * acceleration_distance)
        cruise_time = cruise_distance / peak_speed if cruise_distance > 0.0 else 0.0

        raw_phases = [
            (jerk_time, max_jerk_mps3),
            (constant_acceleration_time, 0.0),
            (jerk_time, -max_jerk_mps3),
            (cruise_time, 0.0),
            (jerk_time, -max_jerk_mps3),
            (constant_acceleration_time, 0.0),
            (jerk_time, max_jerk_mps3),
        ]
        phases: list[_ProfilePhase] = []
        time_s = 0.0
        distance = 0.0
        speed = 0.0
        acceleration = 0.0
        for duration, jerk in raw_phases:
            if duration <= 1e-12:
                continue
            phase = _ProfilePhase(
                start_t=time_s,
                duration=duration,
                start_s=distance,
                start_v=speed,
                start_a=acceleration,
                jerk=jerk,
            )
            phases.append(phase)
            distance, speed, acceleration = phase.state(duration)
            time_s += duration
        self._distance_m = distance_m
        self._duration_s = time_s
        self._phases = tuple(phases)

    @staticmethod
    def _acceleration_times(
        speed_mps: float,
        acceleration_mps2: float,
        jerk_mps3: float,
    ) -> tuple[float, float]:
        transition_speed = acceleration_mps2**2 / jerk_mps3
        if speed_mps <= transition_speed:
            return math.sqrt(speed_mps / jerk_mps3), 0.0
        jerk_time = acceleration_mps2 / jerk_mps3
        return jerk_time, speed_mps / acceleration_mps2 - jerk_time

    @classmethod
    def _peak_speed(
        cls,
        distance_m: float,
        requested_speed_mps: float,
        acceleration_mps2: float,
        jerk_mps3: float,
    ) -> float:
        jerk_time, constant_time = cls._acceleration_times(
            requested_speed_mps,
            acceleration_mps2,
            jerk_mps3,
        )
        required_distance = requested_speed_mps * (2.0 * jerk_time + constant_time)
        if required_distance <= distance_m:
            return requested_speed_mps
        low = 0.0
        high = requested_speed_mps
        for _ in range(80):
            candidate = 0.5 * (low + high)
            jerk_time, constant_time = cls._acceleration_times(
                candidate,
                acceleration_mps2,
                jerk_mps3,
            )
            candidate_distance = candidate * (2.0 * jerk_time + constant_time)
            if candidate_distance < distance_m:
                low = candidate
            else:
                high = candidate
        return 0.5 * (low + high)

    def state_at_distance(self, target_distance_m: float) -> tuple[float, float, float, float]:
        if target_distance_m <= 0.0:
            return 0.0, 0.0, 0.0, 0.0
        if target_distance_m >= self._distance_m:
            return self._duration_s, 0.0, 0.0, 0.0
        low = 0.0
        high = self._duration_s
        for _ in range(64):
            timestamp = 0.5 * (low + high)
            distance, _, _, _ = self._state_at_time(timestamp)
            if distance < target_distance_m:
                low = timestamp
            else:
                high = timestamp
        timestamp = 0.5 * (low + high)
        _, speed, acceleration, jerk = self._state_at_time(timestamp)
        return timestamp, speed, acceleration, jerk

    def _state_at_time(self, timestamp: float) -> tuple[float, float, float, float]:
        for phase in self._phases:
            if timestamp <= phase.start_t + phase.duration + 1e-12:
                distance, speed, acceleration = phase.state(timestamp - phase.start_t)
                return distance, speed, acceleration, phase.jerk
        return self._distance_m, 0.0, 0.0, 0.0


def parameterize_route_plan(
    route_plan: RoutePlan,
    semantic_map: dict[str, Any],
    optimizer_config: dict[str, Any] | None = None,
) -> OptimizedRouteTrajectory:
    trajectories: list[OptimizedStepTrajectory] = []
    failures = list(route_plan.failures)
    for plan in route_plan.plans:
        try:
            trajectories.append(parameterize_step_plan(plan, semantic_map, optimizer_config))
        except ValueError as exc:
            failures.append(
                PlanFailure(
                    step_id=plan.step_id,
                    step_type=plan.step_type,
                    reason="trajectory_parameterization_failed",
                    detail=str(exc),
                )
            )
    return OptimizedRouteTrajectory(
        frame_id=route_plan.frame_id,
        route_name=route_plan.route_name,
        trajectories=tuple(trajectories),
        failures=tuple(failures),
    )


def parameterize_local_path(
    path: Sequence[PathPoint],
    semantic_map: dict[str, Any],
    optimizer_config: dict[str, Any] | None = None,
) -> OptimizedStepTrajectory:
    """Apply the Day4 speed/time parameterizer to one online local path."""

    return parameterize_step_plan(
        StepPlan(
            step_id="online_local_replan",
            step_type="RUN_SEGMENT",
            corridor_ref="active_global_reference",
            target_ref=(path[-1].ref_id or "local_rejoin") if path else "local_rejoin",
            target_source="global_reference_lookahead",
            path=tuple(path),
            planning_time_ms=0.0,
            planner_plugin="reference_aware_hybrid_astar",
            smoother_plugin="none",
        ),
        semantic_map,
        optimizer_config,
    )


def parameterize_step_plan(
    plan: StepPlan,
    semantic_map: dict[str, Any],
    optimizer_config: dict[str, Any] | None = None,
) -> OptimizedStepTrajectory:
    """Convert one geometric global path into a time-parameterized trajectory."""

    if not plan.path:
        raise ValueError(f"{plan.step_id} has no path points")

    params = (optimizer_config or {}).get("trajectory_optimizer", {})
    max_speed_mps = float(params.get("max_speed_mps", 0.50))
    max_acceleration_mps2 = _positive_float(params, "max_acceleration_mps2", 0.30)
    max_deceleration_mps2 = _positive_float(params, "max_deceleration_mps2", 0.50)
    max_lateral_acceleration_mps2 = _positive_float(
        params,
        "max_lateral_acceleration_mps2",
        0.20,
    )
    max_curvature_1pm = _optional_positive_float(params, "max_curvature_1pm")
    high_curvature_threshold_1pm = _optional_positive_float(
        params,
        "high_curvature_threshold_1pm",
    )
    high_curvature_speed_limit_mps = _optional_positive_float(
        params,
        "high_curvature_speed_limit_mps",
    )
    if (high_curvature_threshold_1pm is None) != (
        high_curvature_speed_limit_mps is None
    ):
        raise ValueError(
            "trajectory_optimizer high-curvature threshold and speed limit "
            "must be configured together"
        )
    curvature_overshoot_tolerance_ratio = _nonnegative_float(
        params,
        "curvature_overshoot_tolerance_ratio",
        0.02,
    )
    zone_speed_limits = {
        str(key): float(value)
        for key, value in params.get("obstacle_zone_speed_limits_mps", {}).items()
    }

    distances = _cumulative_distances(plan.path)
    curvatures = _path_curvatures(plan.path)
    if max_curvature_1pm is not None:
        curvatures = _apply_curvature_envelope(
            curvatures,
            max_curvature_1pm=max_curvature_1pm,
            overshoot_tolerance_ratio=curvature_overshoot_tolerance_ratio,
        )
    stop_refs = _semantic_stop_refs(semantic_map)
    obstacle_zones = _obstacle_zones(semantic_map)
    ref_speed_limits = _semantic_ref_speed_limits(semantic_map, zone_speed_limits)

    speed_caps = [
        _speed_cap_for_point(
            point,
            curvature,
            max_speed_mps,
            max_lateral_acceleration_mps2,
            stop_refs,
            obstacle_zones,
            zone_speed_limits,
            ref_speed_limits,
            high_curvature_threshold_1pm,
            high_curvature_speed_limit_mps,
        )
        for point, curvature in zip(plan.path, curvatures)
    ]
    speeds = _apply_acceleration_limits(
        speed_caps,
        distances,
        max_acceleration_mps2,
        max_deceleration_mps2,
    )
    times = _integrate_times(distances, speeds)

    points = tuple(
        TrajectoryPoint(
            x=path_point.x,
            y=path_point.y,
            yaw=path_point.yaw,
            s=distance,
            curvature=curvature,
            v=speed,
            yaw_rate=speed * curvature,
            t=timestamp,
            ref_id=path_point.ref_id,
        )
        for path_point, distance, curvature, speed, timestamp in zip(
            plan.path,
            distances,
            curvatures,
            speeds,
            times,
        )
    )
    return OptimizedStepTrajectory(
        step_id=plan.step_id,
        step_type=plan.step_type,
        corridor_ref=plan.corridor_ref,
        target_ref=plan.target_ref,
        target_source=plan.target_source,
        planner_plugin=plan.planner_plugin,
        smoother_plugin=plan.smoother_plugin,
        points=points,
    )


def _positive_float(params: dict[str, Any], key: str, default: float) -> float:
    value = float(params.get(key, default))
    if value <= 0.0:
        raise ValueError(f"trajectory_optimizer.{key} must be positive")
    return value


def _optional_positive_float(params: dict[str, Any], key: str) -> float | None:
    if key not in params or params[key] is None:
        return None
    value = float(params[key])
    if value <= 0.0:
        raise ValueError(f"trajectory_optimizer.{key} must be positive")
    return value


def _nonnegative_float(params: dict[str, Any], key: str, default: float) -> float:
    value = float(params.get(key, default))
    if value < 0.0:
        raise ValueError(f"trajectory_optimizer.{key} must be non-negative")
    return value


def _apply_curvature_envelope(
    curvatures: Sequence[float],
    *,
    max_curvature_1pm: float,
    overshoot_tolerance_ratio: float,
) -> list[float]:
    hard_limit = max_curvature_1pm * (1.0 + overshoot_tolerance_ratio)
    bounded: list[float] = []
    for curvature in curvatures:
        magnitude = abs(curvature)
        if magnitude > hard_limit + 1e-9:
            raise ValueError(
                "trajectory exceeds the runtime turning-radius envelope: "
                f"|curvature|={magnitude:.6f} 1/m exceeds "
                f"{max_curvature_1pm:.6f} 1/m"
            )
        if magnitude > max_curvature_1pm:
            bounded.append(math.copysign(max_curvature_1pm, curvature))
        else:
            bounded.append(curvature)
    return bounded


def _cumulative_distances(path: Sequence[PathPoint]) -> list[float]:
    distances = [0.0]
    for previous, current in zip(path, path[1:]):
        distances.append(
            distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    return distances


def _path_curvatures(path: Sequence[PathPoint]) -> list[float]:
    if len(path) < 3:
        return [0.0 for _ in path]

    curvatures = [0.0 for _ in path]
    for index, (first, second, third) in enumerate(zip(path, path[1:], path[2:]), start=1):
        curvatures[index] = _signed_curvature(first, second, third)
    curvatures[0] = curvatures[1]
    curvatures[-1] = curvatures[-2]
    # Semantic anchors are possible state-machine restart points. Use their
    # outgoing geometry so curvature from the completed incoming segment does
    # not command an immediate steering transient after a precision stop.
    for index, point in enumerate(path[:-2]):
        if point.ref_id:
            curvatures[index] = _signed_curvature(
                path[index],
                path[index + 1],
                path[index + 2],
            )
    return curvatures


def _signed_curvature(first: PathPoint, second: PathPoint, third: PathPoint) -> float:
    ab = math.hypot(second.x - first.x, second.y - first.y)
    bc = math.hypot(third.x - second.x, third.y - second.y)
    ca = math.hypot(first.x - third.x, first.y - third.y)
    denominator = ab * bc * ca
    if denominator <= 1e-12:
        return 0.0
    cross = (second.x - first.x) * (third.y - first.y) - (
        second.y - first.y
    ) * (third.x - first.x)
    return 2.0 * cross / denominator


def _semantic_stop_refs(semantic_map: dict[str, Any]) -> set[str]:
    refs: set[str] = set()
    for stop_line in semantic_map.get("stop_lines", []):
        if isinstance(stop_line, dict) and stop_line.get("point_ref"):
            refs.add(str(stop_line["point_ref"]))
    for dock_pose in semantic_map.get("dock_poses", []):
        if isinstance(dock_pose, dict) and dock_pose.get("point_ref"):
            refs.add(str(dock_pose["point_ref"]))
    return refs


def _obstacle_zones(
    semantic_map: dict[str, Any],
) -> list[tuple[str, list[tuple[float, float]]]]:
    zones: list[tuple[str, list[tuple[float, float]]]] = []
    for zone in semantic_map.get("obstacle_zones", []):
        if not isinstance(zone, dict):
            continue
        polygon = _polygon(zone.get("boundary"))
        if polygon:
            zones.append((str(zone.get("semantic_type", "")), polygon))
    return zones


def _semantic_ref_speed_limits(
    semantic_map: dict[str, Any],
    zone_speed_limits: dict[str, float],
) -> dict[str, float]:
    limits: dict[str, float] = {}
    points = semantic_map.get("points", {})
    if not isinstance(points, dict):
        return limits
    for ref_id, raw in points.items():
        if not isinstance(raw, dict):
            continue
        role = str(raw.get("role", "")).upper()
        matching = [
            limit
            for semantic_type, limit in zone_speed_limits.items()
            if semantic_type.upper() in role
        ]
        if matching:
            limits[str(ref_id)] = min(matching)
    return limits


def _polygon(raw: Any) -> list[tuple[float, float]]:
    if not isinstance(raw, list):
        return []
    polygon: list[tuple[float, float]] = []
    for vertex in raw:
        if not isinstance(vertex, list | tuple) or len(vertex) < 2:
            return []
        polygon.append((float(vertex[0]), float(vertex[1])))
    return polygon


def _speed_cap_for_point(
    point: PathPoint,
    curvature: float,
    max_speed_mps: float,
    max_lateral_acceleration_mps2: float,
    stop_refs: set[str],
    obstacle_zones: list[tuple[str, list[tuple[float, float]]]],
    zone_speed_limits: dict[str, float],
    ref_speed_limits: dict[str, float],
    high_curvature_threshold_1pm: float | None,
    high_curvature_speed_limit_mps: float | None,
) -> float:
    if point.ref_id in stop_refs:
        return 0.0

    cap = max_speed_mps
    if point.ref_id in ref_speed_limits:
        cap = min(cap, ref_speed_limits[point.ref_id])
    for semantic_type, polygon in obstacle_zones:
        if semantic_type in zone_speed_limits and _point_in_polygon(point.x, point.y, polygon):
            cap = min(cap, zone_speed_limits[semantic_type])
    if (
        high_curvature_threshold_1pm is not None
        and high_curvature_speed_limit_mps is not None
        and abs(curvature) >= high_curvature_threshold_1pm
    ):
        cap = min(cap, high_curvature_speed_limit_mps)
    if abs(curvature) > 1e-9:
        cap = min(cap, math.sqrt(max_lateral_acceleration_mps2 / abs(curvature)))
    return max(0.0, cap)


def _apply_acceleration_limits(
    speed_caps: Sequence[float],
    distances: Sequence[float],
    max_acceleration_mps2: float,
    max_deceleration_mps2: float,
) -> list[float]:
    speeds = list(speed_caps)
    for index in range(1, len(speeds)):
        ds = distances[index] - distances[index - 1]
        if ds <= 1e-12:
            speeds[index] = min(speeds[index], speeds[index - 1])
            continue
        reachable = math.sqrt(speeds[index - 1] ** 2 + 2.0 * max_acceleration_mps2 * ds)
        speeds[index] = min(speeds[index], reachable)
    for index in range(len(speeds) - 2, -1, -1):
        ds = distances[index + 1] - distances[index]
        if ds <= 1e-12:
            speeds[index] = min(speeds[index], speeds[index + 1])
            continue
        reachable = math.sqrt(speeds[index + 1] ** 2 + 2.0 * max_deceleration_mps2 * ds)
        speeds[index] = min(speeds[index], reachable)
    return speeds


def _smooth_speed_profile_for_jerk(
    speed_limits: Sequence[float],
    distances: Sequence[float],
    *,
    max_acceleration_mps2: float,
    max_deceleration_mps2: float,
    max_jerk_mps3: float,
) -> list[float]:
    """Broaden local speed transitions until the sampled profile is jerk limited.

    The acceleration passes provide a fast feasible upper envelope. Repeated
    symmetric smoothing only lowers that envelope, so curvature, speed,
    acceleration, and deceleration constraints remain conservative while sharp
    acceleration corners become an S-shaped transition.
    """

    upper_envelope = list(speed_limits)
    speeds = list(speed_limits)
    for _ in range(5_000):
        _, _, jerks = _speed_profile_kinematics(distances, speeds)
        if max(abs(jerk) for jerk in jerks) <= max_jerk_mps3 + 1e-9:
            return speeds
        previous = speeds
        speeds = list(previous)
        for index in range(1, len(speeds) - 1):
            smoothed = (
                0.25 * previous[index - 1]
                + 0.50 * previous[index]
                + 0.25 * previous[index + 1]
            )
            speeds[index] = min(upper_envelope[index], smoothed)
        speeds = _apply_acceleration_limits(
            speeds,
            distances,
            max_acceleration_mps2,
            max_deceleration_mps2,
        )
    raise ValueError("continuous route could not satisfy the jerk limit")


def _speed_profile_kinematics(
    distances: Sequence[float],
    speeds: Sequence[float],
) -> tuple[list[float], list[float], list[float]]:
    times = _integrate_times(distances, speeds)
    accelerations = [0.0]
    for previous_speed, current_speed, previous_time, current_time in zip(
        speeds, speeds[1:], times, times[1:]
    ):
        accelerations.append(
            (current_speed - previous_speed) / (current_time - previous_time)
        )
    jerks = [0.0]
    for previous_acceleration, current_acceleration, previous_time, current_time in zip(
        accelerations,
        accelerations[1:],
        times,
        times[1:],
    ):
        jerks.append(
            (current_acceleration - previous_acceleration)
            / (current_time - previous_time)
        )
    return times, accelerations, jerks


def _integrate_times(distances: Sequence[float], speeds: Sequence[float]) -> list[float]:
    times = [0.0]
    for previous_index, current_index in zip(range(len(speeds) - 1), range(1, len(speeds))):
        ds = distances[current_index] - distances[previous_index]
        if ds <= 1e-12:
            times.append(times[-1])
            continue
        average_speed = (speeds[previous_index] + speeds[current_index]) * 0.5
        if average_speed <= 1e-12:
            raise ValueError("trajectory contains a non-moving interval with positive distance")
        times.append(times[-1] + ds / average_speed)
    return times


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
