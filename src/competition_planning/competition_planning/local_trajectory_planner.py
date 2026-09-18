"""Reference-aware short-horizon replanning over static and live obstacles."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time
from typing import Any, Iterable, Mapping, Sequence

from competition_planning.hybrid_astar_planner import (
    AsymmetricFootprint,
    HybridAStarPlanner,
    HybridAStarTimeout,
)
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


@dataclass(frozen=True)
class LocalReplanConfig:
    lookahead_distance_m: float = 5.0
    inflation_radius_m: float = 0.30
    search_padding_m: float = 3.0
    sample_spacing_m: float = 0.10
    min_turning_radius_m: float = 0.81
    step_length_m: float = 0.20
    curvature_bins: int = 9
    heading_bins: int = 72
    goal_position_tolerance_m: float = 0.25
    goal_heading_tolerance_rad: float = math.radians(15.0)
    reference_deviation_weight: float = 2.0
    max_expansions: int = 250_000
    planning_timeout_s: float = 2.0
    empty_history_planning_timeout_s: float = 2.0
    relaxed_extension_timeout_s: float = 0.75
    reference_search_window_points: int = 120
    relaxed_segment_entry_ref: str = ""
    relaxed_segment_exit_ref: str = ""
    relaxed_activation_distance_m: float = 0.0
    relaxed_reference_deviation_weight: float = 0.5
    relaxed_corridor_half_width_m: float = 0.85
    relaxed_step_length_m: float = 0.30
    relaxed_extension_curvature_bins: int = 7
    relaxed_goal_heading_tolerance_rad: float = math.radians(20.0)
    trajectory_switch_improvement_ratio: float = 0.15
    obstacle_clearance_distance_m: float = 0.0
    obstacle_clearance_weight: float = 0.0
    search_heuristic_weight: float = 1.0
    footprint: AsymmetricFootprint | None = None
    reference_priority_segment_entry_ref: str = ""
    reference_priority_segment_exit_ref: str = ""
    reference_priority_activation_distance_m: float = 0.0
    reference_priority_inflation_radius_m: float = 0.0
    reference_priority_footprint: AsymmetricFootprint | None = None
    obstacle_aware_heuristic: bool = False
    minimum_short_rejoin_distance_m: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.minimum_short_rejoin_distance_m) or not (
            0.0 <= self.minimum_short_rejoin_distance_m <= self.lookahead_distance_m
        ):
            raise ValueError("minimum short rejoin distance must fit inside lookahead")
        if self.lookahead_distance_m <= 0.0:
            raise ValueError("lookahead_distance_m must be positive")
        if self.reference_deviation_weight < 0.0:
            raise ValueError("reference_deviation_weight must be non-negative")
        if self.planning_timeout_s <= 0.0:
            raise ValueError("planning_timeout_s must be positive")
        if self.empty_history_planning_timeout_s <= 0.0:
            raise ValueError(
                "empty_history_planning_timeout_s must be positive"
            )
        if not 0.0 < self.relaxed_extension_timeout_s <= self.planning_timeout_s:
            raise ValueError(
                "relaxed_extension_timeout_s must be positive and no greater "
                "than planning_timeout_s"
            )
        if self.reference_search_window_points < 2:
            raise ValueError("reference_search_window_points must be at least 2")
        if bool(self.relaxed_segment_entry_ref) != bool(self.relaxed_segment_exit_ref):
            raise ValueError("relaxed segment entry and exit refs must be configured together")
        if self.relaxed_activation_distance_m < 0.0:
            raise ValueError("relaxed_activation_distance_m must be non-negative")
        if self.relaxed_reference_deviation_weight < 0.0:
            raise ValueError("relaxed_reference_deviation_weight must be non-negative")
        if self.relaxed_corridor_half_width_m <= 0.0:
            raise ValueError("relaxed_corridor_half_width_m must be positive")
        if self.relaxed_step_length_m <= 0.0:
            raise ValueError("relaxed_step_length_m must be positive")
        if (
            self.relaxed_extension_curvature_bins < 3
            or self.relaxed_extension_curvature_bins % 2 == 0
        ):
            raise ValueError(
                "relaxed_extension_curvature_bins must be an odd integer >= 3"
            )
        if self.relaxed_goal_heading_tolerance_rad <= 0.0:
            raise ValueError("relaxed_goal_heading_tolerance_rad must be positive")
        if not 0.0 <= self.trajectory_switch_improvement_ratio < 1.0:
            raise ValueError("trajectory_switch_improvement_ratio must be in [0, 1)")
        if self.obstacle_clearance_distance_m < 0.0:
            raise ValueError("obstacle_clearance_distance_m must be non-negative")
        if self.obstacle_clearance_weight < 0.0:
            raise ValueError("obstacle_clearance_weight must be non-negative")
        if self.search_heuristic_weight < 1.0:
            raise ValueError("search_heuristic_weight must be at least 1.0")
        if bool(self.reference_priority_segment_entry_ref) != bool(
            self.reference_priority_segment_exit_ref
        ):
            raise ValueError(
                "reference-priority segment entry and exit refs must be "
                "configured together"
            )
        if self.reference_priority_activation_distance_m < 0.0:
            raise ValueError(
                "reference_priority_activation_distance_m must be non-negative"
            )
        if self.reference_priority_inflation_radius_m < 0.0:
            raise ValueError(
                "reference_priority_inflation_radius_m must be non-negative"
            )


@dataclass(frozen=True)
class LocalPlan:
    path: tuple[PathPoint, ...]
    reference_start_index: int
    rejoin_index: int
    dynamic_obstacle_count: int
    status: str
    path_is_navigable: bool
    planning_grid_cell_count: int


class LocalTrajectoryPlanner:
    """Produce one local path that stays near and rejoins a global reference."""

    def __init__(
        self,
        static_map: OccupancyGridMap,
        config: LocalReplanConfig,
    ) -> None:
        self._static_map = static_map
        self._config = config
        self._held_relaxed_path: tuple[PathPoint, ...] = ()
        self._held_rejoin_index: int | None = None
        self._last_safe_path: tuple[PathPoint, ...] = ()
        self._last_safe_rejoin_index: int | None = None
        self._pending_relaxed_span: tuple[int, int] | None = None
        self._completed_relaxed_span: tuple[int, int] | None = None

    def plan(
        self,
        *,
        reference_path: Sequence[PathPoint],
        current_pose: PathPoint,
        dynamic_obstacle_points: Iterable[tuple[float, float]],
        reference_priority_obstacle_points: (
            Iterable[tuple[float, float]] | None
        ) = None,
        previous_reference_index: int = 0,
        departure_context: bool = False,
    ) -> LocalPlan:
        if len(reference_path) < 2:
            raise ValueError("local replanning requires at least two reference points")

        start_index = _nearest_reference_index(
            reference_path,
            current_pose,
            previous_reference_index=previous_reference_index,
            search_window_points=self._config.reference_search_window_points,
        )
        detected_relaxed_span = _active_relaxed_segment(
            reference_path,
            start_index=start_index,
            entry_ref=self._config.relaxed_segment_entry_ref,
            exit_ref=self._config.relaxed_segment_exit_ref,
            activation_distance_m=self._config.relaxed_activation_distance_m,
            planning_horizon_m=self._config.lookahead_distance_m,
        )
        reference_priority_span = _active_reference_priority_segment(
            reference_path,
            start_index=start_index,
            entry_ref=self._config.reference_priority_segment_entry_ref,
            exit_ref=self._config.reference_priority_segment_exit_ref,
            activation_distance_m=(
                self._config.reference_priority_activation_distance_m
            ),
        )
        if (
            detected_relaxed_span is not None
            and detected_relaxed_span == self._completed_relaxed_span
            and math.hypot(
                reference_path[detected_relaxed_span[1]].x - current_pose.x,
                reference_path[detected_relaxed_span[1]].y - current_pose.y,
            )
            <= self._config.goal_position_tolerance_m + 1e-9
        ):
            detected_relaxed_span = None
        if detected_relaxed_span is not None:
            self._pending_relaxed_span = detected_relaxed_span
            if detected_relaxed_span != self._completed_relaxed_span:
                self._completed_relaxed_span = None
        relaxed_span = detected_relaxed_span or self._pending_relaxed_span
        checkpoint_advance_distance_m = max(
            self._config.relaxed_activation_distance_m,
            self._config.relaxed_step_length_m,
            2.0 * self._config.goal_position_tolerance_m,
        )
        held_rejoin_index = self._held_rejoin_index
        advance_held_checkpoint = held_rejoin_index is not None and (
            math.hypot(
                reference_path[held_rejoin_index].x - current_pose.x,
                reference_path[held_rejoin_index].y - current_pose.y,
            )
            <= checkpoint_advance_distance_m + 1e-9
        )
        planning_horizon_m = self._config.lookahead_distance_m
        if departure_context:
            # A rear-dock departure only needs to reach a collision-free pose
            # beyond the shelf before normal rolling replanning resumes.  Do
            # not couple that first constrained search to a distant checkpoint
            # pose; two turning radii retain enough forward-only recovery room.
            planning_horizon_m = min(
                planning_horizon_m,
                2.0 * self._config.min_turning_radius_m,
            )
        rolling_rejoin_index = _lookahead_index(
            reference_path,
            start_index,
            planning_horizon_m,
        )
        if relaxed_span is None:
            rejoin_index = rolling_rejoin_index
        else:
            segment_entry_index, segment_exit_index = relaxed_span
            segment_exit = reference_path[segment_exit_index]
            advance_segment_exit = (
                start_index >= segment_exit_index
                or math.hypot(
                    segment_exit.x - current_pose.x,
                    segment_exit.y - current_pose.y,
                )
                <= checkpoint_advance_distance_m + 1e-9
            )
            held_rejoin_index = self._held_rejoin_index
            if (
                held_rejoin_index is not None
                and start_index < held_rejoin_index
                and not advance_held_checkpoint
            ):
                rejoin_index = held_rejoin_index
            elif (
                advance_held_checkpoint
                and held_rejoin_index is not None
                and held_rejoin_index < segment_exit_index
            ):
                rejoin_index = segment_exit_index
            elif advance_held_checkpoint or advance_segment_exit:
                rejoin_index = rolling_rejoin_index
            else:
                distance_to_entry_m = sum(
                    math.hypot(
                        reference_path[index].x - reference_path[index - 1].x,
                        reference_path[index].y - reference_path[index - 1].y,
                    )
                    for index in range(start_index + 1, segment_entry_index + 1)
                )
                use_approach_checkpoint = (
                    start_index < segment_entry_index < rolling_rejoin_index
                    and distance_to_entry_m
                    > self._config.relaxed_activation_distance_m + 1e-9
                )
                rejoin_index = (
                    min(segment_exit_index, rolling_rejoin_index)
                    if use_approach_checkpoint
                    else segment_exit_index
                )
        if rejoin_index <= start_index:
            if (
                start_index == len(reference_path) - 1
                and rejoin_index == start_index
            ):
                # A checkpoint-gated reference intentionally ends at the active
                # checkpoint.  Keep its final segment available so the controller
                # can capture and hold the endpoint while obstacle validation still
                # runs through the normal planning path below.
                start_index -= 1
            else:
                raise ValueError(
                    "global reference has no forward local replanning horizon"
                )

        local_reference = tuple(reference_path[start_index : rejoin_index + 1])
        planning_map = _crop_planning_map(
            self._static_map,
            current_pose,
            local_reference,
            margin_m=(
                self._config.search_padding_m
                + self._config.inflation_radius_m
                + self._config.step_length_m
            ),
        )
        dynamic_obstacle_points = tuple(dynamic_obstacle_points)
        if (
            reference_priority_span is not None
            and reference_priority_obstacle_points is not None
            and not departure_context
        ):
            priority_map, priority_obstacle_count = _overlay_obstacles(
                planning_map,
                reference_priority_obstacle_points,
            )
            priority_config = replace(
                self._config,
                inflation_radius_m=(
                    self._config.reference_priority_inflation_radius_m
                ),
                footprint=self._config.reference_priority_footprint,
                obstacle_clearance_distance_m=0.0,
                obstacle_clearance_weight=0.0,
            )
            priority_planner = self._planner(
                priority_map,
                local_reference,
                config_override=priority_config,
            )
            if priority_planner.path_is_navigable(local_reference):
                self._held_relaxed_path = ()
                self._held_rejoin_index = None
                self._remember_safe_path(local_reference, rejoin_index)
                return LocalPlan(
                    path=local_reference,
                    reference_start_index=start_index,
                    rejoin_index=rejoin_index,
                    dynamic_obstacle_count=priority_obstacle_count,
                    status="REFERENCE_PRIORITY_CLEAR",
                    path_is_navigable=True,
                    planning_grid_cell_count=(
                        priority_map.width * priority_map.height
                    ),
                )
        live_map, obstacle_count = _overlay_obstacles(
            planning_map,
            dynamic_obstacle_points,
        )
        relaxed = relaxed_span is not None
        blocked_relaxed_rejoin = False
        if relaxed:
            stable_clear_distance_m = (
                self._config.relaxed_activation_distance_m
                + self._config.relaxed_step_length_m
            )
            selection_limit_index = _lookahead_index(
                reference_path,
                rejoin_index,
                stable_clear_distance_m,
            )
            selection_reference = tuple(
                reference_path[start_index : selection_limit_index + 1]
            )
            selection_planner = self._planner(
                live_map,
                selection_reference,
                relaxed=True,
            )
            held_path_is_safe = (
                bool(
                    self._safe_held_path(
                        current_pose=current_pose,
                        planner=selection_planner,
                        rejoin_index=held_rejoin_index,
                    )
                )
                if held_rejoin_index is not None
                else False
            )
            if not held_path_is_safe or advance_held_checkpoint:
                dynamic_rejoin_index = _relaxed_rejoin_after_blockage(
                    reference_path,
                    selection_planner,
                    start_index=start_index,
                    nominal_rejoin_index=rejoin_index,
                    maximum_rejoin_index=selection_limit_index,
                    stable_clear_distance_m=stable_clear_distance_m,
                )
                if dynamic_rejoin_index != rejoin_index:
                    rejoin_index = dynamic_rejoin_index
                    local_reference = tuple(
                        reference_path[start_index : rejoin_index + 1]
                    )
                    blocked_relaxed_rejoin = True
        extension_timeout_s = (
            self._config.relaxed_extension_timeout_s
            if (
                relaxed
                and held_rejoin_index is not None
                and rejoin_index != held_rejoin_index
            )
            else None
        )
        planner = self._planner(
            live_map,
            local_reference,
            relaxed=relaxed,
            planning_timeout_s=extension_timeout_s,
            reduced_curvature_lattice=(
                blocked_relaxed_rejoin or extension_timeout_s is not None
            ),
        )
        selected_rejoin_index = _select_navigable_rejoin_index(
            reference_path,
            planner,
            start_index=start_index,
            preferred_rejoin_index=rejoin_index,
        )
        if selected_rejoin_index != rejoin_index:
            rejoin_index = selected_rejoin_index
            local_reference = tuple(reference_path[start_index : rejoin_index + 1])
            extension_timeout_s = (
                self._config.relaxed_extension_timeout_s
                if (
                    relaxed
                    and held_rejoin_index is not None
                    and rejoin_index != held_rejoin_index
                )
                else None
            )
            planner = self._planner(
                live_map,
                local_reference,
                relaxed=relaxed,
                planning_timeout_s=extension_timeout_s,
                reduced_curvature_lattice=(
                    blocked_relaxed_rejoin or extension_timeout_s is not None
                ),
            )
        fallback_rejoin_index = self._held_rejoin_index
        fallback_held_path = (
            self._safe_held_path(
                current_pose=current_pose,
                planner=planner,
                rejoin_index=fallback_rejoin_index,
            )
            if relaxed and fallback_rejoin_index is not None
            else ()
        )
        if extension_timeout_s is not None and not fallback_held_path:
            planner = self._planner(
                live_map,
                local_reference,
                relaxed=relaxed,
                reduced_curvature_lattice=blocked_relaxed_rejoin,
            )
        transitioning_from_relaxed = False
        if relaxed and _relaxed_exit_is_ready(
            reference_path,
            current_pose=current_pose,
            start_index=start_index,
            segment_exit_index=relaxed_span[1],
            planner=planner,
            position_tolerance_m=max(
                self._config.goal_position_tolerance_m,
                self._config.relaxed_step_length_m,
            ),
            heading_tolerance_rad=max(
                self._config.goal_heading_tolerance_rad,
                self._config.relaxed_goal_heading_tolerance_rad,
            ),
            clear_distance_m=self._config.relaxed_activation_distance_m,
        ):
            transitioning_from_relaxed = True
            relaxed = False
            rejoin_index = rolling_rejoin_index
            local_reference = tuple(reference_path[start_index : rejoin_index + 1])
            planner = self._planner(live_map, local_reference, relaxed=False)
            selected_rejoin_index = _select_navigable_rejoin_index(
                reference_path,
                planner,
                start_index=start_index,
                preferred_rejoin_index=rejoin_index,
            )
            if selected_rejoin_index != rejoin_index:
                rejoin_index = selected_rejoin_index
                local_reference = tuple(
                    reference_path[start_index : rejoin_index + 1]
                )
                planner = self._planner(live_map, local_reference, relaxed=False)
        if relaxed:
            held_path = (
                fallback_held_path
                if fallback_rejoin_index == rejoin_index
                else self._safe_held_path(
                    current_pose=current_pose,
                    planner=planner,
                    rejoin_index=rejoin_index,
                )
            )
            if held_path:
                if relaxed_span is not None:
                    segment_entry_index, segment_exit_index = relaxed_span
                    distance_from_entry_m = sum(
                        math.hypot(
                            reference_path[index].x
                            - reference_path[index - 1].x,
                            reference_path[index].y
                            - reference_path[index - 1].y,
                        )
                        for index in range(
                            segment_entry_index + 1,
                            start_index + 1,
                        )
                    )
                    if (
                        distance_from_entry_m
                        >= self._config.relaxed_activation_distance_m - 1e-9
                    ):
                        spliced_path = self._splice_held_path_to_reference(
                            held_path=held_path,
                            planner=planner,
                            reference_path=reference_path,
                            start_index=start_index,
                            segment_exit_index=segment_exit_index,
                        )
                        if spliced_path:
                            self._held_relaxed_path = spliced_path
                            self._remember_safe_path(spliced_path, rejoin_index)
                            return LocalPlan(
                                path=spliced_path,
                                reference_start_index=start_index,
                                rejoin_index=rejoin_index,
                                dynamic_obstacle_count=obstacle_count,
                                status="EARLY_REJOIN_SPLICED",
                                path_is_navigable=True,
                                planning_grid_cell_count=(
                                    live_map.width * live_map.height
                                ),
                            )
                self._held_relaxed_path = held_path
                self._remember_safe_path(held_path, rejoin_index)
                return LocalPlan(
                    path=held_path,
                    reference_start_index=start_index,
                    rejoin_index=rejoin_index,
                    dynamic_obstacle_count=obstacle_count,
                    status="RELAXED_HOLD",
                    path_is_navigable=True,
                    planning_grid_cell_count=live_map.width * live_map.height,
                )
        if not relaxed and planner.path_is_navigable(local_reference):
            if transitioning_from_relaxed:
                self._completed_relaxed_span = relaxed_span
                self._pending_relaxed_span = None
            self._held_relaxed_path = ()
            self._held_rejoin_index = None
            self._remember_safe_path(local_reference, rejoin_index)
            return LocalPlan(
                path=local_reference,
                reference_start_index=start_index,
                rejoin_index=rejoin_index,
                dynamic_obstacle_count=obstacle_count,
                status="REFERENCE_CLEAR",
                path_is_navigable=True,
                planning_grid_cell_count=live_map.width * live_map.height,
            )

        try:
            (
                path,
                rejoin_index,
                local_reference,
                planner,
                short_rejoin_used,
            ) = self._plan_with_short_rejoin(
                reference_path=reference_path,
                current_pose=current_pose,
                live_map=live_map,
                start_index=start_index,
                rejoin_index=rejoin_index,
                local_reference=local_reference,
                planner=planner,
                relaxed=relaxed,
                reduced_curvature_lattice=(
                    blocked_relaxed_rejoin or extension_timeout_s is not None
                ),
            )
        except GridPlanningError as exc:
            held_path = (
                fallback_held_path
                if relaxed or transitioning_from_relaxed
                else ()
            )
            held_rejoin_index = fallback_rejoin_index
            held_status = "RELAXED_HOLD"
            if not held_path:
                held_path = self._safe_last_path(
                    current_pose=current_pose,
                    planner=planner,
                )
                held_rejoin_index = self._last_safe_rejoin_index
                held_status = (
                    "PLANNING_TIMEOUT_HOLD"
                    if isinstance(exc, HybridAStarTimeout)
                    else "SAFE_PATH_HOLD"
                )
            if not held_path:
                raise
            self._held_relaxed_path = held_path
            self._remember_safe_path(
                held_path,
                held_rejoin_index if held_rejoin_index is not None else rejoin_index,
            )
            return LocalPlan(
                path=held_path,
                reference_start_index=start_index,
                rejoin_index=(
                    held_rejoin_index
                    if held_rejoin_index is not None
                    else rejoin_index
                ),
                dynamic_obstacle_count=obstacle_count,
                status=held_status,
                path_is_navigable=True,
                planning_grid_cell_count=live_map.width * live_map.height,
            )
        status = "REPLANNED_SHORT_REJOIN" if short_rejoin_used else "REPLANNED"
        if relaxed:
            held_path = (
                fallback_held_path
                if fallback_rejoin_index == rejoin_index
                else self._safe_held_path(
                    current_pose=current_pose,
                    planner=planner,
                    rejoin_index=rejoin_index,
                )
            )
            if (
                held_path
                and fallback_rejoin_index == rejoin_index
                and not _path_is_materially_better(
                    path,
                    held_path,
                    reference_path=local_reference,
                    reference_deviation_weight=(
                        self._config.relaxed_reference_deviation_weight
                    ),
                    improvement_ratio=(
                        self._config.trajectory_switch_improvement_ratio
                    ),
                )
            ):
                path = held_path
                status = "RELAXED_HOLD"
            else:
                status = "RELAXED_REPLANNED"
            self._held_relaxed_path = tuple(path)
            self._held_rejoin_index = rejoin_index
        else:
            if transitioning_from_relaxed:
                self._completed_relaxed_span = relaxed_span
                self._pending_relaxed_span = None
            self._held_relaxed_path = ()
            self._held_rejoin_index = None
        self._remember_safe_path(path, rejoin_index)
        return LocalPlan(
            path=path,
            reference_start_index=start_index,
            rejoin_index=rejoin_index,
            dynamic_obstacle_count=obstacle_count,
            status=status,
            path_is_navigable=planner.path_is_navigable(path),
            planning_grid_cell_count=live_map.width * live_map.height,
        )

    def _splice_held_path_to_reference(
        self,
        *,
        held_path: tuple[PathPoint, ...],
        planner: HybridAStarPlanner,
        reference_path: Sequence[PathPoint],
        start_index: int,
        segment_exit_index: int,
    ) -> tuple[PathPoint, ...]:
        """Opportunistically join a safe held path to the clear reference."""

        reference_start_index = start_index + 1
        if len(held_path) < 2 or reference_start_index >= segment_exit_index:
            return ()
        maximum_join_distance_m = max(
            self._config.relaxed_step_length_m,
            2.0 * self._config.goal_position_tolerance_m,
        )
        reference_indexes = range(reference_start_index, segment_exit_index)
        for held_index in range(1, len(held_path)):
            held_point = held_path[held_index]
            for reference_index in reference_indexes:
                reference_point = reference_path[reference_index]
                join_distance_m = math.hypot(
                    reference_point.x - held_point.x,
                    reference_point.y - held_point.y,
                )
                if join_distance_m > maximum_join_distance_m + 1e-9:
                    continue
                if (
                    abs(_wrap_angle(reference_point.yaw - held_point.yaw))
                    > self._config.relaxed_goal_heading_tolerance_rad + 1e-9
                ):
                    continue
                next_reference = reference_path[reference_index + 1]
                if not _splice_respects_turning_radius(
                    (
                        held_path[held_index - 1],
                        held_point,
                        reference_point,
                    ),
                    min_turning_radius_m=self._config.min_turning_radius_m,
                ):
                    continue
                if not _splice_respects_turning_radius(
                    (held_point, reference_point, next_reference),
                    min_turning_radius_m=self._config.min_turning_radius_m,
                ):
                    continue
                reference_tail_index = reference_index
                if join_distance_m <= 1e-9:
                    if not _splice_respects_turning_radius(
                        (
                            held_path[held_index - 1],
                            held_point,
                            next_reference,
                        ),
                        min_turning_radius_m=self._config.min_turning_radius_m,
                    ):
                        continue
                    reference_tail_index += 1
                bridge_sample_count = max(
                    1,
                    math.ceil(join_distance_m / self._config.sample_spacing_m),
                )
                bridge = tuple(
                    PathPoint(
                        x=held_point.x
                        + (reference_point.x - held_point.x)
                        * sample_index
                        / bridge_sample_count,
                        y=held_point.y
                        + (reference_point.y - held_point.y)
                        * sample_index
                        / bridge_sample_count,
                        yaw=held_point.yaw,
                    )
                    for sample_index in range(1, bridge_sample_count + 1)
                )
                candidate = (
                    *held_path[: held_index + 1],
                    *reference_path[
                        reference_tail_index : segment_exit_index + 1
                    ],
                )
                if planner.path_is_navigable(bridge) and planner.path_is_navigable(
                    candidate
                ):
                    return tuple(candidate)
        return ()

    def _safe_held_path(
        self,
        *,
        current_pose: PathPoint,
        planner: HybridAStarPlanner,
        rejoin_index: int,
    ) -> tuple[PathPoint, ...]:
        held_tail = self._safe_held_tail(
            current_pose=current_pose,
            planner=planner,
            rejoin_index=rejoin_index,
        )
        if not held_tail:
            return ()
        candidate = (current_pose, *held_tail[1:])
        if not _splice_respects_turning_radius(
            candidate,
            min_turning_radius_m=self._config.min_turning_radius_m,
        ):
            return ()
        return candidate

    def _safe_held_tail(
        self,
        *,
        current_pose: PathPoint,
        planner: HybridAStarPlanner,
        rejoin_index: int,
    ) -> tuple[PathPoint, ...]:
        """Return held geometry only while it remains collision-free."""

        if self._held_rejoin_index != rejoin_index:
            return ()
        return self._safe_path_tail(
            self._held_relaxed_path,
            current_pose=current_pose,
            planner=planner,
        )

    def _safe_last_path(
        self,
        *,
        current_pose: PathPoint,
        planner: HybridAStarPlanner,
    ) -> tuple[PathPoint, ...]:
        last_tail = self._safe_path_tail(
            self._last_safe_path,
            current_pose=current_pose,
            planner=planner,
        )
        if not last_tail:
            return ()
        candidate = (current_pose, *last_tail[1:])
        if not _splice_respects_turning_radius(
            candidate,
            min_turning_radius_m=self._config.min_turning_radius_m,
        ):
            return ()
        return candidate

    def _safe_path_tail(
        self,
        path: tuple[PathPoint, ...],
        *,
        current_pose: PathPoint,
        planner: HybridAStarPlanner,
    ) -> tuple[PathPoint, ...]:
        if not path:
            return ()
        nearest_index = min(
            range(len(path)),
            key=lambda index: math.hypot(
                path[index].x - current_pose.x,
                path[index].y - current_pose.y,
            ),
        )
        nearest = path[nearest_index]
        if math.hypot(nearest.x - current_pose.x, nearest.y - current_pose.y) > 0.50:
            return ()
        held_tail = path[nearest_index:]
        if len(held_tail) < 2 or not planner.path_is_navigable(
            (current_pose, *held_tail)
        ):
            return ()
        return held_tail

    def _plan_with_short_rejoin(
        self,
        *,
        reference_path: Sequence[PathPoint],
        current_pose: PathPoint,
        live_map: OccupancyGridMap,
        start_index: int,
        rejoin_index: int,
        local_reference: tuple[PathPoint, ...],
        planner: HybridAStarPlanner,
        relaxed: bool,
        reduced_curvature_lattice: bool,
    ) -> tuple[
        tuple[PathPoint, ...],
        int,
        tuple[PathPoint, ...],
        HybridAStarPlanner,
        bool,
    ]:
        start_pose = PathPoint(
            current_pose.x,
            current_pose.y,
            current_pose.yaw,
            ref_id=local_reference[0].ref_id,
        )
        attempts: list[dict[str, object]] = []

        def attempt(search_planner, search_reference, index, timeout_s):
            started = time.perf_counter()
            record = {
                "rejoin_index": index,
                "goal": [search_reference[-1].x, search_reference[-1].y],
                "timeout_s": timeout_s,
            }
            attempts.append(record)
            try:
                candidate_path = tuple(
                    search_planner.plan((start_pose, search_reference[-1]))
                )
                if (
                    index != rejoin_index
                    and self._config.minimum_short_rejoin_distance_m > 0.0
                ):
                    length_m = sum(
                        math.hypot(b.x - a.x, b.y - a.y)
                        for a, b in zip(candidate_path, candidate_path[1:])
                    )
                    if length_m + 1e-9 < self._config.minimum_short_rejoin_distance_m:
                        raise GridPlanningError(
                            f"short rejoin path covers only {length_m:.3f} m; "
                            f"requires {self._config.minimum_short_rejoin_distance_m:.3f} m"
                        )
                return candidate_path
            except GridPlanningError as exc:
                record["error_type"] = type(exc).__name__
                record["detail"] = str(exc)
                raise
            finally:
                record["elapsed_ms"] = (time.perf_counter() - started) * 1000

        try:
            return (
                attempt(planner, local_reference, rejoin_index, None),
                rejoin_index,
                local_reference,
                planner,
                False,
            )
        except GridPlanningError as exc:
            primary_error = exc
            last_error = exc

        # A full rolling horizon can become unreachable after a legitimate
        # turn-in lag. Try shorter, still collision-checked rejoin horizons.
        candidate_indexes: list[int] = []
        for distance_m in (2.0, 1.2):
            if distance_m < self._config.minimum_short_rejoin_distance_m:
                continue
            candidate_index = _lookahead_index(
                reference_path,
                start_index,
                min(distance_m, self._config.lookahead_distance_m),
            )
            if (
                start_index < candidate_index < rejoin_index
                and candidate_index not in candidate_indexes
            ):
                candidate_indexes.append(candidate_index)

        for candidate_index in candidate_indexes:
            candidate_reference = tuple(
                reference_path[start_index : candidate_index + 1]
            )
            candidate_planner = self._planner(
                live_map,
                candidate_reference,
                relaxed=relaxed,
                planning_timeout_s=0.10,
                reduced_curvature_lattice=reduced_curvature_lattice,
            )
            try:
                return (
                    attempt(
                        candidate_planner, candidate_reference, candidate_index, 0.10
                    ),
                    candidate_index,
                    candidate_reference,
                    candidate_planner,
                    True,
                )
            except GridPlanningError as exc:
                last_error = exc

        error = (
            primary_error
            if self._config.minimum_short_rejoin_distance_m > 0.0
            else last_error
        )
        error.rejoin_attempts = attempts
        raise error

    def _remember_safe_path(
        self,
        path: Sequence[PathPoint],
        rejoin_index: int,
    ) -> None:
        self._last_safe_path = tuple(path)
        self._last_safe_rejoin_index = rejoin_index

    def _planner(
        self,
        grid_map: OccupancyGridMap,
        reference_path: Sequence[PathPoint],
        *,
        relaxed: bool = False,
        planning_timeout_s: float | None = None,
        reduced_curvature_lattice: bool = False,
        config_override: LocalReplanConfig | None = None,
    ) -> HybridAStarPlanner:
        config = self._config if config_override is None else config_override
        default_timeout_s = (
            max(
                config.planning_timeout_s,
                config.empty_history_planning_timeout_s,
            )
            if not self._last_safe_path
            else config.planning_timeout_s
        )
        return HybridAStarPlanner(
            grid_map,
            inflation_radius_m=config.inflation_radius_m,
            search_padding_m=config.search_padding_m,
            sample_spacing_m=config.sample_spacing_m,
            min_turning_radius_m=config.min_turning_radius_m,
            step_length_m=(
                config.relaxed_step_length_m if relaxed else config.step_length_m
            ),
            curvature_bins=(
                config.relaxed_extension_curvature_bins
                if relaxed and reduced_curvature_lattice
                else config.curvature_bins
            ),
            heading_bins=config.heading_bins,
            goal_position_tolerance_m=config.goal_position_tolerance_m,
            goal_heading_tolerance_rad=(
                config.relaxed_goal_heading_tolerance_rad
                if relaxed
                else config.goal_heading_tolerance_rad
            ),
            max_expansions=config.max_expansions,
            planning_timeout_s=(
                default_timeout_s
                if planning_timeout_s is None
                else planning_timeout_s
            ),
            reference_path=reference_path,
            reference_deviation_weight=(
                config.relaxed_reference_deviation_weight
                if relaxed
                else config.reference_deviation_weight
            ),
            corridor_half_width_m=(
                config.relaxed_corridor_half_width_m if relaxed else None
            ),
            obstacle_clearance_distance_m=config.obstacle_clearance_distance_m,
            obstacle_clearance_weight=config.obstacle_clearance_weight,
            search_heuristic_weight=config.search_heuristic_weight,
            footprint=config.footprint,
            obstacle_aware_heuristic=config.obstacle_aware_heuristic,
        )


def reference_prefix_to_checkpoint(
    reference_path: Sequence[PathPoint],
    checkpoint: PathPoint,
    *,
    exact_pose: bool = True,
    minimum_index: int = 0,
) -> tuple[PathPoint, ...]:
    """Expose the next active checkpoint gate after current route progress."""

    if not checkpoint.ref_id:
        raise ValueError("active checkpoint requires a ref_id")
    if minimum_index < 0:
        raise ValueError("minimum_index must be non-negative")
    for index in range(minimum_index, len(reference_path)):
        point = reference_path[index]
        if point.ref_id != checkpoint.ref_id:
            continue
        if index < 1:
            raise ValueError(
                f"active checkpoint {checkpoint.ref_id!r} leaves fewer than two points"
            )
        if exact_pose:
            return tuple(reference_path[:index]) + (checkpoint,)
        return tuple(reference_path[: index + 1])
    raise ValueError(
        "active checkpoint ref not found at or after route index "
        f"{minimum_index}: {checkpoint.ref_id}"
    )


def docking_mode_is_active(
    *,
    current_pose: PathPoint,
    checkpoint: PathPoint | None,
    active_checkpoint_ref: str | None,
    docking_refs: set[str],
    activation_distance_m: float,
) -> bool:
    """Return whether precision docking collision geometry should be active."""

    if activation_distance_m <= 0.0:
        raise ValueError("docking activation distance must be positive")
    if (
        checkpoint is None
        or active_checkpoint_ref is None
        or active_checkpoint_ref not in docking_refs
    ):
        return False
    return (
        math.hypot(checkpoint.x - current_pose.x, checkpoint.y - current_pose.y)
        <= activation_distance_m + 1e-9
    )


def docking_context_checkpoint(
    *,
    current_pose: PathPoint,
    active_checkpoint: PathPoint | None,
    active_checkpoint_ref: str | None,
    departure_checkpoints: Mapping[str, PathPoint],
    docking_refs: set[str],
    activation_distance_m: float,
) -> tuple[PathPoint | None, str | None]:
    """Choose dock collision geometry without adding a motion-state transition."""

    if docking_mode_is_active(
        current_pose=current_pose,
        checkpoint=active_checkpoint,
        active_checkpoint_ref=active_checkpoint_ref,
        docking_refs=docking_refs,
        activation_distance_m=activation_distance_m,
    ):
        return active_checkpoint, active_checkpoint_ref

    nearby_departures = tuple(
        (math.hypot(point.x - current_pose.x, point.y - current_pose.y), ref, point)
        for ref, point in departure_checkpoints.items()
        if math.hypot(point.x - current_pose.x, point.y - current_pose.y)
        <= activation_distance_m + 1e-9
    )
    if not nearby_departures:
        return None, None
    _, departure_ref, departure_checkpoint = min(
        nearby_departures,
        key=lambda item: item[0],
    )
    return departure_checkpoint, departure_ref


def precision_docking_work_sides(
    dock_records: Iterable[Any],
) -> dict[str, str]:
    """Return only pickup/drop work-side semantics, excluding finish parking."""

    return {
        str(record["point_ref"]): str(record.get("work_side", "RIGHT")).upper()
        for record in dock_records
        if isinstance(record, Mapping)
        and record.get("point_ref")
        and str(record.get("dock_type", "")).upper() in {"PICKUP", "DROP"}
    }


def filter_expected_docking_shelf_points(
    points: Iterable[tuple[float, float]],
    *,
    checkpoint: PathPoint,
    work_side: str,
    vehicle_length_m: float,
    vehicle_width_m: float,
    front_clearance_m: float,
    approach_distance_m: float,
    physical_guard_m: float,
) -> tuple[tuple[tuple[float, float], ...], int]:
    """Ignore expected shelf-face echoes outside the final physical sweep.

    The static map still contains the shelf.  This filter only removes live
    shelf returns on the declared work side when they are farther from the
    docking centerline than the physical half-width plus a guard.  Obstacles
    inside that guard, on the non-work side, or beyond the final approach
    corridor remain available to collision checking.
    """

    normalized_side = work_side.upper()
    if normalized_side not in {"LEFT", "RIGHT"}:
        raise ValueError(f"unsupported docking work_side: {work_side}")
    if vehicle_length_m <= 0.0 or vehicle_width_m <= 0.0:
        raise ValueError("docking vehicle dimensions must be positive")
    if front_clearance_m < 0.0 or approach_distance_m <= 0.0:
        raise ValueError("docking approach geometry is invalid")
    if physical_guard_m < 0.0:
        raise ValueError("docking physical guard must be non-negative")

    cos_yaw = math.cos(checkpoint.yaw)
    sin_yaw = math.sin(checkpoint.yaw)
    half_length_m = 0.5 * vehicle_length_m
    protected_lateral_m = 0.5 * vehicle_width_m + physical_guard_m
    minimum_longitudinal_m = -approach_distance_m - half_length_m
    maximum_longitudinal_m = half_length_m + front_clearance_m
    kept: list[tuple[float, float]] = []
    removed_count = 0
    for x, y in points:
        delta_x = x - checkpoint.x
        delta_y = y - checkpoint.y
        longitudinal_m = cos_yaw * delta_x + sin_yaw * delta_y
        lateral_m = -sin_yaw * delta_x + cos_yaw * delta_y
        on_work_side = (
            lateral_m >= protected_lateral_m
            if normalized_side == "LEFT"
            else lateral_m <= -protected_lateral_m
        )
        in_final_corridor = (
            minimum_longitudinal_m
            <= longitudinal_m
            <= maximum_longitudinal_m
        )
        if on_work_side and in_final_corridor:
            removed_count += 1
            continue
        kept.append((x, y))
    return tuple(kept), removed_count


def docking_shelf_filter_is_active(
    *,
    current_pose: PathPoint,
    checkpoint: PathPoint,
    activation_distance_m: float,
    heading_tolerance_rad: float,
) -> bool:
    """Limit shelf filtering to an aligned final precision approach."""

    if activation_distance_m <= 0.0:
        raise ValueError("docking shelf filter distance must be positive")
    if heading_tolerance_rad < 0.0:
        raise ValueError("docking shelf filter heading tolerance must be non-negative")
    return (
        math.hypot(checkpoint.x - current_pose.x, checkpoint.y - current_pose.y)
        <= activation_distance_m + 1e-9
        and abs(_wrap_angle(current_pose.yaw - checkpoint.yaw))
        <= heading_tolerance_rad + 1e-9
    )


def concatenate_reference_paths(
    paths: Sequence[Sequence[PathPoint]],
    *,
    connection_tolerance_m: float = 0.15,
    connection_heading_tolerance_rad: float = math.radians(10.0),
    connection_blend_distance_m: float = 0.80,
    preserve_next_path: bool = False,
) -> tuple[PathPoint, ...]:
    """Join route segments into one monotonic local-planning reference."""

    if (
        connection_tolerance_m < 0.0
        or connection_heading_tolerance_rad < 0.0
        or connection_blend_distance_m <= 0.0
    ):
        raise ValueError(
            "reference connection tolerances and blend distance are invalid"
        )
    if not paths:
        raise ValueError("at least one reference segment is required")
    if any(len(path) < 2 for path in paths):
        raise ValueError("every reference segment requires at least two points")

    joined = list(paths[0])
    for path in paths[1:]:
        previous = joined[-1]
        current = path[0]
        position_gap_m = math.hypot(previous.x - current.x, previous.y - current.y)
        heading_gap_rad = abs(_wrap_angle(previous.yaw - current.yaw))
        if (
            position_gap_m > connection_tolerance_m
            or heading_gap_rad > connection_heading_tolerance_rad
        ):
            raise ValueError(
                "reference segments are disconnected: "
                f"position_gap_m={position_gap_m:.3f}, "
                f"heading_gap_deg={math.degrees(heading_gap_rad):.3f}"
            )
        offset_x_m = previous.x - current.x
        offset_y_m = previous.y - current.y
        offset_yaw_rad = _wrap_angle(previous.yaw - current.yaw)
        if preserve_next_path:
            tail_distances_m = [0.0]
            for index in range(len(joined) - 1, 0, -1):
                tail_distances_m.append(
                    tail_distances_m[-1]
                    + math.hypot(
                        joined[index].x - joined[index - 1].x,
                        joined[index].y - joined[index - 1].y,
                    )
                )
            blend_distance_m = min(
                connection_blend_distance_m,
                tail_distances_m[-1],
            )
            for reverse_index, distance_m in enumerate(tail_distances_m):
                if distance_m > blend_distance_m:
                    break
                index = len(joined) - 1 - reverse_index
                blend_weight = (
                    max(0.0, 1.0 - distance_m / blend_distance_m)
                    if blend_distance_m > 0.0
                    else 1.0
                )
                point = joined[index]
                seam_ref_id = previous.ref_id or current.ref_id
                joined[index] = (
                    PathPoint(current.x, current.y, current.yaw, seam_ref_id)
                    if reverse_index == 0
                    else PathPoint(
                        point.x - offset_x_m * blend_weight,
                        point.y - offset_y_m * blend_weight,
                        _wrap_angle(point.yaw - offset_yaw_rad * blend_weight),
                        point.ref_id,
                    )
                )
            joined.extend(path[1:])
            continue
        distances_m: list[float] = []
        distance_m = 0.0
        prior = current
        for point in path[1:]:
            distance_m += math.hypot(point.x - prior.x, point.y - prior.y)
            distances_m.append(distance_m)
            prior = point
        blend_distance_m = min(connection_blend_distance_m, distances_m[-1])
        if previous.ref_id is None and current.ref_id is not None:
            joined[-1] = PathPoint(
                previous.x,
                previous.y,
                previous.yaw,
                current.ref_id,
            )
        for point, distance_m in zip(path[1:], distances_m):
            blend_weight = (
                max(0.0, 1.0 - distance_m / blend_distance_m)
                if blend_distance_m > 0.0
                else 0.0
            )
            joined.append(
                PathPoint(
                    point.x + offset_x_m * blend_weight,
                    point.y + offset_y_m * blend_weight,
                    _wrap_angle(point.yaw + offset_yaw_rad * blend_weight),
                    point.ref_id,
                )
            )
    return tuple(joined)


def _nearest_reference_index(
    reference_path: Sequence[PathPoint],
    current_pose: PathPoint,
    *,
    previous_reference_index: int,
    search_window_points: int,
) -> int:
    start = max(0, min(previous_reference_index, len(reference_path) - 2) - 3)
    end = min(len(reference_path), start + search_window_points)
    return min(
        range(start, end),
        key=lambda index: (
            (reference_path[index].x - current_pose.x) ** 2
            + (reference_path[index].y - current_pose.y) ** 2
            + 0.04
            * _wrap_angle(reference_path[index].yaw - current_pose.yaw) ** 2,
            index,
        ),
    )


def _lookahead_index(
    reference_path: Sequence[PathPoint],
    start_index: int,
    lookahead_distance_m: float,
) -> int:
    distance = 0.0
    for index in range(start_index + 1, len(reference_path)):
        previous = reference_path[index - 1]
        current = reference_path[index]
        distance += math.hypot(current.x - previous.x, current.y - previous.y)
        if distance + 1e-9 >= lookahead_distance_m:
            return index
    return len(reference_path) - 1


def _relaxed_exit_is_ready(
    reference_path: Sequence[PathPoint],
    *,
    current_pose: PathPoint,
    start_index: int,
    segment_exit_index: int,
    planner: HybridAStarPlanner,
    position_tolerance_m: float,
    heading_tolerance_rad: float,
    clear_distance_m: float,
) -> bool:
    """Leave relaxed routing only after pose recovery and a clear route prefix."""

    segment_exit = reference_path[segment_exit_index]
    if start_index < segment_exit_index and math.hypot(
        segment_exit.x - current_pose.x,
        segment_exit.y - current_pose.y,
    ) > position_tolerance_m + 1e-9:
        return False
    nearest_reference = reference_path[start_index]
    if math.hypot(
        nearest_reference.x - current_pose.x,
        nearest_reference.y - current_pose.y,
    ) > position_tolerance_m + 1e-9:
        return False
    if (
        abs(_wrap_angle(nearest_reference.yaw - current_pose.yaw))
        > heading_tolerance_rad + 1e-9
    ):
        return False
    clear_end_index = _lookahead_index(
        reference_path,
        start_index,
        max(clear_distance_m, position_tolerance_m),
    )
    return planner.path_is_navigable(
        reference_path[start_index : clear_end_index + 1]
    )


def _relaxed_rejoin_after_blockage(
    reference_path: Sequence[PathPoint],
    planner: HybridAStarPlanner,
    *,
    start_index: int,
    nominal_rejoin_index: int,
    maximum_rejoin_index: int,
    stable_clear_distance_m: float,
) -> int:
    """Move a blocked segment goal to the first stable clear route prefix."""

    first_blocked_index = next(
        (
            index
            for index in range(start_index + 1, nominal_rejoin_index + 1)
            if not planner.path_is_navigable((reference_path[index],))
        ),
        None,
    )
    if first_blocked_index is None:
        return nominal_rejoin_index

    clear_distance_m = 0.0
    for index in range(first_blocked_index + 1, maximum_rejoin_index + 1):
        if not planner.path_is_navigable((reference_path[index],)):
            clear_distance_m = 0.0
            continue
        previous = reference_path[index - 1]
        current = reference_path[index]
        clear_distance_m += math.hypot(
            current.x - previous.x,
            current.y - previous.y,
        )
        if clear_distance_m + 1e-9 >= stable_clear_distance_m:
            return max(index, nominal_rejoin_index)
    return nominal_rejoin_index


def _active_relaxed_segment(
    reference_path: Sequence[PathPoint],
    *,
    start_index: int,
    entry_ref: str,
    exit_ref: str,
    activation_distance_m: float,
    planning_horizon_m: float,
) -> tuple[int, int] | None:
    """Activate once the local planning horizon reaches the approach window."""

    if not entry_ref or not exit_ref:
        return None

    exit_search_start = 0
    for entry_index, point in enumerate(reference_path):
        if point.ref_id != entry_ref:
            continue
        exit_index = next(
            (
                index
                for index in range(max(entry_index + 1, exit_search_start), len(reference_path))
                if reference_path[index].ref_id == exit_ref
            ),
            None,
        )
        if exit_index is None:
            return None
        exit_search_start = exit_index + 1
        if entry_index <= start_index < exit_index:
            return entry_index, exit_index
        if start_index < entry_index:
            distance = sum(
                math.hypot(
                    reference_path[index].x - reference_path[index - 1].x,
                    reference_path[index].y - reference_path[index - 1].y,
                )
                for index in range(start_index + 1, entry_index + 1)
            )
            if distance <= activation_distance_m + 1e-9:
                return entry_index, exit_index
            if distance <= activation_distance_m + planning_horizon_m + 1e-9:
                return entry_index, exit_index
    return None


def _active_reference_priority_segment(
    reference_path: Sequence[PathPoint],
    *,
    start_index: int,
    entry_ref: str,
    exit_ref: str,
    activation_distance_m: float,
) -> tuple[int, int] | None:
    """Activate a verified-reference span shortly before its entry marker."""

    if not entry_ref or not exit_ref:
        return None

    exit_search_start = 0
    for entry_index, point in enumerate(reference_path):
        if point.ref_id != entry_ref:
            continue
        exit_index = next(
            (
                index
                for index in range(
                    max(entry_index + 1, exit_search_start),
                    len(reference_path),
                )
                if reference_path[index].ref_id == exit_ref
            ),
            None,
        )
        if exit_index is None:
            return None
        exit_search_start = exit_index + 1
        if entry_index <= start_index < exit_index:
            return entry_index, exit_index
        if start_index >= entry_index:
            continue
        distance_to_entry_m = sum(
            math.hypot(
                reference_path[index].x - reference_path[index - 1].x,
                reference_path[index].y - reference_path[index - 1].y,
            )
            for index in range(start_index + 1, entry_index + 1)
        )
        if distance_to_entry_m <= activation_distance_m + 1e-9:
            return entry_index, exit_index
    return None


def _path_is_materially_better(
    candidate: Sequence[PathPoint],
    held: Sequence[PathPoint],
    *,
    reference_path: Sequence[PathPoint],
    reference_deviation_weight: float,
    improvement_ratio: float,
) -> bool:
    candidate_cost = _path_cost(
        candidate,
        reference_path=reference_path,
        reference_deviation_weight=reference_deviation_weight,
    )
    held_cost = _path_cost(
        held,
        reference_path=reference_path,
        reference_deviation_weight=reference_deviation_weight,
    )
    return candidate_cost < held_cost * (1.0 - improvement_ratio)


def _splice_respects_turning_radius(
    path: Sequence[PathPoint],
    *,
    min_turning_radius_m: float,
) -> bool:
    """Validate the only new bend introduced by prepending the current pose."""

    if len(path) < 3:
        return True
    first, second, third = path[:3]
    ab = math.hypot(second.x - first.x, second.y - first.y)
    bc = math.hypot(third.x - second.x, third.y - second.y)
    ca = math.hypot(first.x - third.x, first.y - third.y)
    denominator = ab * bc * ca
    if denominator <= 1e-12:
        return True
    cross = (second.x - first.x) * (third.y - first.y) - (
        second.y - first.y
    ) * (third.x - first.x)
    curvature = 2.0 * cross / denominator
    return abs(curvature) <= 1.0 / min_turning_radius_m + 1e-6


def _path_cost(
    path: Sequence[PathPoint],
    *,
    reference_path: Sequence[PathPoint],
    reference_deviation_weight: float,
) -> float:
    cost = 0.0
    reference_xy = tuple((point.x, point.y) for point in reference_path)
    for previous, point in zip(path, path[1:]):
        distance = math.hypot(point.x - previous.x, point.y - previous.y)
        deviation_squared = min(
            (point.x - x) ** 2 + (point.y - y) ** 2
            for x, y in reference_xy
        )
        cost += distance * (
            1.0 + reference_deviation_weight * deviation_squared
        )
    return cost


def _select_navigable_rejoin_index(
    reference_path: Sequence[PathPoint],
    planner: HybridAStarPlanner,
    *,
    start_index: int,
    preferred_rejoin_index: int,
) -> int:
    """Pick a reachable forward reference point near the requested rejoin point.

    Live point-cloud obstacles can temporarily occupy the exact lookahead point.
    Failing the whole local plan in that case causes the vehicle to stop even
    though a nearby point on the same global route is still usable.
    """

    if planner.path_is_navigable((reference_path[preferred_rejoin_index],)):
        return preferred_rejoin_index

    forward_candidates = range(preferred_rejoin_index + 1, len(reference_path))
    backward_candidates = range(preferred_rejoin_index - 1, start_index, -1)
    for candidates in (forward_candidates, backward_candidates):
        for index in candidates:
            if planner.path_is_navigable((reference_path[index],)):
                return index
    return preferred_rejoin_index


def _overlay_obstacles(
    static_map: OccupancyGridMap,
    points: Iterable[tuple[float, float]],
) -> tuple[OccupancyGridMap, int]:
    occupied = list(static_map.occupied)
    count = 0
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        cell = static_map.world_to_cell(x, y)
        if not static_map.contains(cell):
            continue
        occupied[static_map.index(cell)] = True
        count += 1
    return (
        OccupancyGridMap(
            width=static_map.width,
            height=static_map.height,
            resolution=static_map.resolution,
            origin_x=static_map.origin_x,
            origin_y=static_map.origin_y,
            occupied=tuple(occupied),
        ),
        count,
    )


def _crop_planning_map(
    static_map: OccupancyGridMap,
    current_pose: PathPoint,
    reference_path: Sequence[PathPoint],
    *,
    margin_m: float,
) -> OccupancyGridMap:
    x_values = [current_pose.x, *(point.x for point in reference_path)]
    y_values = [current_pose.y, *(point.y for point in reference_path)]
    resolution = static_map.resolution
    min_col = max(
        0,
        math.floor((min(x_values) - margin_m - static_map.origin_x) / resolution),
    )
    max_col = min(
        static_map.width - 1,
        math.floor((max(x_values) + margin_m - static_map.origin_x) / resolution),
    )
    min_bottom = max(
        0,
        math.floor((min(y_values) - margin_m - static_map.origin_y) / resolution),
    )
    max_bottom = min(
        static_map.height - 1,
        math.floor((max(y_values) + margin_m - static_map.origin_y) / resolution),
    )
    if min_col > max_col or min_bottom > max_bottom:
        raise ValueError("local planning window is outside the static occupancy map")

    width = max_col - min_col + 1
    height = max_bottom - min_bottom + 1
    occupied: list[bool] = []
    first_source_row = static_map.height - 1 - max_bottom
    for row in range(first_source_row, first_source_row + height):
        start = row * static_map.width + min_col
        occupied.extend(static_map.occupied[start : start + width])
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=static_map.origin_x + min_col * resolution,
        origin_y=static_map.origin_y + min_bottom * resolution,
        occupied=tuple(occupied),
    )


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def occupied_grid_cell_centers(
    data: Iterable[int],
    *,
    width: int,
    height: int,
    resolution_m: float,
    origin_x_m: float,
    origin_y_m: float,
    occupancy_threshold: int,
    x_min_m: float,
    x_max_m: float,
    y_half_width_m: float,
    max_points: int,
) -> tuple[tuple[float, float], ...]:
    """Extract bounded obstacle cells from an already-inflated local costmap."""

    if width < 1 or height < 1 or resolution_m <= 0.0:
        raise ValueError("costmap geometry is invalid")
    if not 0 <= occupancy_threshold <= 100:
        raise ValueError("occupancy_threshold must be in [0, 100]")
    if x_max_m <= x_min_m or y_half_width_m <= 0.0 or max_points < 1:
        raise ValueError("costmap obstacle bounds are invalid")

    occupied: list[tuple[float, float]] = []
    for index, value in enumerate(data):
        if index >= width * height:
            break
        if int(value) < occupancy_threshold:
            continue
        row, column = divmod(index, width)
        x = origin_x_m + (column + 0.5) * resolution_m
        y = origin_y_m + (row + 0.5) * resolution_m
        if not (x_min_m <= x <= x_max_m and abs(y) <= y_half_width_m):
            continue
        occupied.append((x, y))

    if len(occupied) <= max_points:
        return tuple(occupied)
    stride = math.ceil(len(occupied) / max_points)
    return tuple(occupied[::stride][:max_points])
