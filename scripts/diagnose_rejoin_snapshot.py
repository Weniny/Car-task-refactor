#!/usr/bin/env python3
"""Read a stationary ROS snapshot and trace offline rejoin searches; never drive."""

import argparse
import json
import math
import time
from dataclasses import asdict, fields, replace

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from competition_planning.local_replanner_node import _load_reference_paths
from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
    concatenate_reference_paths,
    occupied_grid_cell_centers,
)
from competition_planning.occupancy_grid_planner import GridAStarPlanner, GridPlanningError, OccupancyGridMap
from competition_planning.semantic_planner import PathPoint
from competition_safety.path_aware_proximity import PathAwareConfig, evaluate_path_aware_stop
from competition_safety.proximity_stop import ProximityStopConfig


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class Snapshot(Node):
    def __init__(self):
        super().__init__("rejoin_snapshot_diagnostic")
        self.grid = None
        self.status = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(
            OccupancyGrid, "/avoidance/local_costmap", self.on_grid, qos_profile_sensor_data
        )
        self.create_subscription(String, "/control/status", self.on_status, 1)

    def on_grid(self, msg):
        self.grid = msg

    def on_status(self, msg):
        self.status = json.loads(msg.data)


def wait(node, future):
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or future.result() is None:
        raise RuntimeError("parameter request timed out")
    return future.result()


def run(node, fine_lattice=False, planning_budget_s=None, steering_step_bins=1, grid_guide=False, obstacle_heuristic=False, lookahead_distance_m=None, compare=False, obstacle_x_max_m=None):
    if node.count_publishers("/cmd_vel"):
        raise RuntimeError("stop chassis adapter before diagnostic")
    list_client = node.create_client(ListParameters, "/rebuild_local_replanner/list_parameters")
    get_client = node.create_client(GetParameters, "/rebuild_local_replanner/get_parameters")
    if not list_client.wait_for_service(timeout_sec=5) or not get_client.wait_for_service(timeout_sec=5):
        raise RuntimeError("local replanner parameter service unavailable")
    names = wait(node, list_client.call_async(ListParameters.Request())).result.names
    values = wait(node, get_client.call_async(GetParameters.Request(names=names))).values
    params = {}
    for name, value in zip(names, values):
        if value.type == 1:
            params[name] = value.bool_value
        elif value.type == 2:
            params[name] = value.integer_value
        elif value.type == 3:
            params[name] = value.double_value
        elif value.type == 4:
            params[name] = value.string_value
    deadline = time.monotonic() + 6
    transform = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.grid is None or node.status is None:
            continue
        try:
            transform = node.tf.lookup_transform(
                params["map_frame"], node.grid.header.frame_id,
                rclpy.time.Time.from_msg(node.grid.header.stamp),
            ).transform
            break
        except Exception:
            continue
    if transform is None:
        raise RuntimeError("fresh costmap, control status and scan-time TF required")
    if node.status.get("route_enabled") is not False or node.count_publishers("/cmd_vel"):
        raise RuntimeError("route must be disabled and chassis adapter absent")
    grid = node.grid
    age = (node.get_clock().now() - rclpy.time.Time.from_msg(grid.header.stamp)).nanoseconds / 1e9
    if not 0 <= age <= params["max_obstacle_age_s"]:
        raise RuntimeError(f"snapshot costmap stale: {age:.3f}s")
    if grid.header.frame_id != params["base_frame"]:
        raise RuntimeError("diagnostic currently requires body-frame costmap")
    if abs(yaw(grid.info.origin.orientation)) > 1e-6:
        raise RuntimeError("rotated grid origin unsupported")
    points_body = occupied_grid_cell_centers(
        grid.data, width=grid.info.width, height=grid.info.height,
        resolution_m=grid.info.resolution,
        origin_x_m=grid.info.origin.position.x, origin_y_m=grid.info.origin.position.y,
        occupancy_threshold=params["costmap_occupancy_threshold"],
        x_min_m=params["obstacle_x_min_m"],
        x_max_m=obstacle_x_max_m or params["obstacle_x_max_m"],
        y_half_width_m=params["obstacle_y_half_width_m"],
        max_points=params["max_obstacle_points"],
    )
    angle = yaw(transform.rotation)
    c, s = math.cos(angle), math.sin(angle)
    tx, ty = transform.translation.x, transform.translation.y
    points = tuple((tx + c*x - s*y, ty + s*x + c*y) for x, y in points_body)
    config_values = {f.name: params[f.name] for f in fields(LocalReplanConfig) if f.name in params}
    for field in ("goal_heading_tolerance", "relaxed_goal_heading_tolerance"):
        if field + "_deg" in params:
            config_values[field + "_rad"] = math.radians(params[field + "_deg"])
    config = LocalReplanConfig(**config_values)
    if lookahead_distance_m is not None:
        config = replace(config, lookahead_distance_m=lookahead_distance_m)
    if obstacle_heuristic:
        config = replace(config, obstacle_aware_heuristic=True)
    if fine_lattice:
        config = replace(config, step_length_m=0.10, curvature_bins=17, heading_bins=144)
    if planning_budget_s is not None:
        config = replace(config, planning_timeout_s=planning_budget_s,
                         empty_history_planning_timeout_s=planning_budget_s,
                         relaxed_extension_timeout_s=min(config.relaxed_extension_timeout_s, planning_budget_s))
    reference = concatenate_reference_paths(_load_reference_paths(params["trajectory_file"], params["map_frame"]))
    static_map = OccupancyGridMap.from_yaml(params["map_file"])
    start_cell = static_map.world_to_cell(tx, ty)
    nearest_static = None
    radius_cells = math.ceil(2.0 / static_map.resolution)
    for row in range(start_cell[1]-radius_cells, start_cell[1]+radius_cells+1):
        for col in range(start_cell[0]-radius_cells, start_cell[0]+radius_cells+1):
            cell = (col, row)
            if static_map.contains(cell) and static_map.occupied[static_map.index(cell)]:
                x, y = static_map.cell_to_world(cell)
                distance = math.hypot(x-tx, y-ty)
                if nearest_static is None or distance < nearest_static["distance_m"]:
                    nearest_static = {"point": [x, y], "distance_m": distance}
    attempts = []

    class TracePlanner(LocalTrajectoryPlanner):
        def _planner(self, *args, **kwargs):
            planner = super()._planner(*args, **kwargs)
            if steering_step_bins != 1:
                planner._successor_curvatures = lambda index: range(
                    max(0, index-steering_step_bins),
                    min(len(planner._curvatures), index+steering_step_bins+1),
                )
            original = planner.plan

            def traced(poses):
                start, goal = poses[0], poses[-1]
                record = {
                    "goal": [goal.x, goal.y],
                    "goal_distance_m": math.hypot(goal.x-start.x, goal.y-start.y),
                    "start_clear": planner.path_is_navigable((start,)),
                    "goal_clear": planner.path_is_navigable((goal,)),
                    "nearest_dynamic_to_goal_m": min(
                        (math.hypot(x-goal.x, y-goal.y) for x, y in points), default=None
                    ),
                    "timeout_override_s": kwargs.get("planning_timeout_s"),
                }
                attempts.append(record)
                started = time.perf_counter()
                try:
                    if grid_guide:
                        try:
                            guide = GridAStarPlanner(
                                args[0], inflation_radius_m=config.inflation_radius_m,
                                search_padding_m=config.search_padding_m,
                                sample_spacing_m=config.sample_spacing_m,
                            ).plan(poses)
                            record["grid_guide_points"] = len(guide)
                            planner._default_reference_xy = tuple((p.x, p.y) for p in guide)
                            planner._reference_xy = planner._default_reference_xy
                            planner._reference_distance_sq_cache.clear()
                        except GridPlanningError as exc:
                            record["grid_guide_error"] = str(exc)
                    path = original(poses)
                    record["outcome"] = "SUCCESS"
                    record["point_count"] = len(path)
                    return path
                except Exception as exc:
                    record["outcome"] = type(exc).__name__
                    record["error"] = str(exc)
                    raise
                finally:
                    record["elapsed_ms"] = (time.perf_counter()-started)*1000

            planner.plan = traced
            return planner

    variants = [("requested", config)]
    if compare:
        guarded = replace(config, minimum_short_rejoin_distance_m=1.8)
        variants = [
            ("baseline", config),
            ("guarded", guarded),
            ("guarded_obstacle_heuristic", replace(guarded, obstacle_aware_heuristic=True)),
            ("guarded_obstacle_heuristic_2s", replace(
                guarded, obstacle_aware_heuristic=True,
                planning_timeout_s=2.0, empty_history_planning_timeout_s=2.0,
            )),
            ("guarded_heading50_offline_only", replace(
                guarded, obstacle_aware_heuristic=True,
                goal_heading_tolerance_rad=math.radians(50.0),
                planning_timeout_s=2.0, empty_history_planning_timeout_s=2.0,
            )),
            ("guarded_lookahead4_offline_only", replace(
                guarded, obstacle_aware_heuristic=True, lookahead_distance_m=4.0,
                planning_timeout_s=2.0, empty_history_planning_timeout_s=2.0,
            )),
            ("guarded_lookahead4_budget04_offline_only", replace(
                guarded, obstacle_aware_heuristic=True, lookahead_distance_m=4.0,
            )),
            ("guarded_lookahead4_budget06_offline_only", replace(
                guarded, obstacle_aware_heuristic=True, lookahead_distance_m=4.0,
                planning_timeout_s=0.6, empty_history_planning_timeout_s=0.6,
            )),
        ]
    results = []
    for label, config in variants:
        attempts = []
        planner = TracePlanner(static_map, config)
        result = {"variant": label, "snapshot_only": True, "fine_lattice": fine_lattice, "grid_guide": grid_guide,
              "inflation_radius_m": config.inflation_radius_m,
              "planning_budget_s": config.planning_timeout_s,
              "lookahead_distance_m": config.lookahead_distance_m,
              "obstacle_x_max_m": obstacle_x_max_m or params["obstacle_x_max_m"],
              "obstacle_aware_heuristic": config.obstacle_aware_heuristic,
              "goal_heading_tolerance_deg": math.degrees(config.goal_heading_tolerance_rad),
              "steering_step_bins": steering_step_bins,
              "costmap_age_s": age, "obstacle_count": len(points), "attempts": attempts,
              "nearest_static_to_start": nearest_static,
              "nearest_dynamic_to_start_m": min(
                  (math.hypot(x-tx, y-ty) for x, y in points), default=None
              )}
        try:
            path = planner.plan(reference_path=reference, current_pose=PathPoint(tx, ty, angle),
                                dynamic_obstacle_points=points,
                                previous_reference_index=node.status.get("route_progress_index", 0))
            result["status"] = path.status
            result["path_length_m"] = sum(
                math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(path.path, path.path[1:])
            )
            path_body = tuple(
                (c*(p.x-tx) + s*(p.y-ty), -s*(p.x-tx) + c*(p.y-ty))
                for p in path.path
            )
            result["costmap_only_safety_shadow_at_zero_speed"] = asdict(
                evaluate_path_aware_stop(
                    ((x, y, 0.0) for x, y in points_body), path_body,
                    ProximityStopConfig(stop_distance_m=1.8), PathAwareConfig(),
                    measured_speed_mps=0.0,
                )
            )
        except Exception as exc:
            result["status"] = type(exc).__name__
            result["error"] = str(exc)
        results.append(result)
    print(json.dumps(results if compare else results[0], indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fine-lattice", action="store_true", help="Offline only: 0.1 m, 17 curvature bins, 144 headings")
    parser.add_argument("--planning-budget-s", type=float, help="Offline only: search timeout (0 < value <= 3)")
    parser.add_argument("--steering-step-bins", type=int, default=1, choices=(1, 2), help="Offline only: successor curvature-index change; not a vehicle validation")
    parser.add_argument("--grid-guide", action="store_true", help="Offline only: guide reference cost using existing grid A*")
    parser.add_argument("--obstacle-heuristic", action="store_true", help="Offline only: enable candidate obstacle-aware distance field")
    parser.add_argument("--lookahead-distance-m", type=float, choices=(3.0, 4.0), help="Offline only: test farther rejoin; obstacles remain limited to captured bounds")
    parser.add_argument("--compare", action="store_true", help="Compare configurations against the same read-only snapshot")
    parser.add_argument("--obstacle-x-max-m", type=float, choices=(4.0, 5.5), help="Offline only: include farther obstacles from the captured costmap")
    args = parser.parse_args()
    if args.planning_budget_s is not None and not 0 < args.planning_budget_s <= 3:
        parser.error("planning budget must be positive and at most 3 seconds")
    rclpy.init()
    node = Snapshot()
    try:
        run(node, args.fine_lattice, args.planning_budget_s, args.steering_step_bins, args.grid_guide, args.obstacle_heuristic, args.lookahead_distance_m, args.compare, args.obstacle_x_max_m)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
