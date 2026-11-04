#!/usr/bin/env python3
"""Frozen-map planner/MPPI/Safety model regression; never imports ROS or drives hardware."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import time

from competition_control.mppi_controller import (
    MPPIController, MPPIParams, ControlTrajectory, ControlTrajectoryPoint,
    VehicleState, BodyCommand, apply_mission_speed_limit,
)
from competition_control.local_plan_continuity import local_paths_are_equivalent
from competition_planning.local_trajectory_planner import (
    LocalReplanConfig, LocalTrajectoryPlanner, _overlay_obstacles,
)
from competition_planning.occupancy_grid_planner import OccupancyGridMap, GridPlanningError
from competition_planning.semantic_planner import PathPoint
from competition_planning.trajectory_parameterizer import parameterize_local_path
from competition_safety.supervisor import SafetyLimits, SafetyContext, SafetySupervisor


def run(fixture, clearance_weight=0., seed=7, max_seconds=90., motion_yaw_bias_deg=0.,
        obstacle_heuristic=False, plan_delay_s=0., smooth_reconnect=False):
    grid_data = dict(fixture['grid'])
    grid_data['occupied'] = tuple(grid_data['occupied'])
    grid = OccupancyGridMap(**grid_data)
    reference = tuple(PathPoint(**p) for p in fixture['reference'])
    local_goal = reference[-1]
    reference += tuple(PathPoint(**p) for p in fixture.get('continuation', ()))
    obstacles = tuple(tuple(p) for p in fixture['obstacles'])
    cfg = LocalReplanConfig(
        lookahead_distance_m=4., inflation_radius_m=.04,
        dynamic_obstacle_cell_size_m=.05, retain_safe_detour=True,
        minimum_short_rejoin_distance_m=2., min_turning_radius_m=.6,
        planning_timeout_s=.4, empty_history_planning_timeout_s=.4,
        relaxed_extension_timeout_s=.35, search_padding_m=1.5,
        goal_position_tolerance_m=.15, goal_heading_tolerance_rad=math.radians(20),
        reference_search_window_points=160,
        search_heuristic_weight=1.6,
        obstacle_clearance_distance_m=.30 if clearance_weight else 0.,
        obstacle_clearance_weight=clearance_weight,
        obstacle_aware_heuristic=obstacle_heuristic,
        smooth_detour_reconnect=smooth_reconnect,
    )
    planner = LocalTrajectoryPlanner(grid, cfg)
    live, _ = _overlay_obstacles(grid, obstacles, cell_size_m=.05)
    checker = planner._planner(live, reference)
    params = MPPIParams(
        rollout_count=384, max_speed_mps=.5, command_speed_memory_limit_mps=.5,
        max_acceleration_mps2=.12, max_deceleration_mps2=.18,
        max_jerk_mps3=.2, max_curvature_rate_1pmps=2.5, min_turning_radius_m=.6,
        speed_noise_std_mps=.01, curvature_noise_std_1pm=.08,
        goal_position_tolerance_m=.03,goal_heading_tolerance_rad=math.radians(2),
    )
    safety = SafetySupervisor(SafetyLimits(
        max_speed_mps=.5, max_acceleration_mps2=.12, max_deceleration_mps2=.18,
        min_turning_radius_m=.6, recovery_lateral_error_m=.2,
        recovery_clear_lateral_error_m=.1, recovery_heading_error_rad=math.radians(20),
        recovery_clear_heading_error_rad=math.radians(8), recovery_speed_mps=.15,
    ))
    p = fixture['pose']
    state = VehicleState(p['x'], p['y'], p['yaw'], .2)
    controller = None
    accepted = None
    previous = 0
    stop_ticks = 0
    consecutive_stop_ticks = 0
    motion_started = False
    in_route_stop_ticks = 0
    failures = 0
    replacements = 0
    samples = []
    outcome = 'TIME_LIMIT'
    pending = None
    planning_times = []
    last_plan_status = 'NONE'
    failure_examples = []
    started = time.perf_counter()
    for tick in range(int(max_seconds/.05)):
        now = 1. + tick*.05
        if tick % 10 == 0:
            planning_started = time.perf_counter()
            try:
                result = planner.plan(reference_path=reference,
                    current_pose=PathPoint(state.x,state.y,state.yaw),
                    dynamic_obstacle_points=obstacles, previous_reference_index=previous)
                if not planner.path_is_collision_free(result.path, obstacles):
                    raise GridPlanningError('output collision')
                previous = result.reference_start_index
                last_plan_status = result.status
                pending = (tick + round(plan_delay_s/.05), result.path)
            except (GridPlanningError,ValueError) as exc:
                if len(failure_examples)<3:
                    failure_examples.append(str(exc))
                last_plan_status = 'NO_PATH'
                failures += 1
                controller = None
                accepted = None
                pending = None
            planning_times.append((time.perf_counter()-planning_started)*1000)
        if pending is not None and tick >= pending[0]:
            geometry = pending[1]
            pending = None
            if accepted is None or not local_paths_are_equivalent(accepted,geometry):
                try:
                    parameterized = parameterize_local_path(geometry, {}, {
                        'trajectory_optimizer':dict(max_speed_mps=.5,
                            max_acceleration_mps2=.12,max_deceleration_mps2=.18,
                            max_lateral_acceleration_mps2=.12,
                            high_curvature_threshold_1pm=.5,high_curvature_speed_limit_mps=.3)})
                    trajectory = ControlTrajectory('map','offline',tuple(
                        ControlTrajectoryPoint(v.x,v.y,v.yaw,v.s,v.curvature,v.v,v.t)
                        for v in parameterized.points))
                    if controller is None:
                        controller = MPPIController(trajectory,params,random_seed=seed)
                    else:
                        controller.replace_trajectory(trajectory)
                    accepted = geometry
                    replacements += 1
                except ValueError:
                    failures += 1
                    controller = None
                    accepted = None
        command = (controller.compute_command(state) if controller else
                   BodyCommand.hold(target_index=0,lateral_error_m=0.,heading_error_rad=0.,status='LOCAL_PLANNER_STOP'))
        command = apply_mission_speed_limit(command,.2)
        safe = safety.filter_command(command,SafetyContext(
            now,now,now,state.linear_speed_mps,True,True,True,True,False,False))
        speed,yaw_rate = safe.linear_x_mps,safe.yaw_rate_radps
        if speed<.02:
            stop_ticks += 1
            consecutive_stop_ticks += 1
            if motion_started:
                in_route_stop_ticks += 1
        else:
            consecutive_stop_ticks = 0
            if speed >= .03:
                motion_started = True
        if consecutive_stop_ticks >= 200:
            outcome='STALLED'
            break
        direction=state.yaw+yaw_rate*.025+math.radians(motion_yaw_bias_deg)
        x=state.x+speed*math.cos(direction)*.05
        y=state.y+speed*math.sin(direction)*.05
        yaw=state.yaw+yaw_rate*.05
        if not grid.contains(grid.world_to_cell(x,y)):
            outcome='OUTSIDE_FIXTURE'
            break
        if not checker.path_is_navigable((PathPoint(x,y,yaw),)):
            outcome='PREDICTED_COLLISION'
            break
        state=VehicleState(x,y,yaw,speed)
        samples.append({'t':tick*.05,'x':x,'y':y,'yaw':yaw,'v':speed,
                        'planner_index':previous,'safety':safe.status,
                        'control':command.status,'plan':last_plan_status,
                        'endpoint':([accepted[-1].x,accepted[-1].y] if accepted else None)})
        dx,dy=x-local_goal.x,y-local_goal.y
        along=dx*math.cos(local_goal.yaw)+dy*math.sin(local_goal.yaw)
        lateral=-dx*math.sin(local_goal.yaw)+dy*math.cos(local_goal.yaw)
        # This fixture is a passage, not a parking goal. Require crossing its
        # exit plane within 0.35m of the unchanged route; check obstacles throughout.
        if along >= 0 and abs(lateral) <= .35:
            outcome='PASSED_SECTION'
            break
    return dict(outcome=outcome,simulated_s=tick*.05,stop_s=stop_ticks*.05,
                in_route_stop_s=in_route_stop_ticks*.05,
                planning_failures=failures,replacements=replacements,
                failure_examples=failure_examples,
                final_distance_m=math.hypot(state.x-local_goal.x,state.y-local_goal.y),
                elapsed_s=time.perf_counter()-started,
                planning_max_ms=max(planning_times,default=0),
                planning_p95_ms=sorted(planning_times)[int(.95*(len(planning_times)-1))] if planning_times else 0,
                samples=samples)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fixture',type=Path)
    parser.add_argument('--clearance-weight',type=float,default=0.)
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--motion-yaw-bias-deg',type=float,default=0.)
    parser.add_argument('--obstacle-heuristic',action='store_true')
    parser.add_argument('--plan-delay-s',type=float,default=0.)
    parser.add_argument('--smooth-reconnect',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=run(json.loads(args.fixture.read_text()),args.clearance_weight,args.seed,
               motion_yaw_bias_deg=args.motion_yaw_bias_deg,
               obstacle_heuristic=args.obstacle_heuristic,plan_delay_s=args.plan_delay_s,
               smooth_reconnect=args.smooth_reconnect)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'}))
