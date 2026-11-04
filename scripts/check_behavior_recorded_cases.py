#!/usr/bin/env python3
"""Recheck recorded startup/far-blockage/near-blockage geometry offline."""
import argparse
import json
import math
from pathlib import Path
import sys

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--runtime',required=True,type=Path)
ap.add_argument('--snapshots',required=True,type=Path)
ap.add_argument('--output',required=True,type=Path)
args=ap.parse_args()
sys.path.insert(0,str(args.runtime/'src/competition_planning'))
import yaml
from competition_planning.local_trajectory_planner import LocalReplanConfig,LocalTrajectoryPlanner
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.forward_rejoin import ObservationWindow
from competition_planning.path_guard import PathGuard
from competition_planning.semantic_planner import PathPoint

static=OccupancyGridMap.from_yaml(args.runtime/'maps/indoor_manual_final/map.yaml')
raw=yaml.safe_load((args.runtime/'artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml').read_text())['points']
reference=tuple(PathPoint(r['x'],r['y'],r['yaw']) for r in raw)
snapshots=json.loads(args.snapshots.read_text())
def sample(t):return min(snapshots,key=lambda r:abs(r['t']-t))
cfg=LocalReplanConfig(lookahead_distance_m=3.,inflation_radius_m=.04,
    search_padding_m=1.5,min_turning_radius_m=.6,planning_timeout_s=.4,
    empty_history_planning_timeout_s=.4,relaxed_extension_timeout_s=.35,
    forward_rejoin_enabled=True,tracking_clearance_m=.1,
    goal_position_tolerance_m=.15,goal_heading_tolerance_rad=math.radians(20),search_heuristic_weight=1.6)
def seed(s):
    p=LocalTrajectoryPlanner(static,cfg)
    p._last_safe_path=tuple(PathPoint(*r) for r in s['path'])
    p._last_safe_rejoin_index=s['rejoin_index']
    p._forward_rejoin_target_index=s['rejoin_index']
    return p
def window(s):return ObservationWindow(*s['pose'],-.5,-2.5,5.5,2.5)

startup=sample(2.2);p=seed(startup)
kw=dict(reference_path=reference,current_pose=PathPoint(*startup['pose']),
    dynamic_obstacle_points=startup['inflated_world'],observation_window=window(startup),prestart_stationary=True)
before=p.plan(**kw);after=p.plan(**kw)
assert before.status=='FORWARD_REJOIN_HOLD' and after.status=='REFERENCE_CLEAR'
assert max(abs(r.y-reference[after.reference_start_index+j].y) for j,r in enumerate(after.path))<1e-9

far=sample(79.05);p=seed(far)
out=p.continue_after_failure(reference_path=reference,current_pose=PathPoint(*far['pose']),
    dynamic_obstacle_points=far['inflated_world'],previous_reference_index=far['reference_start_index'],
    observation_window=window(far),speed_mps=.5,latency_s=1.7,deceleration_mps2=.18,max_speed_mps=.5,stop_margin_m=.1)
assert out is not None,'recorded far case has no stopping-safe prefix'
original_goal_retained=p._forward_rejoin_target_index
assert original_goal_retained==far['rejoin_index']
guard=PathGuard(1_000_000_000,2_000_000_000,out.speed_limit_mps,True,1.7,.18,.1,.1,math.radians(20))
limit=guard.speed_limit(out.path,PathPoint(*far['pose']),now_ns=1_000_000_000,path_stamp_ns=guard.path_stamp_ns,speed_mps=.5)
assert limit is not None and limit>0

near=sample(113.4);p=seed(sample(112.9))
blocked=p.continue_after_failure(reference_path=reference,current_pose=PathPoint(*near['pose']),
    dynamic_obstacle_points=near['inflated_world'],previous_reference_index=781,
    observation_window=window(near),speed_mps=.5,latency_s=1.7,deceleration_mps2=.18,max_speed_mps=.5,stop_margin_m=.1)
assert blocked is None,'near recorded blockage must retain stop'
result={'status':'PASS','startup':{'old_status':before.status,'new_status':after.status,'old_path_max_deviation_m':startup['path_reference_max_deviation_m'],'new_matches_reference':True},
    'far_failure':{'status':out.status,'clear_distance_m':out.continuation_clear_distance_m,'guard_speed_limit_mps':limit,'original_goal_retained':original_goal_retained},
    'near_failure':{'stop_retained':True},
    'limitations':['Frozen adjacent recorded snapshots, not exact callback replay or a field motion test.','Uses the existing inflated point collision model; no new rectangular footprint proof.']}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
