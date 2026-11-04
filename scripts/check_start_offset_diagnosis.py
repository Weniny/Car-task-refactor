#!/usr/bin/env python3
"""Reproduce prestart detour retention, changing longitudinal offset only."""
import argparse
import json
import math
from pathlib import Path
import sys

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--runtime',type=Path,required=True)
ap.add_argument('--snapshots',type=Path,required=True)
ap.add_argument('--output',type=Path,required=True)
args=ap.parse_args()
sys.path.insert(0,str(args.runtime/'src/competition_planning'))
import yaml
from competition_planning.local_trajectory_planner import LocalReplanConfig,LocalTrajectoryPlanner
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.forward_rejoin import ObservationWindow
from competition_planning.semantic_planner import PathPoint

rows=json.loads(args.snapshots.read_text())
scene=min(rows,key=lambda r:abs(r['t']-37.2))
raw=yaml.safe_load((args.runtime/'artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml').read_text())['points']
ref=tuple(PathPoint(r['x'],r['y'],r['yaw']) for r in raw)
pose=PathPoint(*scene['pose']);origin=ref[0]
c,s=math.cos(origin.yaw),math.sin(origin.yaw)
dx,dy=pose.x-origin.x,pose.y-origin.y
along=c*dx+s*dy;cross=-s*dx+c*dy
corrected=PathPoint(pose.x-along*c,pose.y-along*s,pose.yaw)
cfg=LocalReplanConfig(lookahead_distance_m=3.,inflation_radius_m=.04,search_padding_m=1.5,
    min_turning_radius_m=.6,planning_timeout_s=.4,empty_history_planning_timeout_s=.4,
    relaxed_extension_timeout_s=.35,forward_rejoin_enabled=True,tracking_clearance_m=.1,
    goal_position_tolerance_m=.15,goal_heading_tolerance_rad=math.radians(20),search_heuristic_weight=1.6)
static=OccupancyGridMap.from_yaml(args.runtime/'maps/indoor_manual_final/map.yaml')
def run(at_pose):
    planner=LocalTrajectoryPlanner(static,cfg)
    planner._last_safe_path=tuple(PathPoint(*r) for r in scene['path'])
    planner._last_safe_rejoin_index=scene['rejoin_index']
    planner._forward_rejoin_target_index=scene['rejoin_index']
    states=[]
    for _ in range(2):
        result=planner.plan(reference_path=ref,current_pose=at_pose,
            dynamic_obstacle_points=scene['inflated_world'],previous_reference_index=0,
            observation_window=ObservationWindow(*scene['pose'],-.5,-2.5,5.5,2.5),prestart_stationary=True)
        states.append({'status':result.status,'clear_count':planner._prestart_clear_count,'rejoin_index':result.rejoin_index})
    return states
actual=run(pose);counterfactual=run(corrected)
assert all(r['status']=='FORWARD_REJOIN_HOLD' and r['clear_count']==0 for r in actual)
assert counterfactual[-1]['status']=='REFERENCE_CLEAR'
result={'status':'REPRODUCED','sample_bag_s':scene['grid_t'],
    'reference_start':[origin.x,origin.y,origin.yaw],'localized_pose':[pose.x,pose.y,pose.yaw],
    'distance_to_path_start_m':math.hypot(dx,dy),'longitudinal_offset_m':along,'lateral_offset_m':cross,
    'heading_error_deg':math.degrees(math.atan2(math.sin(pose.yaw-origin.yaw),math.cos(pose.yaw-origin.yaw))),
    'preflight_start_distance_limit_m':.30,'prestart_clear_distance_limit_m':.15,
    'static_reference_blocked':scene['static_blocked'],'live_reference_blocked':scene['live_blocked'],
    'retained_path_max_distance_to_reference_m':scene['path_reference_max_deviation_m'],
    'actual_pose_two_clear_cycles':actual,'longitudinal_only_corrected_two_clear_cycles':counterfactual,
    'limitations':['Localization-derived pose, not an independent survey of true physical position.',
        'Counterfactual changes longitudinal pose only; no robot moved and no runtime code/parameters changed.',
        'Exact trigger that first generated the retained detour predates this recording.']}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
