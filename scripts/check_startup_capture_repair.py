#!/usr/bin/env python3
"""Recorded behind-start case: geometry, rolling MPPI/Safety, and handoff."""
import argparse
import dataclasses
import json
import math
from pathlib import Path
import sys

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--runtime',type=Path,required=True)
ap.add_argument('--snapshots',type=Path,required=True)
ap.add_argument('--output',type=Path,required=True)
args=ap.parse_args()
for name in ['competition_planning','competition_control','competition_safety']:
    sys.path.insert(0,str(args.runtime/'src'/name))
import yaml
import numpy as np
from competition_planning.semantic_planner import PathPoint
from competition_planning.startup_capture import start_offsets
from competition_planning.local_trajectory_planner import LocalTrajectoryPlanner,LocalReplanConfig,_crop_planning_map,_overlay_obstacles
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.forward_rejoin import ObservationWindow
from competition_planning.trajectory_parameterizer import parameterize_local_path
from competition_control.mppi_controller import MPPIController,MPPIParams,VehicleState,ControlTrajectory,ControlTrajectoryPoint
from competition_control.local_plan_continuity import local_paths_are_equivalent
from competition_safety.supervisor import SafetySupervisor,SafetyLimits,SafetyContext

scenes=json.loads(args.snapshots.read_text());scene=min(scenes,key=lambda r:abs(r['t']-37.2))
records=yaml.safe_load((args.runtime/'artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml').read_text())['points']
ref=tuple(PathPoint(r['x'],r['y'],r['yaw']) for r in records)
static=OccupancyGridMap.from_yaml(args.runtime/'maps/indoor_manual_final/map.yaml')
cfg=LocalReplanConfig(lookahead_distance_m=3.,inflation_radius_m=.04,search_padding_m=1.5,
    min_turning_radius_m=.6,planning_timeout_s=.4,empty_history_planning_timeout_s=.4,
    relaxed_extension_timeout_s=.35,forward_rejoin_enabled=True,tracking_clearance_m=.1,
    goal_position_tolerance_m=.15,goal_heading_tolerance_rad=math.radians(20),search_heuristic_weight=1.6)
lp=LocalTrajectoryPlanner(static,cfg)
lp._last_safe_path=tuple(PathPoint(*r) for r in scene['path']);lp._last_safe_rejoin_index=scene['rejoin_index'];lp._forward_rejoin_target_index=scene['rejoin_index']
initial=PathPoint(*scene['pose']);world=np.asarray(scene['inflated_world'])
kw=dict(reference_path=ref,current_pose=initial,dynamic_obstacle_points=tuple(map(tuple,world)),
    previous_reference_index=0,prestart_stationary=True,observation_window=ObservationWindow(*scene['pose'],-.5,-2.5,5.5,2.5))
first=lp.plan(**kw);prepared=lp.plan(**kw)
assert prepared.status=='STARTUP_CAPTURE'
assert prepared.path[0]==initial
max_lateral=max(abs(start_offsets(ref[0],p)[1]) for p in prepared.path)
assert max_lateral<.08,(max_lateral,prepared.status)
max_curvature=max(abs(math.atan2(math.sin(b.yaw-a.yaw),math.cos(b.yaw-a.yaw)))/math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(prepared.path,prepared.path[1:]))
assert max_curvature<=1/.6
baseline=args.runtime/'log/retest_obstacle_full_loop_050straight_030turn_20261102_200354_970032218'
def params(n):return next(iter(yaml.safe_load((baseline/(n+'.yaml')).read_text()).values()))['ros__parameters']
def fields(cls,row):return {k:v for k,v in row.items() if k in {f.name for f in dataclasses.fields(cls)}}
row=params('mppi');values=fields(MPPIParams,row);values['goal_heading_tolerance_rad']=math.radians(row['goal_heading_tolerance_deg']);mp=MPPIParams(**values)
row=params('safety');values=fields(SafetyLimits,row)
for name in ['recovery_heading_error','recovery_clear_heading_error','max_heading_error']:
    values[name+'_rad']=math.radians(row[name+'_deg'])
safety=SafetySupervisor(SafetyLimits(**values))
semantic=yaml.safe_load((args.runtime/'maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml').read_text())
optimizer=yaml.safe_load((args.runtime/'config/planning/continuous_trajectory_radius120_050straight_030turn.yaml').read_text())
optimizer['trajectory_optimizer']['max_curvature_1pm']=1/.6
def trajectory(geometry):
    out=parameterize_local_path(geometry,semantic,optimizer)
    return ControlTrajectory('map','startup_capture_check',tuple(ControlTrajectoryPoint(v.x,v.y,v.yaw,v.s,v.curvature,v.v,v.t) for v in out.points))
accepted=prepared.path;ctrl=MPPIController(trajectory(accepted),mp,random_seed=7)
state=VehicleState(initial.x,initial.y,initial.yaw,0.);previous=0;last_k=0.;samples=[];plans=[]
crop=_crop_planning_map(static,initial,ref[:130],margin_m=2.)
live,_=_overlay_obstacles(crop,tuple(map(tuple,world)));checker=lp._planner(live,ref[:130])
goal=ref[81];outcome='TIME_LIMIT';pending=None
for tick in range(600):
    t=tick*.05
    if tick%10==0:
        c,s=math.cos(state.yaw),math.sin(state.yaw)
        body=(world-[state.x,state.y])@np.array([[c,-s],[s,c]])
        visible=world[(body[:,0]>=-.5)&(body[:,0]<5.5)&(abs(body[:,1])<2.5)]
        out=lp.plan(reference_path=ref,current_pose=PathPoint(state.x,state.y,state.yaw),
            dynamic_obstacle_points=tuple(map(tuple,visible)),previous_reference_index=previous,
            observation_window=ObservationWindow(state.x,state.y,state.yaw,-.5,-2.5,5.5,2.5),initial_curvature_1pm=last_k)
        previous=out.reference_start_index;plans.append({'t':t,'status':out.status})
        pending=(tick+3,out.path) # explicit 0.15s path delivery delay
    if pending and tick>=pending[0]:
        geometry=pending[1];pending=None
        if not local_paths_are_equivalent(accepted,geometry,position_tolerance_m=.05,heading_tolerance_rad=math.radians(5)):
            ctrl.replace_trajectory(trajectory(geometry));accepted=geometry
    cmd=ctrl.compute_command(state)
    safe=safety.filter_command(cmd,SafetyContext(1+t,1+t,1+t,state.linear_speed_mps,True,True,True,True,False,False))
    target=safe.linear_x_mps
    v=state.linear_speed_mps+(target-state.linear_speed_mps)*.2 # 0.25s velocity response
    if abs(target)>1e-6:last_k=safe.yaw_rate_radps/target
    w=v*last_k;direction=state.yaw+w*.025
    state=VehicleState(state.x+v*math.cos(direction)*.05,state.y+v*math.sin(direction)*.05,state.yaw+w*.05,v)
    samples.append({'t':t,'x':state.x,'y':state.y,'v':v,'status':cmd.status,'safety':safe.status,
        'cross_track_m':start_offsets(ref[0],PathPoint(state.x,state.y,state.yaw))[1]})
    if not checker.path_is_navigable((PathPoint(state.x,state.y,state.yaw),)):
        outcome='PREDICTED_COLLISION';break
    if ((state.x-goal.x)*math.cos(goal.yaw)+(state.y-goal.y)*math.sin(goal.yaw)>=0
        and abs(start_offsets(goal,PathPoint(state.x,state.y,state.yaw))[1])<.1):
        outcome='PASSED_START_SECTION';break
result={'status':'PASS' if outcome=='PASSED_START_SECTION' else 'FAIL','outcome':outcome,
    'recorded_pose':scene['pose'],'old_path_deviation_m':scene['path_reference_max_deviation_m'],
    'new_initial_path_lateral_max_m':max_lateral,'new_initial_path_curvature_max_1pm':max_curvature,
    'model_s':t,'max_model_lateral_m':max(abs(s['cross_track_m']) for s in samples),
    'hold_s':sum(s['status']!='TRACKING' or s['safety']=='SAFE_HOLD' for s in samples)*.05,
    'plans':plans,'samples':samples,
    'limitations':['Frozen recorded costmap, synthetic 0.25s speed response and 0.15s path delivery delay.',
        'Local start section only; no live chassis or full-loop acceptance.']}
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ['plans','samples','limitations']}))
assert result['status']=='PASS' and result['hold_s']==0
