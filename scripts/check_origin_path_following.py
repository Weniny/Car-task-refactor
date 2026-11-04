#!/usr/bin/env python3
"""Optimistic fixed-path MPPI/Safety model check, not a field acceptance test."""
import argparse
import dataclasses
import json
import math
from pathlib import Path
import sys


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--baseline-root',required=True,type=Path)
    ap.add_argument('--input',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    ap.add_argument('--geometry-only',action='store_true')
    ap.add_argument('--paths-file',type=Path)
    args=ap.parse_args()
    for pkg in ['competition_control','competition_planning','competition_safety']:
        sys.path.insert(0,str(args.baseline_root/'src'/pkg))
    import yaml
    from competition_control.mppi_controller import MPPIController,MPPIParams,ControlTrajectory,ControlTrajectoryPoint,VehicleState
    from competition_planning.local_trajectory_planner import LocalTrajectoryPlanner,LocalReplanConfig,_crop_planning_map,_overlay_obstacles
    from competition_planning.occupancy_grid_planner import OccupancyGridMap
    from competition_planning.semantic_planner import PathPoint
    from competition_planning.trajectory_parameterizer import parameterize_local_path
    from competition_safety.supervisor import SafetySupervisor,SafetyLimits,SafetyContext
    run='log/retest_obstacle_full_loop_050straight_030turn_20261102_200354_970032218'
    def load(n):return next(iter(yaml.safe_load((args.baseline_root/run/(n+'.yaml')).read_text()).values()))['ros__parameters']
    p=load('mppi');fs={x.name for x in dataclasses.fields(MPPIParams)}
    kw={k:v for k,v in p.items() if k in fs};kw['goal_heading_tolerance_rad']=math.radians(p['goal_heading_tolerance_deg'])
    params=MPPIParams(**kw)
    p=load('safety');fs={x.name for x in dataclasses.fields(SafetyLimits)}
    sk={k:v for k,v in p.items() if k in fs}
    for k in ['recovery_heading_error','recovery_clear_heading_error','max_heading_error']:
        if k+'_deg' in p:sk[k+'_rad']=math.radians(p[k+'_deg'])
    optimizer=yaml.safe_load((args.baseline_root/'config/planning/continuous_trajectory_radius120_050straight_030turn.yaml').read_text())
    optimizer['trajectory_optimizer'].update(max_curvature_1pm=1/params.min_turning_radius_m)
    semantic=yaml.safe_load((args.baseline_root/'maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml').read_text())
    grid=OccupancyGridMap.from_yaml(args.baseline_root/'maps/indoor_manual_final/map.yaml')
    fixtures=json.loads((args.input/'origin_fixtures.json').read_text())
    goals=json.loads((args.paths_file or args.input/'probe_forward_goal_only.json').read_text())['results']
    output=[]
    for goal in goals:
        if 'path' not in goal:continue
        fixture=min(fixtures,key=lambda f:abs(f['t']-goal['t']))
        geometry=tuple(PathPoint(**p) for p in goal['path'])
        par=parameterize_local_path(geometry,semantic,optimizer)
        trajectory=ControlTrajectory('map','origin_fixed_path',tuple(ControlTrajectoryPoint(v.x,v.y,v.yaw,v.s,v.curvature,v.v,v.t) for v in par.points))
        ctrl=MPPIController(trajectory,params,random_seed=7)
        safe=SafetySupervisor(SafetyLimits(**sk))
        cp=PathPoint(*fixture['pose']);state=VehicleState(cp.x,cp.y,cp.yaw,.48)
        crop=_crop_planning_map(grid,cp,geometry,margin_m=1.74)
        live,_=_overlay_obstacles(crop,tuple(map(tuple,fixture['full_points'])))
        checker=LocalTrajectoryPlanner(grid,LocalReplanConfig(inflation_radius_m=.04,min_turning_radius_m=.6))._planner(live,geometry)
        def densely_clear(points):
            samples=[]
            for a,b in zip(points,points[1:]):
                distance=math.hypot(b.x-a.x,b.y-a.y);count=max(1,math.ceil(distance/.005))
                angle=math.atan2(math.sin(b.yaw-a.yaw),math.cos(b.yaw-a.yaw))
                for j in range(count):
                    f=j/count;samples.append(PathPoint(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),a.yaw+f*angle))
            samples.append(PathPoint(points[-1].x,points[-1].y,points[-1].yaw))
            bad=[i for i,p in enumerate(samples) if not checker.path_is_navigable((p,))]
            return {'clear':not bad,'samples':len(samples),'blocked_samples':len(bad),
                    'first_bad_pose':dataclasses.asdict(samples[bad[0]]) if bad else None}
        geometry_check={'raw_geometry':densely_clear(geometry),'parameterized_geometry':densely_clear(par.points)}
        if args.geometry_only:
            geometry_check['lateral_offset_probe']={str(offset):densely_clear(tuple(
                PathPoint(p.x-math.sin(p.yaw)*offset,p.y+math.cos(p.yaw)*offset,p.yaw)
                for p in geometry))['clear'] for offset in [-.1,-.05,-.02,-.01,-.005,0,.005,.01,.02,.05,.1]}
            row={'fixture_t':fixture['t'],**geometry_check};output.append(row);print(json.dumps(row),flush=True);continue
        samples=[];outcome='TIME_LIMIT';maxerr=0
        for tick in range(800):
            now=1+tick*.05;cmd=ctrl.compute_command(state)
            filtered=safe.filter_command(cmd,SafetyContext(now,now,now,state.linear_speed_mps,True,True,True,True,False,False))
            v,w=filtered.linear_x_mps,filtered.yaw_rate_radps
            direction=state.yaw+w*.025
            state=VehicleState(state.x+v*math.cos(direction)*.05,state.y+v*math.sin(direction)*.05,state.yaw+w*.05,v)
            maxerr=max(maxerr,abs(cmd.lateral_error_m))
            samples.append({'t':tick*.05,'x':state.x,'y':state.y,'yaw':state.yaw,'v':v,'safety':filtered.status})
            if not checker.path_is_navigable((PathPoint(state.x,state.y,state.yaw),)):
                outcome='PREDICTED_COLLISION_WITH_INFLATED_GRID';break
            if math.hypot(state.x-geometry[-1].x,state.y-geometry[-1].y)<.15:
                outcome='REACHED_LOCAL_GOAL';break
        row={'fixture_t':fixture['t'],'outcome':outcome,'model_duration_s':tick*.05,'max_lateral_error_m':maxerr,'samples':samples,**geometry_check}
        print(json.dumps({k:v for k,v in row.items() if k!='samples'}),flush=True);output.append(row)
    args.output.write_text(json.dumps({'assumptions':['Ideal body-command execution; no wheel delay, no perception delay, no path replacement, static recorded obstacles.','Initial speed 0.48 m/s; deterministic seed 7; original runtime MPPI/Safety parameters.','Failure is a model counterexample; success would not establish field safety or closed-loop avoidance.'],'results':output}))


if __name__=='__main__':main()
