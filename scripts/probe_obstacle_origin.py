#!/usr/bin/env python3
"""Single-factor, pure-planner probes of recorded pre-stop inputs; no ROS."""
import argparse
import dataclasses
import json
import math
from pathlib import Path
import sys
import time


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--baseline-root',required=True,type=Path)
    ap.add_argument('--fixtures',required=True,type=Path)
    ap.add_argument('--mode',choices=['baseline','long_budget','no_short_rejoin','competition','forward_goal_only','fixed_goal_soft_clearance'],required=True)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    src=(Path('/home/agilex/competition_ws') if args.mode=='competition' else args.baseline_root)
    sys.path.insert(0,str(src/'src/competition_planning'))
    import yaml
    from competition_planning.local_trajectory_planner import (
        LocalTrajectoryPlanner,LocalReplanConfig,_crop_planning_map,_overlay_obstacles,_lookahead_index)
    from competition_planning.semantic_planner import PathPoint
    from competition_planning.occupancy_grid_planner import OccupancyGridMap
    run='log/retest_obstacle_full_loop_050straight_030turn_20261102_200354_970032218'
    p=yaml.safe_load((args.baseline_root/run/'replanner.yaml').read_text())['/rebuild_local_replanner']['ros__parameters']
    if args.mode=='competition':p.update(yaml.safe_load((src/'config/planning/local_hybrid_astar_runtime_params_day5.yaml').read_text())['local_hybrid_astar_runtime'])
    f={v.name for v in dataclasses.fields(LocalReplanConfig)}
    kw={k:v for k,v in p.items() if k in f};kw['goal_heading_tolerance_rad']=math.radians(p['goal_heading_tolerance_deg'])
    if args.mode=='long_budget':kw.update(planning_timeout_s=2.,empty_history_planning_timeout_s=2.)
    if args.mode=='no_short_rejoin':kw['minimum_short_rejoin_distance_m']=3.
    if args.mode=='fixed_goal_soft_clearance':kw.update(obstacle_clearance_distance_m=.2,obstacle_clearance_weight=.8)
    cfg=LocalReplanConfig(**kw)
    data=yaml.safe_load((args.baseline_root/'artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml').read_text())
    ref=tuple(PathPoint(v['x'],v['y'],v['yaw']) for v in data['points'])
    static=OccupancyGridMap.from_yaml(args.baseline_root/'maps/indoor_manual_final/map.yaml')
    results=[]
    for case in json.loads(args.fixtures.read_text()):
        cp=PathPoint(*case['pose']);idx=case['reference_index'];end=_lookahead_index(ref,idx,3.)
        local=ref[idx:end+1];lp=LocalTrajectoryPlanner(static,cfg)
        crop=_crop_planning_map(static,cp,local,margin_m=cfg.search_padding_m+cfg.inflation_radius_m+cfg.step_length_m)
        live,_=_overlay_obstacles(crop,tuple(map(tuple,case['dynamic_points'])))
        checker=lp._planner(live,local)
        blocked=next((i for i in range(idx,end+1) if not checker.path_is_navigable((ref[i],))),None)
        row={'t':case['t'],'first_blocked_reference_index':blocked,'reference_index':idx,
             'input_selection_residual_s':case['input_selection_residual_s']}
        started=time.perf_counter()
        try:
            if args.mode in ['forward_goal_only','fixed_goal_soft_clearance']:
                # Diagnostic only: vary goal distance, retain baseline search settings.
                # Validate every candidate against all occupied cells of the recorded grid.
                attempts=[];out=None
                distances=[(d,_lookahead_index(ref,idx,d)) for d in [3.5,4.,4.5]]
                if args.mode=='fixed_goal_soft_clearance':
                    prior=json.loads((args.fixtures.parent/'probe_forward_goal_only.json').read_text())['results']
                    chosen=min(prior,key=lambda z:abs(z['t']-case['t']))
                    distances=[(chosen['goal_attempts'][-1]['distance_m'],chosen['rejoin_index'])]
                for distance,target in distances:
                    rr=ref[idx:target+1]
                    cm=_crop_planning_map(static,cp,rr,margin_m=cfg.search_padding_m+cfg.inflation_radius_m+cfg.step_length_m)
                    lm,_=_overlay_obstacles(cm,tuple(map(tuple,case['dynamic_points'])))
                    h=lp._planner(lm,rr)
                    try:
                        route=h.plan((cp,ref[target]))
                        fm,_=_overlay_obstacles(cm,tuple(map(tuple,case['full_points'])))
                        valid=lp._planner(fm,rr).path_is_navigable(route)
                        attempts.append({'distance_m':distance,'status':'FOUND','full_grid_clear':valid})
                        if valid:
                            from types import SimpleNamespace
                            out=SimpleNamespace(path=route,rejoin_index=target,status='FORWARD_GOAL_DIAGNOSTIC')
                            break
                    except Exception as exc:attempts.append({'distance_m':distance,'error':str(exc)})
                row['goal_attempts']=attempts
                if out is None:raise ValueError('no fully validated forward-goal path')
            else:
                out=lp.plan(reference_path=ref,current_pose=cp,dynamic_obstacle_points=tuple(map(tuple,case['dynamic_points'])),previous_reference_index=max(0,idx-5))
            cur=[]
            for a,b in zip(out.path,out.path[1:]):
                ds=math.hypot(b.x-a.x,b.y-a.y)
                cur.append(math.atan2(math.sin(b.yaw-a.yaw),math.cos(b.yaw-a.yaw))/max(ds,1e-6))
            row.update(status=out.status,rejoin_index=out.rejoin_index,points=len(out.path),
                       max_abs_curvature=max(map(abs,cur),default=0),
                       rejoin_before_first_blockage=blocked is not None and out.rejoin_index<blocked,
                       path=[dataclasses.asdict(v) for v in out.path])
        except Exception as exc:row.update(status=type(exc).__name__,error=str(exc),attempts=getattr(exc,'rejoin_attempts',[]))
        row['elapsed_ms']=(time.perf_counter()-started)*1000;results.append(row)
        print(json.dumps({k:v for k,v in row.items() if k!='path'}),flush=True)
    args.output.write_text(json.dumps({'mode':args.mode,'config':dataclasses.asdict(cfg),'results':results},indent=2))


if __name__=='__main__':main()
