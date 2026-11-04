#!/usr/bin/env python3
"""Replay pre-stop fixtures against an isolated frozen-baseline candidate."""
import argparse
import dataclasses
import json
import math
from pathlib import Path
import sys
import time


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--runtime',required=True,type=Path)
    ap.add_argument('--fixtures',required=True,type=Path)
    ap.add_argument('--mode',choices=['goals_only','tracking_margin','radius081'],required=True)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args();sys.path.insert(0,str(args.runtime/'src/competition_planning'))
    import yaml
    from competition_planning.local_trajectory_planner import LocalTrajectoryPlanner,LocalReplanConfig,_crop_planning_map,_overlay_obstacles
    from competition_planning.occupancy_grid_planner import OccupancyGridMap
    from competition_planning.semantic_planner import PathPoint
    from competition_planning.forward_rejoin import ObservationWindow
    run='log/retest_obstacle_full_loop_050straight_030turn_20261102_200354_970032218'
    p=yaml.safe_load((args.runtime/run/'replanner.yaml').read_text())['/rebuild_local_replanner']['ros__parameters']
    fields={v.name for v in dataclasses.fields(LocalReplanConfig)}
    kw={k:v for k,v in p.items() if k in fields};kw['goal_heading_tolerance_rad']=math.radians(p['goal_heading_tolerance_deg'])
    kw.update(forward_rejoin_enabled=True,tracking_clearance_m=0 if args.mode=='goals_only' else .10)
    if args.mode=='radius081':kw['min_turning_radius_m']=.81
    cfg=LocalReplanConfig(**kw)
    raw=yaml.safe_load((args.runtime/'artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml').read_text())['points']
    reference=tuple(PathPoint(v['x'],v['y'],v['yaw']) for v in raw)
    grid=OccupancyGridMap.from_yaml(args.runtime/'maps/indoor_manual_final/map.yaml')
    rows=[]
    for fixture in json.loads(args.fixtures.read_text()):
        row={'t':fixture['t']};cp=PathPoint(*fixture['pose']);planner=LocalTrajectoryPlanner(grid,cfg)
        started=time.perf_counter()
        try:
            result=planner.plan(reference_path=reference,current_pose=cp,
                dynamic_obstacle_points=tuple(map(tuple,fixture['dynamic_points'])),
                previous_reference_index=max(0,fixture['reference_index']-5),
                observation_window=ObservationWindow(*fixture['full_grid_pose'],*fixture['full_grid_bounds']))
            row['planning_elapsed_ms']=(time.perf_counter()-started)*1000
            row.update(status=result.status,rejoin_index=result.rejoin_index,
                       path=[dataclasses.asdict(v) for v in result.path])
            # Independent full-grid, dense check against the original hard envelope.
            checker_lp=LocalTrajectoryPlanner(grid,dataclasses.replace(cfg,forward_rejoin_enabled=False,tracking_clearance_m=0))
            crop=_crop_planning_map(grid,cp,result.path,margin_m=1.74)
            live,_=_overlay_obstacles(crop,tuple(map(tuple,fixture['full_points'])))
            checker=checker_lp._planner(live,result.path)
            dense=[]
            for a,b in zip(result.path,result.path[1:]):
                n=max(1,math.ceil(math.hypot(b.x-a.x,b.y-a.y)/.005))
                angle=math.atan2(math.sin(b.yaw-a.yaw),math.cos(b.yaw-a.yaw))
                dense.extend(PathPoint(a.x+(b.x-a.x)*j/n,a.y+(b.y-a.y)*j/n,a.yaw+angle*j/n) for j in range(n))
            dense.append(result.path[-1]);row['full_grid_dense_clear']=checker.path_is_navigable(dense)
            first_blocked=next((i for i in range(fixture['reference_index']+1,min(len(reference),fixture['reference_index']+62))
                                if not checker.path_is_navigable((reference[i],))),None)
            row['first_blocked_reference_index']=first_blocked
            row['rejoin_before_recorded_blockage']=first_blocked is not None and result.rejoin_index<=first_blocked
        except Exception as exc:row.update(status=type(exc).__name__,error=str(exc))
        row['total_elapsed_ms']=(time.perf_counter()-started)*1000
        rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='path'}),flush=True)
    args.output.write_text(json.dumps({'mode':args.mode,'config':dataclasses.asdict(cfg),'results':rows},indent=2))


if __name__=='__main__':main()
