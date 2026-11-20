#!/usr/bin/env python3
"""Offline moving-horizon regression on frozen recorded obstacle sections.

Uses ideal Ackermann body kinematics, with optional measured-scale braking lag
and heading perturbation. It cannot establish field safety or perception quality.
"""
import argparse
import bisect
import dataclasses
import json
import math
from pathlib import Path
import sys
import time


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--runtime',type=Path,required=True)
    ap.add_argument('--forced-search-failures',type=int,default=2)
    ap.add_argument('--obstacle-clear-after',type=float)
    ap.add_argument('--fixtures',type=Path,required=True)
    ap.add_argument('--fixture-time',type=float,default=171.327)
    ap.add_argument('--margin',type=float,default=.10)
    ap.add_argument('--radius',type=float,default=.60)
    ap.add_argument('--yaw-bias-deg',type=float,default=0.)
    ap.add_argument('--speed-lag-s',type=float,default=0.)
    ap.add_argument('--clear-after-stop',action='store_true')
    ap.add_argument('--block-during-detour',action='store_true')
    ap.add_argument('--raw-snapshots',type=Path)
    ap.add_argument('--max-seconds',type=float,default=45.)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    for pkg in ['competition_planning','competition_control','competition_safety']:
        sys.path.insert(0,str(args.runtime/'src'/pkg))
    import numpy as np
    import yaml
    from competition_planning.local_trajectory_planner import LocalTrajectoryPlanner,LocalReplanConfig,_crop_planning_map,_overlay_obstacles
    from competition_planning.forward_rejoin import ObservationWindow
    from competition_planning.semantic_planner import PathPoint
    from competition_planning.occupancy_grid_planner import OccupancyGridMap,GridPlanningError
    from competition_planning.trajectory_parameterizer import parameterize_local_path
    from competition_control.mppi_controller import MPPIController,MPPIParams,ControlTrajectory,ControlTrajectoryPoint,VehicleState,BodyCommand,apply_mission_speed_limit
    from competition_planning.path_guard import PathGuard
    from competition_planning.hybrid_astar_planner import HybridAStarPlanner,HybridAStarTimeout
    from competition_control.local_plan_continuity import local_paths_are_equivalent
    from competition_safety.supervisor import SafetyLimits,SafetyContext,SafetySupervisor
    from competition_safety.proximity_stop import ProximityStopConfig,_point_is_in_stop_box
    fixtures=json.loads(args.fixtures.read_text());fixture=min(fixtures,key=lambda f:abs(f['t']-args.fixture_time))
    run='log/retest_obstacle_full_loop_050straight_030turn_20261102_200354_970032218'
    def params(n):return next(iter(yaml.safe_load((args.runtime/run/(n+'.yaml')).read_text()).values()))['ros__parameters']
    def config(cls,p):return {k:v for k,v in p.items() if k in {x.name for x in dataclasses.fields(cls)}}
    p=params('replanner');kw=config(LocalReplanConfig,p)
    kw.update(goal_heading_tolerance_rad=math.radians(p['goal_heading_tolerance_deg']),forward_rejoin_enabled=True,tracking_clearance_m=args.margin,min_turning_radius_m=args.radius)
    cfg=LocalReplanConfig(**kw)
    p=params('mppi');kw=config(MPPIParams,p);kw['goal_heading_tolerance_rad']=math.radians(p['goal_heading_tolerance_deg']);mp=MPPIParams(**kw)
    p=params('safety');kw=config(SafetyLimits,p)
    for name in ['recovery_heading_error','recovery_clear_heading_error','max_heading_error']:
        kw[name+'_rad']=math.radians(p[name+'_deg'])
    safety=SafetySupervisor(SafetyLimits(**kw))
    optimizer=yaml.safe_load((args.runtime/'config/planning/continuous_trajectory_radius120_050straight_030turn.yaml').read_text())
    optimizer['trajectory_optimizer']['max_curvature_1pm']=1/mp.min_turning_radius_m
    semantic=yaml.safe_load((args.runtime/'maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml').read_text())
    raw=yaml.safe_load((args.runtime/'artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml').read_text())['points']
    reference=tuple(PathPoint(v['x'],v['y'],v['yaw']) for v in raw)
    static=OccupancyGridMap.from_yaml(args.runtime/'maps/indoor_manual_final/map.yaml')
    planner=LocalTrajectoryPlanner(static,cfg)
    pose=fixture['pose'];state=VehicleState(*pose,.48);previous=max(0,fixture['reference_index']-5)
    world=np.asarray(fixture['full_points']);base_world=world.copy()
    raw_world=None
    if args.raw_snapshots:
        raw_snapshot=min(json.loads(args.raw_snapshots.read_text()),key=lambda s:abs(s['t']-fixture['t']))
        raw_world=np.asarray(raw_snapshot['raw_obstacles'])
    proximity_config=ProximityStopConfig(stop_distance_m=.40)
    base_raw_world=None if raw_world is None else raw_world.copy()
    exit_index=min(len(reference)-1,fixture['reference_index']+81);goal=reference[exit_index]
    def tx(xy,p):
        c,s=math.cos(p.yaw),math.sin(p.yaw)
        return xy@np.array([[c,s],[-s,c]])+[p.x,p.y]
    def visible(xy,p):
        c,s=math.cos(p.yaw),math.sin(p.yaw)
        body=(xy-[p.x,p.y])@np.array([[c,-s],[s,c]])
        keep=(body[:,0]>=-.5)&(body[:,0]<5.5)&(abs(body[:,1])<2.5)
        # Re-rasterize the visible occupancy into the moving body grid.
        cells=np.unique(np.floor((body[keep]-[-.5,-2.5])/.05).astype(int),axis=0)
        return tx(cells*.05+[-.475,-2.475],p)
    def trajectory(geometry):
        out=parameterize_local_path(geometry,semantic,optimizer)
        return ControlTrajectory('map','closed_loop',tuple(ControlTrajectoryPoint(v.x,v.y,v.yaw,v.s,v.curvature,v.v,v.t) for v in out.points))
    accepted=tuple(PathPoint(*v) for v in fixture['published_path'])
    ctrl=MPPIController(trajectory(accepted),mp,random_seed=7)
    planner._last_safe_path=accepted
    planner._last_safe_rejoin_index=fixture.get('rejoin_index',fixture['reference_index']+100)
    planner._forward_rejoin_target_index=planner._last_safe_rejoin_index
    budget=dict(latency_s=1.7,deceleration_mps2=.18,max_speed_mps=.5,stop_margin_m=.1)
    original_search=HybridAStarPlanner.plan
    forced_remaining=args.forced_search_failures
    active_guard=None
    def timeout_search(*a,**kw):raise HybridAStarTimeout('forced recorded-scale search timeout')

    deliveries=[];pending=None;synthetic_clear=False;local_stop=False;samples=[];plans=[];outcome='TIME_LIMIT';stopped_since=None;removed_at=None;resumed_at=None;last_k=0.
    crop=_crop_planning_map(static,PathPoint(*pose),reference[max(0,previous):exit_index+21],margin_m=2.)
    blocker=None
    if args.clear_after_stop and not args.block_during_detour:
        # Artificial wall closes the observed corridor; remove only after stopping.
        raw_body=np.asarray([(1.8,y) for y in np.arange(-2.5,2.51,.10)])
        inflated=[]
        for x,y in raw_body:
            inflated.extend((x+dx,y+dy) for dx in np.arange(-.45,.451,.05) for dy in np.arange(-.45,.451,.05) if dx*dx+dy*dy<=.44**2)
        blocker=tx(np.asarray(inflated),state);world=np.vstack([base_world,blocker])
    def collision_checker(points):
        live,_=_overlay_obstacles(crop,tuple(map(tuple,points)))
        lp=LocalTrajectoryPlanner(static,LocalReplanConfig(inflation_radius_m=.04,min_turning_radius_m=.6))
        return lp._planner(live,reference[max(0,previous):exit_index+1])
    checker=collision_checker(world)
    started=time.perf_counter();max_error=0.
    for tick in range(int(args.max_seconds/.05)):
        t=tick*.05
        if args.obstacle_clear_after is not None and t>=args.obstacle_clear_after and not synthetic_clear:
            c,s=math.cos(pose[2]),math.sin(pose[2])
            body=(world-pose[:2])@np.array([[c,-s],[s,c]])
            # Synthetic removal of the frozen corridor occupant; walls retained.
            world=world[~((body[:,0]>2.)&(body[:,0]<5.5)&(abs(body[:,1])<.8))]
            checker=collision_checker(world);synthetic_clear=True
        if args.block_during_detour and blocker is None and removed_at is None and t>=3.:
            raw_body=np.asarray([(1.8,y) for y in np.arange(-2.5,2.51,.10)])
            inflated=[]
            for x,y in raw_body:
                inflated.extend((x+dx,y+dy) for dx in np.arange(-.45,.451,.05) for dy in np.arange(-.45,.451,.05) if dx*dx+dy*dy<=.44**2)
            blocker=tx(np.asarray(inflated),state);world=np.vstack([base_world,blocker]);checker=collision_checker(world)
            wall_raw=tx(raw_body,state)
            raw_world=wall_raw if base_raw_world is None else np.vstack([base_raw_world,wall_raw])
        if args.clear_after_stop and blocker is not None and removed_at is None and state.linear_speed_mps<.02:
            stopped_since=t if stopped_since is None else stopped_since
            if t-stopped_since>=1.:
                world=base_world.copy();raw_world=base_raw_world;checker=collision_checker(world);removed_at=t
        if tick%10==0:
            timer=time.perf_counter();pts=visible(world,state)
            forced = forced_remaining > 0
            if forced:
                HybridAStarPlanner.plan=timeout_search
                forced_remaining-=1
            try:
                out=planner.plan(reference_path=reference,current_pose=PathPoint(state.x,state.y,state.yaw),dynamic_obstacle_points=tuple(map(tuple,pts)),previous_reference_index=previous,
                    observation_window=ObservationWindow(state.x,state.y,state.yaw,-.5,-2.5,5.5,2.5),
                    initial_curvature_1pm=last_k)
                plan={'t':t,'status':out.status,'rejoin':out.rejoin_index,'error':None}
                geometry=out.path;previous=out.reference_start_index
            except (GridPlanningError,ValueError) as exc:
                plan={'t':t,'status':type(exc).__name__,'error':str(exc)};geometry=None
                out=planner.continue_after_failure(reference_path=reference,
                    current_pose=PathPoint(state.x,state.y,state.yaw),dynamic_obstacle_points=tuple(map(tuple,pts)),
                    previous_reference_index=previous,observation_window=ObservationWindow(state.x,state.y,state.yaw,-.5,-2.5,5.5,2.5),
                    speed_mps=state.linear_speed_mps,**budget)
                if out is not None:
                    geometry=out.path;previous=out.reference_start_index
                    plan.update(status=out.status,clear_distance_m=out.continuation_clear_distance_m)
            finally:
                HybridAStarPlanner.plan=original_search
            elapsed=time.perf_counter()-timer
            if forced:elapsed=max(.4,elapsed)
            plan['elapsed_ms']=elapsed*1000;plans.append(plan)
            delivery=tick+max(3,math.ceil((elapsed+.05)/.05))
            stamp_ns=round((1000+delivery*.05)*1e9)
            guard=None if geometry is None else PathGuard(stamp_ns,round((1000+t+1.1)*1e9),out.speed_limit_mps or .5,
                out.status=='VALIDATED_PREFIX',1.7,.18,.1,.1,math.radians(20))
            deliveries.append((delivery,geometry,guard))
        if deliveries and tick>=deliveries[0][0]:
            pending=deliveries.pop(0)
            geometry=pending[1];active_guard=pending[2];pending=None;local_stop=geometry is None
            if geometry is None:
                accepted=None;ctrl.reset()
            elif accepted is None or not local_paths_are_equivalent(accepted,geometry,position_tolerance_m=.05,heading_tolerance_rad=math.radians(5)):
                try:ctrl.replace_trajectory(trajectory(geometry));accepted=geometry
                except ValueError as exc:local_stop=True;accepted=None;plans[-1]['parameterize_error']=str(exc)
        cmd=BodyCommand.hold(target_index=0,lateral_error_m=0.,heading_error_rad=0.,status='LOCAL_PLANNER_STOP') if local_stop else ctrl.compute_command(state)
        if active_guard is not None and accepted is not None and cmd.status=='TRACKING':
            limit=active_guard.speed_limit(accepted,PathPoint(state.x,state.y,state.yaw),now_ns=round((1000+t)*1e9),
                path_stamp_ns=active_guard.path_stamp_ns,speed_mps=state.linear_speed_mps)
            if limit is None:
                cmd=BodyCommand.hold(target_index=0,lateral_error_m=0.,heading_error_rad=0.,status='LOCAL_PLAN_GUARD_HOLD')
            else:cmd=apply_mission_speed_limit(cmd,limit)
        near_stop=False
        if raw_world is not None:
            c,s=math.cos(state.yaw),math.sin(state.yaw)
            body_raw=(raw_world-[state.x,state.y])@np.array([[c,-s],[s,c]])
            near_stop=sum(_point_is_in_stop_box(float(x),float(y),proximity_config) for x,y in body_raw)>=3
        safe=safety.filter_command(cmd,SafetyContext(1+t,1+t,1+t,state.linear_speed_mps,True,True,True,True,near_stop,False))
        target_v=safe.linear_x_mps
        v=(state.linear_speed_mps+(target_v-state.linear_speed_mps)*min(1.,.05/args.speed_lag_s)) if args.speed_lag_s>0 else target_v
        if target_v>1e-6:last_k=safe.yaw_rate_radps/target_v
        w=v*last_k
        direction=state.yaw+w*.025+math.radians(args.yaw_bias_deg)
        state=VehicleState(state.x+v*math.cos(direction)*.05,state.y+v*math.sin(direction)*.05,state.yaw+w*.05,v)
        max_error=max(max_error,abs(cmd.lateral_error_m))
        samples.append({'t':t,'x':state.x,'y':state.y,'yaw':state.yaw,'v':v,'local_stop':local_stop,'control_status':cmd.status,'safety':safe.status,'tracking_error_m':cmd.lateral_error_m,'proximity_stop':near_stop})
        if removed_at is not None and resumed_at is None and v>.05:resumed_at=t
        if not checker.path_is_navigable((PathPoint(state.x,state.y,state.yaw),)):
            outcome='PREDICTED_COLLISION';break
        along=(state.x-goal.x)*math.cos(goal.yaw)+(state.y-goal.y)*math.sin(goal.yaw)
        cross=-(state.x-goal.x)*math.sin(goal.yaw)+(state.y-goal.y)*math.cos(goal.yaw)
        if along>=0 and abs(cross)<=.35:
            outcome='PASSED_SECTION';break
    result={'fixture_t':fixture['t'],'mode':vars(args)|{'runtime':str(args.runtime),'fixtures':str(args.fixtures),'output':str(args.output)},
        'outcome':outcome,'model_s':tick*.05,'max_tracking_error_m':max_error,'removed_at_s':removed_at,'resumed_at_s':resumed_at,
        'planning_max_ms':max(p['elapsed_ms'] for p in plans),'planning_failures':sum(p['error'] is not None for p in plans),
        'planner_stop_s':sum(s['local_stop'] for s in samples)*.05,
        'guard_stop_s':sum(s['control_status']=='LOCAL_PLAN_GUARD_HOLD' for s in samples)*.05,
        'stationary_s':sum(s['v']<.02 for s in samples)*.05,
        'proximity_stop_s':sum(s['proximity_stop'] for s in samples)*.05,
        'elapsed_s':time.perf_counter()-started,'plans':plans,'samples':samples,
        'limitations':['Frozen recorded occupancy, not raw perception simulation.','Moving observation window assumes no unseen additional obstacles beyond initial snapshot.','Ideal body curvature; optional speed lag and constant heading perturbation are stress assumptions, not calibrated vehicle dynamics.','A passage crossing test, not a complete loop or parking test.']}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,default=str))
    print(json.dumps({k:v for k,v in result.items() if k not in ['plans','samples','mode','limitations']}),flush=True)


if __name__=='__main__':main()
