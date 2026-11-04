#!/usr/bin/env python3
"""Run actual callback bodies with DDS arrival orders and clock changes."""
import argparse
import ast
from dataclasses import asdict
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest

ap=argparse.ArgumentParser();ap.add_argument('--runtime',type=Path,required=True);args=ap.parse_args()
sys.path.insert(0,str(args.runtime/'src/competition_planning'))
from competition_planning.path_guard import PathGuard
from competition_planning.semantic_planner import PathPoint

def method(file,name,scope):
    tree=ast.parse(file.read_text(encoding='utf-8'))
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    scope=dict(scope);exec(compile(ast.Module(body=[fn],type_ignores=[]),str(file),'exec'),scope)
    return scope[name]

class Callbacks(unittest.TestCase):
    def setUp(self):
        root=args.runtime/'src/competition_control/competition_control/mppi_control_node.py'
        scope=dict(NavPath=object,String=object,VehicleState=object,PathPoint=PathPoint,
            yaw_from_quaternion=lambda *a:0.,local_paths_are_equivalent=lambda *a,**kw:True,
            _stamp_to_seconds=lambda t:t.sec+t.nanosec/1e9)
        self.path_cb=method(root,'_local_trajectory_callback',scope)
        self.guard_cb=method(root,'_local_plan_guard_callback',scope)
        self.limit_cb=method(root,'_guard_speed_limit',scope)
        self.old=(PathPoint(0,0,0),PathPoint(3,0,0))
        self.node=NS(_require_local_plan_guard=True,_local_plan_guards={},_pending_guarded_path=None,
            _latest_local_plan_stamp_ns=1_000_000_000,_latest_local_plan_stamp_s=1.,
            _precision_active=False,_map_frame='map',_accepted_local_geometry=self.old,
            _local_plan_reuse_position_tolerance_m=.05,_local_plan_reuse_heading_tolerance_rad=.1,
            _local_plan_reuse_count=0,_controller=NS(reset=lambda:None))
        self.node._annotate_active_checkpoint=lambda geometry:geometry
        self.node._local_trajectory_callback=lambda m:self.path_cb(self.node,m)

    def path(self,sec=2):
        q=NS(x=0.,y=0.,z=0.,w=1.)
        poses=[NS(pose=NS(position=NS(x=r.x,y=r.y),orientation=q)) for r in self.old]
        return NS(header=NS(frame_id='map',stamp=NS(sec=sec,nanosec=0)),poses=poses)

    def guard(self,sec=2):
        ticket=PathGuard(sec*10**9,(sec+1)*10**9,.5,False,1.,.18,.1,.1,.35)
        return NS(data=json.dumps(asdict(ticket)))

    def test_path_before_guard_keeps_previous_pair_then_accepts(self):
        self.path_cb(self.node,self.path())
        self.assertEqual(self.node._latest_local_plan_stamp_ns,10**9)
        self.guard_cb(self.node,self.guard())
        self.assertEqual(self.node._latest_local_plan_stamp_ns,2*10**9)
        self.assertIsNone(self.node._pending_guarded_path)

    def test_guard_before_path_keeps_previous_pair_then_accepts(self):
        self.guard_cb(self.node,self.guard())
        self.assertEqual(self.node._latest_local_plan_stamp_ns,10**9)
        self.path_cb(self.node,self.path())
        self.assertEqual(self.node._latest_local_plan_stamp_ns,2*10**9)

    def test_wrong_guard_cannot_release_pending_path(self):
        self.path_cb(self.node,self.path())
        self.guard_cb(self.node,self.guard(3))
        self.assertEqual(self.node._latest_local_plan_stamp_ns,10**9)

    def test_guard_expiry_holds_even_with_fresh_geometry(self):
        self.guard_cb(self.node,self.guard());self.path_cb(self.node,self.path())
        state=NS(x=0.,y=0.,yaw=0.,linear_speed_mps=.5)
        self.assertEqual(self.limit_cb(self.node,state,2_100_000_000),.5)
        self.assertIsNone(self.limit_cb(self.node,state,3_000_000_000))

    def test_clear_path_removes_tickets_pending_path_and_geometry(self):
        self.guard_cb(self.node,self.guard());message=self.path();message.poses=[]
        self.path_cb(self.node,message)
        self.assertEqual(self.node._local_plan_guards,{})
        self.assertIsNone(self.node._accepted_local_geometry)
        self.assertEqual(self.node._latest_local_plan_stamp_ns,0)

    def test_stale_controller_state_or_moving_odom_cannot_reset_detour(self):
        path=args.runtime/'src/competition_planning/competition_planning/local_replanner_node.py'
        cb=method(path,'_prestart_stationary',{})
        node=NS(_behavior_guard_enabled=True,_controller_status_received_s=1.,
            _guard_command_delay_s=.15,_controller_status={'route_enabled':False,'status':'ROUTE_DISABLED'},
            _linear_speed_mps=0.,_odom_header_stamp_s=1.,_max_odom_age_s=.5,
            _obstacle_header_stamp_s=1.,_max_obstacle_age_s=.9,_prestart_costmap_stamp_s=None)
        self.assertTrue(cb(node,1.1))
        self.assertFalse(cb(node,1.2)) # same costmap is not a second observation
        node._obstacle_header_stamp_s=1.2;node._linear_speed_mps=.1
        self.assertFalse(cb(node,1.2))
        node._linear_speed_mps=0.;node._obstacle_header_stamp_s=2.
        self.assertFalse(cb(node,2.)) # controller state is stale

    def test_prefix_cannot_release_a_car_that_has_already_held(self):
        path=args.runtime/'src/competition_planning/competition_planning/local_replanner_node.py'
        cb=method(path,'_can_continue',{})
        node=NS(_controller_status_received_s=1.,_guard_command_delay_s=.15,
            _controller_status={'route_enabled':True,'status':'TRACKING'})
        self.assertTrue(cb(node,1.1))
        for status in ['LOCAL_PLAN_GUARD_HOLD','LOCAL_PLANNER_STOP','INVALID_STATE','ROUTE_DISABLED']:
            node._controller_status['status']=status
            self.assertFalse(cb(node,1.1))


if __name__=='__main__':unittest.main(argv=[sys.argv[0]],verbosity=2)
