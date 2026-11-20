#!/usr/bin/env python3
"""Meaningful offline regression of the isolated candidate and ROS callbacks."""
import argparse
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--runtime',required=True,type=Path)
args=ap.parse_args()
for package in ['competition_planning','competition_control']:
    sys.path.insert(0,str(args.runtime/'src'/package))
from competition_planning.hybrid_astar_planner import HybridAStarPlanner
from competition_planning.local_trajectory_planner import LocalTrajectoryPlanner,LocalReplanConfig
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import PathPoint
from competition_planning.forward_rejoin import ObservationWindow


def callback(path,name,globals_):
    tree=ast.parse(path.read_text(encoding='utf-8'))
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    module=ast.Module(body=[method],type_ignores=[])
    scope=dict(globals_);exec(compile(module,str(path),'exec'),scope)
    return scope[name]


class CandidateIntegration(unittest.TestCase):
    def hybrid_args(self):
        return dict(inflation_radius_m=0.,search_padding_m=1.5,sample_spacing_m=.1,
            min_turning_radius_m=.6,step_length_m=.2,curvature_bins=9,heading_bins=72,
            goal_position_tolerance_m=.15,goal_heading_tolerance_rad=math.radians(20))

    def grid(self):
        return OccupancyGridMap(width=200,height=100,resolution=.05,origin_x=-2,origin_y=-2,occupied=(False,)*20000)

    def test_current_curvature_reaches_first_search_node(self):
        class Spy(HybridAStarPlanner):
            def _search(self,start,goal,**kw):
                self.seed=kw['start_curvature_index'];return [start,goal],self.seed
        hp=Spy(self.grid(),**self.hybrid_args())
        hp.plan((PathPoint(0,0,0),PathPoint(2,0,0)),initial_curvature_1pm=.8)
        self.assertAlmostEqual(hp._curvatures[hp.seed],.8333333333333333)
        hp.plan((PathPoint(0,0,0),PathPoint(2,0,0)))
        self.assertAlmostEqual(hp._curvatures[hp.seed],0)

    def test_dense_collision_check_rejects_obstacle_between_waypoints(self):
        g=self.grid();cells=list(g.occupied);cells[g.index(g.world_to_cell(.5,0))]=True
        from dataclasses import replace
        hp=HybridAStarPlanner(replace(g,occupied=tuple(cells)),**self.hybrid_args())
        self.assertFalse(hp.path_is_navigable((PathPoint(0,0,0),PathPoint(1,0,0))))

    def test_no_reference_handoff_while_car_is_laterally_offset(self):
        lp=LocalTrajectoryPlanner(self.grid(),LocalReplanConfig(lookahead_distance_m=3,inflation_radius_m=.04,
            min_turning_radius_m=.6,goal_position_tolerance_m=.15,goal_heading_tolerance_rad=math.radians(20),forward_rejoin_enabled=True,tracking_clearance_m=.1,planning_timeout_s=.4,empty_history_planning_timeout_s=.4,relaxed_extension_timeout_s=.35))
        lp._forward_rejoin_target_index=50
        ref=tuple(PathPoint(i*.1,0,0) for i in range(81))
        out=lp.plan(reference_path=ref,current_pose=PathPoint(3,.3,0),dynamic_obstacle_points=(),previous_reference_index=30,
            observation_window=ObservationWindow(3,.3,0,-.5,-2.5,5.5,2.5))
        self.assertNotEqual(out.status,'REFERENCE_CLEAR')
        self.assertAlmostEqual(out.path[0].y,.3)

    def test_odom_retains_curvature_at_stop_and_bounds_noise(self):
        path=args.runtime/'src/competition_planning/competition_planning/local_replanner_node.py'
        cb=callback(path,'_odom_callback',{'Odometry':object,'math':math,'_stamp_to_seconds':lambda stamp:stamp.sec})
        node=NS(_initial_curvature_1pm=0.,_config=NS(min_turning_radius_m=.6),_now_s=lambda:1.)
        def msg(v,w):return NS(header=NS(stamp=NS(sec=1)),twist=NS(twist=NS(linear=NS(x=v),angular=NS(z=w))))
        cb(node,msg(.4,.3));self.assertAlmostEqual(node._initial_curvature_1pm,.75)
        cb(node,msg(0,0));self.assertAlmostEqual(node._initial_curvature_1pm,.75)
        cb(node,msg(.1,10));self.assertAlmostEqual(node._initial_curvature_1pm,1/.6)

    def test_stop_clears_displayed_path_before_publishing_stop(self):
        path=args.runtime/'src/competition_planning/competition_planning/local_replanner_node.py'
        cb=callback(path,'_publish_stop',{'Bool':lambda **k:NS(**k),'_path_message':lambda points,*a:points})
        calls=[]
        node=NS(_config=NS(forward_rejoin_enabled=True),_map_frame='map',
            _path_publisher=NS(publish=lambda v:calls.append(('path',v))),
            _stop_publisher=NS(publish=lambda v:calls.append(('stop',v.data))),
            _publish_status=lambda *a,**k:calls.append(('status',a[0])),
            get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:0)))
        cb(node,'NO_PATH')
        self.assertEqual(calls[:2],[('path',()),('stop',True)])

    def test_controller_drops_cleared_geometry_and_resets(self):
        path=args.runtime/'src/competition_control/competition_control/mppi_control_node.py'
        cb=callback(path,'_local_trajectory_callback',{'NavPath':object})
        calls=[];node=NS(_local_plan_guards={},_controller=NS(reset=lambda:calls.append('reset')),_accepted_local_geometry=object(),_latest_local_plan_stamp_s=1.)
        cb(node,NS(poses=[]))
        self.assertIsNone(node._accepted_local_geometry)
        self.assertIsNone(node._latest_local_plan_stamp_s)
        self.assertEqual(calls,['reset'])


if __name__=='__main__':unittest.main(argv=[sys.argv[0]],verbosity=2)
