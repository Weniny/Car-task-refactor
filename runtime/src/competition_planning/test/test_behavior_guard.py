import dataclasses
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition_planning.path_guard import PathGuard, project_path, stopping_distance, speed_for_distance, validated_prefix
from competition_planning.forward_rejoin import ObservationWindow
from competition_planning.local_trajectory_planner import LocalReplanConfig, LocalTrajectoryPlanner
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import PathPoint


class BehaviorGuardTests(unittest.TestCase):
    def setUp(self):
        self.ref=tuple(PathPoint(i*.1,0.,0.) for i in range(71))
        self.pose=PathPoint(.05,0.,0.)
        self.view=ObservationWindow(0,0,0,-.5,-2.5,5.5,2.5)
        self.grid=OccupancyGridMap(200,100,.05,-2,-2,(False,)*20000)
        self.cfg=LocalReplanConfig(lookahead_distance_m=3.,inflation_radius_m=.04,
            min_turning_radius_m=.6,planning_timeout_s=.4,empty_history_planning_timeout_s=.4,
            relaxed_extension_timeout_s=.35,forward_rejoin_enabled=True,tracking_clearance_m=.1,
            goal_position_tolerance_m=.15,goal_heading_tolerance_rad=math.radians(20))

    def planner(self):
        p=LocalTrajectoryPlanner(self.grid,self.cfg)
        p._last_safe_path=self.ref[:51]
        p._last_safe_rejoin_index=50
        p._forward_rejoin_target_index=50
        return p

    def continuation(self,p=None,obstacle_x=3.,speed=.5,pose=None):
        return (p or self.planner()).continue_after_failure(reference_path=self.ref,
            current_pose=pose or self.pose,dynamic_obstacle_points=((obstacle_x,0.),),
            previous_reference_index=0,observation_window=self.view,speed_mps=speed,
            latency_s=1.5,deceleration_mps2=.18,max_speed_mps=.5,stop_margin_m=.1)

    def ticket(self,**kw):
        return dataclasses.replace(PathGuard(1_000_000_000,2_000_000_000,.5,True,1.5,.18,.1,.1,math.radians(20)),**kw)

    def test_clear_reference_discards_prestart_detour_after_two_observations(self):
        p=self.planner()
        # An old detour is still captured but no longer necessary.
        p._last_safe_path=tuple(PathPoint(r.x,.4*math.sin(math.pi*r.x/5)**2,0) for r in self.ref[:51])
        kw=dict(reference_path=self.ref,current_pose=PathPoint(0,.04,0),dynamic_obstacle_points=(),
            observation_window=self.view,prestart_stationary=True)
        p.plan(**kw)
        out=p.plan(**kw)
        self.assertEqual(out.status,'REFERENCE_CLEAR')
        self.assertIsNone(p._forward_rejoin_target_index)
        self.assertTrue(all(r.y==0 for r in out.path))

    def test_moving_route_does_not_discard_detour(self):
        p=self.planner()
        out=p.plan(reference_path=self.ref,current_pose=self.pose,dynamic_obstacle_points=(),
            observation_window=self.view,prestart_stationary=False)
        self.assertEqual(out.status,'FORWARD_REJOIN_HOLD')
        self.assertEqual(p._forward_rejoin_target_index,50)

    def test_current_blockage_prevents_prestart_clear(self):
        p=self.planner()
        for _ in range(2):
            try:
                out=p.plan(reference_path=self.ref,current_pose=self.pose,dynamic_obstacle_points=((1.5,0.),),
                    observation_window=self.view,prestart_stationary=True)
                self.assertNotEqual(out.status,'REFERENCE_CLEAR')
            except RuntimeError:
                pass
        self.assertEqual(p._prestart_clear_count,0)

    def test_far_blockage_yields_checked_prefix_and_retains_original_goal(self):
        p=self.planner();out=self.continuation(p)
        self.assertIsNotNone(out)
        self.assertEqual(out.status,'VALIDATED_PREFIX')
        self.assertLess(out.path[-1].x,3.)
        self.assertGreater(out.continuation_clear_distance_m,2.)
        self.assertAlmostEqual(out.speed_limit_mps,.5)
        self.assertEqual(p._forward_rejoin_target_index,50)
        self.assertEqual(p._last_safe_path[-1].x,5.)

    def test_near_blockage_cannot_fit_stopping_distance(self):
        self.assertIsNone(self.continuation(obstacle_x=1.,speed=.5))

    def test_no_history_never_creates_a_fake_prefix(self):
        p=self.planner();p._last_safe_path=()
        self.assertIsNone(self.continuation(p))

    def test_large_capture_offset_does_not_bridge_to_old_path(self):
        self.assertIsNone(self.continuation(pose=PathPoint(.05,.2,0)))

    def test_prefix_ends_at_first_bad_segment_even_if_route_is_clear_beyond(self):
        out=validated_prefix(self.ref,self.pose,is_clear=lambda path:all(r.x<2 for r in path),
            is_observed=lambda p:True,capture_distance_m=.1,capture_heading_rad=.3)
        self.assertLess(out[-1].x,2)

    def test_observation_boundary_caps_prefix(self):
        out=validated_prefix(self.ref,self.pose,is_clear=lambda p:True,
            is_observed=lambda p:p.x<=1.5,capture_distance_m=.1,capture_heading_rad=.3)
        self.assertAlmostEqual(out[-1].x,1.5)

    def test_braking_limit_satisfies_budget(self):
        for distance in [.1,.5,1.,3.]:
            v=speed_for_distance(distance,1.5,.18)
            self.assertAlmostEqual(stopping_distance(v,1.5,.18),distance)

    def test_stale_or_mismatched_ticket_holds(self):
        for now,stamp in [(2_000_000_000,1_000_000_000),(1_000_000_001,999_000_000),(900_000_000,1_000_000_000)]:
            self.assertIsNone(self.ticket().speed_limit(self.ref,self.pose,now_ns=now,path_stamp_ns=stamp,speed_mps=.5))

    def test_controller_rechecks_remaining_distance_as_vehicle_advances(self):
        ticket=self.ticket()
        self.assertAlmostEqual(ticket.speed_limit(self.ref,self.pose,now_ns=1_100_000_000,path_stamp_ns=ticket.path_stamp_ns,speed_mps=.5),.5)
        self.assertIsNone(ticket.speed_limit(self.ref,PathPoint(6.5,0,0),now_ns=1_100_000_000,path_stamp_ns=ticket.path_stamp_ns,speed_mps=.5))

    def test_nonfinite_guard_and_braking_inputs_rejected(self):
        import json
        bad=dataclasses.asdict(self.ticket());bad['speed_limit_mps']=float('nan')
        with self.assertRaises(ValueError):PathGuard.parse(json.dumps(bad))
        with self.assertRaises(ValueError):stopping_distance(.5,1.,0.)

    def test_projection_does_not_count_path_behind_vehicle(self):
        self.assertAlmostEqual(project_path(self.ref,PathPoint(2.25,0,0)).remaining_m,4.75)


if __name__=='__main__':unittest.main()
