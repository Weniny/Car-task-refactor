import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition_planning.semantic_planner import PathPoint
from competition_planning.startup_capture import startup_capture_path,start_offsets
from competition_planning.local_trajectory_planner import LocalReplanConfig,LocalTrajectoryPlanner
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.forward_rejoin import ObservationWindow


class StartupCaptureTests(unittest.TestCase):
    def setUp(self):
        self.ref=tuple(PathPoint(i*.05,0,0) for i in range(121))
        self.pose=PathPoint(-.289,-.0365,math.radians(6.76))
        self.kw=dict(goal_index=60,sample_spacing_m=.1,min_turning_radius_m=.6,capture_half_width_m=.1)
        self.grid=OccupancyGridMap(200,120,.05,-2,-3,(False,)*24000)
        self.cfg=LocalReplanConfig(lookahead_distance_m=3,inflation_radius_m=.04,
            min_turning_radius_m=.6,goal_position_tolerance_m=.15,goal_heading_tolerance_rad=math.radians(20),
            forward_rejoin_enabled=True,tracking_clearance_m=.1,
            planning_timeout_s=.4,empty_history_planning_timeout_s=.4,relaxed_extension_timeout_s=.35)

    def planner(self):
        lp=LocalTrajectoryPlanner(self.grid,self.cfg)
        lp._forward_rejoin_target_index=60;lp._last_safe_rejoin_index=60
        lp._last_safe_path=(self.pose,PathPoint(.3,-.2,0),PathPoint(1.5,-.38,0),self.ref[60])
        return lp

    def plan(self,p,**overrides):
        args=dict(reference_path=self.ref,current_pose=self.pose,dynamic_obstacle_points=(),
            prestart_stationary=True,observation_window=ObservationWindow(self.pose.x,self.pose.y,self.pose.yaw,-.5,-2.5,5.5,2.5))
        args.update(overrides);return p.plan(**args)

    def test_actual_rear_pose_builds_small_capture_with_correct_tangents(self):
        path=startup_capture_path(self.ref,self.pose,**self.kw)
        self.assertEqual(path[0],self.pose);self.assertEqual(path[-1],self.ref[60])
        self.assertLess(max(abs(p.y) for p in path),.06)
        self.assertTrue(all(b.x>a.x for a,b in zip(path,path[1:])))

    def test_old_curve_replaced_after_two_checked_cycles(self):
        p=self.planner();first=self.plan(p);second=self.plan(p)
        self.assertEqual(first.status,'FORWARD_REJOIN_HOLD')
        self.assertEqual(second.status,'STARTUP_CAPTURE')
        self.assertLess(max(abs(r.y) for r in second.path),.06)

    def test_moving_cycle_keeps_capture_and_does_not_return_to_old_curve(self):
        p=self.planner();self.plan(p);out=self.plan(p)
        moving=self.plan(p,prestart_stationary=False)
        self.assertEqual(moving.status,'STARTUP_CAPTURE_HOLD')
        self.assertEqual(moving.path,out.path)

    def test_rear_position_is_not_reported_as_large_lateral_error(self):
        along,cross,heading=start_offsets(self.ref[0],self.pose)
        self.assertAlmostEqual(along,-.289);self.assertAlmostEqual(cross,-.0365)

    def test_rotated_map_frame_has_equivalent_capture(self):
        a=2.6;c,s=math.cos(a),math.sin(a)
        def tx(p):return PathPoint(12+c*p.x-s*p.y,-3+s*p.x+c*p.y,p.yaw+a)
        path=startup_capture_path(tuple(tx(p) for p in self.ref),tx(self.pose),**self.kw)
        original=startup_capture_path(self.ref,self.pose,**self.kw)
        self.assertEqual(len(path),len(original))
        for p,q in zip(path,map(tx,original)):
            self.assertAlmostEqual(p.x,q.x);self.assertAlmostEqual(p.y,q.y)

    def test_outside_existing_start_gate_is_not_relaxed(self):
        self.assertFalse(startup_capture_path(self.ref,PathPoint(-.31,0,0),**self.kw))
        self.assertFalse(startup_capture_path(self.ref,PathPoint(-.2,0,math.radians(13)),**self.kw))

    def test_large_lateral_error_does_not_replace_a_detour(self):
        self.assertFalse(startup_capture_path(self.ref,PathPoint(-.2,.15,0),**self.kw))

    def test_initial_capture_collision_is_checked_even_if_reference_is_clear(self):
        p=self.planner()
        # Occupancy on the new startup curve behind the nominal reference.
        for _ in range(2):
            try:out=self.plan(p,dynamic_obstacle_points=((-.18,-.03),))
            except RuntimeError:continue
            self.assertNotEqual(out.status,'STARTUP_CAPTURE')
        self.assertNotEqual(p._startup_capture_goal,60)

    def test_obstacle_on_nominal_reference_does_not_trigger_clear_reset(self):
        p=self.planner()
        for _ in range(2):
            try:out=self.plan(p,dynamic_obstacle_points=((1.5,0),))
            except RuntimeError:continue
            self.assertNotEqual(out.status,'STARTUP_CAPTURE')
        self.assertEqual(p._prestart_clear_count,0)

    def test_short_goal_that_cannot_fit_capture_is_rejected(self):
        self.assertFalse(startup_capture_path(self.ref,self.pose,**(self.kw|{'goal_index':5})))


if __name__=='__main__':unittest.main()
