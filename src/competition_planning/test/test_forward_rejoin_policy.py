import sys
from pathlib import Path
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from competition_planning.forward_rejoin import select_forward_rejoin,ObservationWindow,path_capture_error


class ForwardRejoinPolicyTests(unittest.TestCase):
    def route(self):
        return [SimpleNamespace(x=i*.1,y=0.) for i in range(61)]

    def select(self, blocked=(), last_observed=45):
        return select_forward_rejoin(self.route(),start_index=0,nominal_index=30,
            maximum_index=45,clear_run_m=.6,
            is_clear=lambda p: round(p.x*10) not in blocked and p.x<=last_observed*.1+1e-9)

    def test_clear_reference_keeps_nominal_goal(self):
        self.assertEqual(self.select().goal_index,30)

    def test_far_obstacle_does_not_trigger_early_detour(self):
        self.assertIsNone(self.select(set(range(35,40))).blocked_index)

    def test_goal_is_after_blockage_and_clear_run(self):
        s=self.select(set(range(27,36)))
        self.assertEqual(s.blocked_index,27)
        self.assertEqual(s.goal_index,45)

    def test_no_pre_obstacle_short_goal_when_exit_is_blocked(self):
        s=self.select(set(range(27,46)))
        self.assertIsNone(s.goal_index)

    def test_segment_leaving_blockage_is_not_counted_as_clear(self):
        # Only five clear inter-point segments remain, so .6m is unavailable.
        self.assertIsNone(self.select(set(range(27,40))).goal_index)

    def test_unobserved_space_is_not_a_free_rejoin(self):
        self.assertIsNone(self.select(set(range(27,36)),last_observed=40).goal_index)

    def test_second_blockage_does_not_erase_an_earlier_usable_exit(self):
        s=self.select(set(range(20,26))|set(range(40,46)))
        self.assertEqual(s.goal_index,39)

    def test_reject_invalid_intervals(self):
        with self.assertRaises(ValueError):
            select_forward_rejoin(self.route(),start_index=30,nominal_index=30,
                maximum_index=45,clear_run_m=.6,is_clear=lambda p:True)

    def test_rotated_observation_window_reserves_body_and_tracking_space(self):
        import math
        window=ObservationWindow(10.,20.,math.pi/2,-.5,-2.5,5.5,2.5)
        self.assertTrue(window.contains(SimpleNamespace(x=10.,y=24.9),.54))
        self.assertFalse(window.contains(SimpleNamespace(x=10.,y=25.),.54))

    def test_capture_distance_uses_segment_not_nearest_vertex(self):
        line=[SimpleNamespace(x=0.,y=0.,yaw=0.),SimpleNamespace(x=1.,y=0.,yaw=0.)]
        distance,heading=path_capture_error(line,SimpleNamespace(x=.5,y=.02,yaw=.01))
        self.assertAlmostEqual(distance,.02)
        self.assertAlmostEqual(heading,.01)


if __name__=='__main__':unittest.main()
