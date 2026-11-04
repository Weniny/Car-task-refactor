import unittest

from competition_safety.path_aware_proximity import (
    PathAwareConfig,
    StopReleaseLatch,
    evaluate_path_aware_stop as evaluate_stop,
)
from competition_safety.proximity_stop import ProximityStopConfig


FIXED = ProximityStopConfig(stop_distance_m=1.8, min_points=3)
CONFIG = PathAwareConfig()
STRAIGHT = tuple((0.2 * index, 0.0) for index in range(16))
AROUND = (
    (0.0, 0.0),
    (0.2, 0.0),
    (0.4, -0.1),
    (0.6, -0.3),
    (0.7, -0.45),
    (0.8, -0.6),
    (1.0, -0.8),
    (1.2, -0.8),
    (1.4, -0.8),
    (1.6, -0.8),
    (1.8, -0.8),
)


def evaluate_path_aware_stop(points, path, fixed, config):
    return evaluate_stop(points, path, fixed, config, measured_speed_mps=0.25)


class PathAwareProximityTests(unittest.TestCase):
    def test_valid_bypass_does_not_inherit_fixed_box_stop(self):
        obstacle = [(1.45, 0.0, 0.0)] * 3
        decision = evaluate_path_aware_stop(obstacle, AROUND, FIXED, CONFIG)
        self.assertTrue(decision.path_valid)
        self.assertEqual(decision.fixed_count, 3)
        self.assertEqual(decision.path_collision_count, 0)
        self.assertFalse(decision.stop)
        self.assertEqual(decision.speed_limit_mps, 0.25)

    def test_obstacle_on_path_stops(self):
        obstacle = [(1.45, 0.0, 0.0)] * 3
        decision = evaluate_path_aware_stop(obstacle, STRAIGHT, FIXED, CONFIG)
        self.assertTrue(decision.stop)
        self.assertEqual(decision.reason, "obstacle_on_local_path")

    def test_path_that_passes_too_close_stops(self):
        obstacle = [(1.45, 0.0, 0.0)] * 3
        path = tuple((0.2 * index, -0.35) for index in range(16))
        path = ((0.0, 0.0), (0.2, -0.2)) + path[2:]
        decision = evaluate_path_aware_stop(obstacle, path, FIXED, CONFIG)
        self.assertTrue(decision.path_valid)
        self.assertTrue(decision.stop)

    def test_high_measured_speed_keeps_larger_emergency_distance(self):
        obstacle = [(1.45, 0.0, 0.0)] * 3
        slow = evaluate_stop(obstacle, AROUND, FIXED, CONFIG, measured_speed_mps=0.25)
        fast = evaluate_stop(obstacle, AROUND, FIXED, CONFIG, measured_speed_mps=0.5)
        self.assertFalse(slow.stop)
        self.assertTrue(fast.stop)
        self.assertGreater(fast.emergency_distance_m, slow.emergency_distance_m)

    def test_unknown_or_excess_speed_and_short_path_fall_back(self):
        obstacle = [(1.45, 0.0, 0.0)] * 3
        for speed in (None, float("nan"), 0.6):
            self.assertEqual(
                evaluate_stop(obstacle, AROUND, FIXED, CONFIG, measured_speed_mps=speed).reason,
                "fixed_fallback",
            )
        self.assertFalse(
            evaluate_path_aware_stop(obstacle, STRAIGHT[:4], FIXED, CONFIG).path_valid
        )

    def test_near_body_corridor_stops_even_with_bypass(self):
        obstacle = [(0.8, 0.1, 0.0)] * 3
        decision = evaluate_path_aware_stop(obstacle, AROUND, FIXED, CONFIG)
        self.assertTrue(decision.stop)
        self.assertEqual(decision.reason, "emergency_corridor")

    def test_missing_and_invalid_path_fall_back_to_fixed_box(self):
        obstacle = [(1.45, 0.0, 0.0)] * 3
        for path in (
            None,
            ((1.0, 0.0), (1.2, 0.0)),
            ((0.0, 0.0), (1.0, 0.0)),
            ((0.0, 0.0), (-0.2, 0.0), (-0.4, 0.0)),
        ):
            with self.subTest(path=path):
                decision = evaluate_path_aware_stop(obstacle, path, FIXED, CONFIG)
                self.assertFalse(decision.path_valid)
                self.assertTrue(decision.stop)
                self.assertEqual(decision.reason, "fixed_fallback")

    def test_slow_region_caps_speed_before_fixed_stop(self):
        obstacle = [(2.3, 0.0, 0.0)] * 3
        decision = evaluate_path_aware_stop(obstacle, AROUND, FIXED, CONFIG)
        self.assertFalse(decision.stop)
        self.assertEqual(decision.speed_limit_mps, 0.25)

    def test_clear_scene_has_no_limit(self):
        decision = evaluate_path_aware_stop([], AROUND, FIXED, CONFIG)
        self.assertFalse(decision.stop)
        self.assertIsNone(decision.speed_limit_mps)

    def test_emergency_distance_cannot_undercut_braking_bound(self):
        with self.assertRaisesRegex(ValueError, "braking bound"):
            PathAwareConfig(emergency_distance_m=0.8).validate(FIXED)

    def test_stop_release_needs_time_and_consecutive_clear_scans(self):
        latch = StopReleaseLatch(hold_s=0.4, clear_scan_count=3)
        self.assertTrue(latch.update(True, 1.0))
        self.assertTrue(latch.update(False, 1.1))
        self.assertTrue(latch.update(False, 1.5))
        self.assertFalse(latch.update(False, 1.6))
        self.assertTrue(latch.update(True, 1.7))


if __name__ == "__main__":
    unittest.main()
