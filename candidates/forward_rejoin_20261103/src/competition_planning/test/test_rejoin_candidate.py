"""Candidate search guidance and short-horizon fallback regression tests."""

import math
import unittest
from unittest.mock import Mock

from competition_planning.hybrid_astar_planner import (
    HybridAStarPlanner,
    HybridAStarTimeout,
)
from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
)
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


def grid(width=12, height=12, occupied=()):
    blocked = set(occupied)
    return OccupancyGridMap(
        width, height, 1.0, 0.0, 0.0,
        tuple((col, row) in blocked for row in range(height) for col in range(width)),
    )


def hybrid(map_, **kwargs):
    return HybridAStarPlanner(
        map_, inflation_radius_m=0.0, search_padding_m=4.0,
        sample_spacing_m=0.1, min_turning_radius_m=0.81,
        step_length_m=0.2, curvature_bins=9, heading_bins=72,
        goal_position_tolerance_m=0.15,
        goal_heading_tolerance_rad=math.radians(20), **kwargs,
    )


class ShortRejoinTests(unittest.TestCase):
    def search(self, minimum, fallback):
        config = LocalReplanConfig(
            lookahead_distance_m=3.0,
            minimum_short_rejoin_distance_m=minimum,
        )
        planner = LocalTrajectoryPlanner(grid(), config)
        reference = tuple(PathPoint(i / 10, 0.5, 0.0) for i in range(31))
        primary = Mock()
        primary.plan.side_effect = HybridAStarTimeout("full horizon timed out")
        secondary = Mock()
        secondary.plan.side_effect = fallback
        planner._planner = Mock(return_value=secondary)
        return planner, primary, secondary, reference

    def run_search(self, planner, primary, reference):
        return planner._plan_with_short_rejoin(
            reference_path=reference, current_pose=reference[0],
            live_map=grid(), start_index=0, rejoin_index=30,
            local_reference=reference, planner=primary,
            relaxed=False, reduced_curvature_lattice=False,
        )

    def test_legacy_short_fallback_is_unchanged(self):
        def fallback(poses):
            if poses[-1].x >= 2.0:
                raise GridPlanningError("short goal blocked")
            return poses
        planner, primary, secondary, reference = self.search(0.0, fallback)
        result = self.run_search(planner, primary, reference)
        self.assertTrue(result[-1])
        self.assertAlmostEqual(result[0][-1].x, 1.2)
        self.assertEqual(secondary.plan.call_count, 2)

    def test_guard_skips_inadequate_goal_and_preserves_primary_error(self):
        planner, primary, secondary, reference = self.search(
            1.8, GridPlanningError("short goal blocked")
        )
        with self.assertRaises(HybridAStarTimeout) as caught:
            self.run_search(planner, primary, reference)
        attempts = caught.exception.rejoin_attempts
        self.assertEqual([a["rejoin_index"] for a in attempts], [30, 20])
        self.assertEqual(attempts[1]["detail"], "short goal blocked")
        self.assertTrue(all(a["elapsed_ms"] >= 0 for a in attempts))
        self.assertEqual(secondary.plan.call_count, 1)

    def test_guard_checks_actual_path_length(self):
        planner, primary, _, reference = self.search(
            1.8, lambda poses: (poses[0], PathPoint(1.7, 0.5, 0.0))
        )
        with self.assertRaises(HybridAStarTimeout) as caught:
            self.run_search(planner, primary, reference)
        self.assertIn("covers only 1.700", caught.exception.rejoin_attempts[1]["detail"])

    def test_guard_accepts_sufficient_short_path(self):
        planner, primary, _, reference = self.search(1.8, lambda poses: poses)
        result = self.run_search(planner, primary, reference)
        self.assertEqual(result[1], 20)
        self.assertTrue(result[-1])

    def test_minimum_length_config_validation(self):
        for minimum in (-1.0, math.nan, math.inf, 3.1):
            with self.subTest(minimum=minimum), self.assertRaises(ValueError):
                LocalReplanConfig(lookahead_distance_m=3.0,
                                  minimum_short_rejoin_distance_m=minimum)


class ObstacleHeuristicTests(unittest.TestCase):
    def test_distance_field_includes_wall_detour(self):
        map_ = grid(occupied=((5, row) for row in range(2, 11)))
        planner = hybrid(map_, obstacle_aware_heuristic=True)
        start = PathPoint(*map_.cell_to_world((3, 6)), 0.0)
        goal = PathPoint(*map_.cell_to_world((8, 6)), 0.0)
        field = planner._build_goal_distance_field(goal, (0, 11, 0, 11), None)
        self.assertGreater(field[map_.index((3, 6))], 5.0)
        self.assertTrue(math.isinf(field[map_.index((5, 6))]))

    def test_distance_field_does_not_cut_diagonal_corners(self):
        map_ = grid(2, 2, occupied=((1, 0), (0, 1)))
        planner = hybrid(map_, obstacle_aware_heuristic=True)
        goal = PathPoint(*map_.cell_to_world((1, 1)), 0.0)
        field = planner._build_goal_distance_field(goal, (0, 1, 0, 1), None)
        self.assertTrue(math.isinf(field[map_.index((0, 0))]))

    def test_distance_field_respects_search_deadline(self):
        planner = hybrid(grid(), obstacle_aware_heuristic=True)
        with self.assertRaises(HybridAStarTimeout):
            planner._build_goal_distance_field(PathPoint(2.5, 2.5, 0),
                                              (0, 11, 0, 11), 0.0)

    def test_default_disabled_and_enabled_path_collision_check(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                planner = hybrid(grid(), obstacle_aware_heuristic=enabled,
                                 planning_timeout_s=2.0)
                path = planner.plan((PathPoint(1.5, 4.5, 0), PathPoint(6.5, 4.5, 0)))
                self.assertTrue(planner.path_is_navigable(path))
                self.assertEqual(bool(planner._goal_distance_field), enabled)


if __name__ == "__main__":
    unittest.main()
