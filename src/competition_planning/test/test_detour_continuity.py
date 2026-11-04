from dataclasses import replace
import unittest

from competition_planning.local_trajectory_planner import LocalReplanConfig, LocalTrajectoryPlanner
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import PathPoint


class DetourContinuityTests(unittest.TestCase):
    def setup_planner(self, enabled=True):
        grid = OccupancyGridMap(160, 100, 0.05, -1.0, -2.0, (False,) * 16000)
        planner = LocalTrajectoryPlanner(grid, LocalReplanConfig(
            lookahead_distance_m=4.0, inflation_radius_m=0.04,
            retain_safe_detour=enabled, min_turning_radius_m=0.6))
        reference = tuple(PathPoint(i*0.05, 0., 0.) for i in range(101))
        detour = tuple(PathPoint(i*0.05, 0.3, 0.) for i in range(61))
        planner._last_safe_path = detour
        planner._last_safe_rejoin_index = 60
        planner._detour_active = True
        return planner, reference, detour

    def test_clear_reference_does_not_interrupt_valid_detour(self):
        planner, ref, path = self.setup_planner()
        plan = planner.plan(reference_path=ref, current_pose=path[0],
                            dynamic_obstacle_points=())
        self.assertEqual(plan.status, "DETOUR_CONTINUE")
        self.assertEqual(plan.path[0], path[0])
        self.assertEqual(plan.rejoin_index, 60)

    def test_new_obstacle_invalidates_stored_detour(self):
        planner, ref, path = self.setup_planner()
        plan = planner.plan(reference_path=ref, current_pose=path[0],
                            dynamic_obstacle_points=((1.0, 0.3),))
        self.assertNotEqual(plan.status, "DETOUR_CONTINUE")

    def test_disabled_policy_keeps_legacy_behavior(self):
        planner, ref, path = self.setup_planner(False)
        plan = planner.plan(reference_path=ref, current_pose=path[0],
                            dynamic_obstacle_points=())
        self.assertEqual(plan.status, "REFERENCE_CLEAR")

    def test_near_end_releases_detour_for_replanning(self):
        planner, ref, path = self.setup_planner()
        plan = planner.plan(reference_path=ref, current_pose=path[-10],
                            dynamic_obstacle_points=(), previous_reference_index=45)
        self.assertNotEqual(plan.status, "DETOUR_CONTINUE")

    def test_dense_connector_rejects_obstacle_between_sparse_points(self):
        planner, ref, _ = self.setup_planner()
        sparse = (PathPoint(0., 0.3, 0.), PathPoint(0.3, 0.3, 0.),
                  PathPoint(2., 0.3, 0.))
        from competition_planning.local_trajectory_planner import _overlay_obstacles
        grid, _ = _overlay_obstacles(planner._static_map, ((0.15, 0.3),))
        checker = planner._planner(grid, ref)
        self.assertTrue(checker.path_is_navigable(sparse))
        self.assertFalse(planner._validated_detour_tail(sparse, checker))

    def test_heading_mismatch_does_not_reuse_detour(self):
        planner, ref, path = self.setup_planner()
        plan = planner.plan(reference_path=ref,
                            current_pose=replace(path[0], yaw=1.0),
                            dynamic_obstacle_points=())
        self.assertNotEqual(plan.status, "DETOUR_CONTINUE")

    def test_tracking_offset_can_rejoin_ahead_without_nearest_point_snap(self):
        planner, ref, path = self.setup_planner()
        from competition_planning.local_trajectory_planner import _overlay_obstacles
        checker = planner._planner(planner._static_map, ref)
        pose = replace(path[0], y=0.39)
        self.assertFalse(planner._safe_last_path(current_pose=pose,planner=checker))
        planner._config = replace(planner._config, smooth_detour_reconnect=True)
        joined = planner._safe_last_path(current_pose=pose,planner=checker)
        self.assertTrue(joined)
        self.assertEqual(joined[0], pose)
        self.assertEqual(joined[-1], path[-1])
        self.assertTrue(planner.path_is_collision_free(joined, ()))

    def test_reconnect_never_reuses_newly_blocked_tail(self):
        planner, ref, path = self.setup_planner()
        planner._config = replace(planner._config, smooth_detour_reconnect=True)
        from competition_planning.local_trajectory_planner import _overlay_obstacles
        grid, _ = _overlay_obstacles(planner._static_map, ((1.0,.3),))
        checker = planner._planner(grid, ref)
        self.assertFalse(planner._safe_last_path(
            current_pose=replace(path[0],y=.39),planner=checker))

    def test_forward_bridge_does_not_reverse_to_a_goal_behind(self):
        planner, ref, path = self.setup_planner()
        checker = planner._planner(planner._static_map,ref)
        self.assertIsNone(checker.connect_forward(path[10],path[0]))


if __name__ == "__main__":
    unittest.main()
