import unittest

from competition_planning.local_trajectory_planner import (
    occupied_grid_cell_centers, _overlay_obstacles,
    LocalReplanConfig, LocalTrajectoryPlanner,
)
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import PathPoint


class CostmapConsistencyTests(unittest.TestCase):
    def test_complete_grid_never_discards_occupied_cells(self):
        kwargs = dict(width=100, height=40, resolution_m=.05,
                      origin_x_m=0., origin_y_m=-1., occupancy_threshold=50,
                      x_min_m=0., x_max_m=5., y_half_width_m=1., max_points=2000)
        self.assertEqual(len(occupied_grid_cell_centers([50]*4000, **kwargs)), 2000)
        self.assertEqual(len(occupied_grid_cell_centers(
            [50]*4000, preserve_all=True, **kwargs)), 4000)

    def test_rotated_cell_area_is_not_reduced_to_its_center(self):
        grid = OccupancyGridMap(20, 20, .05, 0., 0., (False,)*400)
        old, _ = _overlay_obstacles(grid, ((.10, .10),))
        full, _ = _overlay_obstacles(grid, ((.10, .10),), cell_size_m=.05)
        for x, y in ((.07,.10), (.13,.10), (.10,.07), (.10,.13)):
            self.assertTrue(full.occupied[full.index(full.world_to_cell(x,y))])
        self.assertFalse(old.occupied[old.index(old.world_to_cell(.07,.10))])

    def test_output_checks_entire_segment_not_only_endpoints(self):
        grid = OccupancyGridMap(100, 40, .05, 0., -1., (False,)*4000)
        planner = LocalTrajectoryPlanner(grid, LocalReplanConfig(
            inflation_radius_m=0., dynamic_obstacle_cell_size_m=.05))
        path = (PathPoint(.1,0.,0.),PathPoint(2.,0.,0.))
        self.assertTrue(planner.path_is_collision_free(path, ()))
        self.assertFalse(planner.path_is_collision_free(path, ((.9,0.),)))


if __name__ == "__main__":
    unittest.main()
