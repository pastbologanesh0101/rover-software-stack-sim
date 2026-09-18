import unittest

from rover_stack.planner import edge_allowed, plan_path
from rover_stack.terrain import Terrain


def flat_terrain(rows, cols, obstacles=None, sand=None):
    elevation = [[0.0] * cols for _ in range(rows)]
    return Terrain(elevation=elevation, obstacles=obstacles or set(), sand=sand or set())


class TestSlopeRejection(unittest.TestCase):
    def test_edge_exceeding_max_slope_is_rejected(self):
        elevation = [
            [0.0, 5.0],
            [0.0, 0.0],
        ]
        terrain = Terrain(elevation=elevation)
        self.assertFalse(edge_allowed(terrain, (0, 0), (0, 1), max_slope=0.6))

    def test_edge_within_slope_limit_is_allowed(self):
        elevation = [
            [0.0, 0.2],
            [0.0, 0.0],
        ]
        terrain = Terrain(elevation=elevation)
        self.assertTrue(edge_allowed(terrain, (0, 0), (0, 1), max_slope=0.6))

    def test_planner_avoids_a_too_steep_direct_route(self):
        # A steep spike directly between start and goal; every edge into or
        # out of it exceeds the max slope, so the planner must detour
        # around it rather than crossing it.
        rows, cols = 3, 3
        elevation = [[0.0] * cols for _ in range(rows)]
        elevation[1][1] = 10.0
        terrain = Terrain(elevation=elevation)
        path = plan_path(terrain, (1, 0), (1, 2), max_slope=0.6)
        self.assertIsNotNone(path)
        self.assertNotIn((1, 1), path)


class TestAStarRouting(unittest.TestCase):
    def test_routes_around_impassable_obstacle(self):
        rows, cols = 5, 5
        # A wall down column 2 with a single gap at row 4.
        obstacles = {(0, 2), (1, 2), (2, 2), (3, 2)}
        terrain = flat_terrain(rows, cols, obstacles=obstacles)
        path = plan_path(terrain, (0, 0), (0, 4))
        self.assertIsNotNone(path)
        for cell in path:
            self.assertNotIn(cell, obstacles)

    def test_prefers_lower_cost_non_sand_route_when_available(self):
        rows, cols = 3, 5
        sand = {(1, 1), (1, 2), (1, 3)}
        terrain = flat_terrain(rows, cols, sand=sand)
        path = plan_path(terrain, (1, 0), (1, 4))
        self.assertIsNotNone(path)
        # A same-length detour via row 0 or row 2 is strictly cheaper than
        # crossing the sand band, so the planner must pick it.
        self.assertTrue(all(cell not in sand for cell in path))

    def test_no_valid_path_reports_failure_not_hang(self):
        rows, cols = 5, 5
        # Goal fully walled off - all 8 surrounding cells are obstacles, so
        # no move (even diagonal) can ever reach it.
        obstacles = {
            (1, 1), (1, 2), (1, 3),
            (2, 1),         (2, 3),
            (3, 1), (3, 2), (3, 3),
        }
        terrain = flat_terrain(rows, cols, obstacles=obstacles)
        path = plan_path(terrain, (0, 0), (2, 2))
        self.assertIsNone(path)

    def test_start_or_goal_inside_blocked_set_reports_failure(self):
        terrain = flat_terrain(4, 4)
        path = plan_path(terrain, (0, 0), (3, 3), blocked={(3, 3)})
        self.assertIsNone(path)


if __name__ == "__main__":
    unittest.main()
