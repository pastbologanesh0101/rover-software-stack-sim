import unittest

from rover_stack.terrain import Terrain


class TestTerrainSlope(unittest.TestCase):
    def test_slope_straight_step(self):
        # rise = 2.0 over a 1-cell run -> slope = 2.0
        elevation = [
            [0.0, 2.0],
            [0.0, 0.0],
        ]
        terrain = Terrain(elevation=elevation, cell_size=1.0)
        self.assertAlmostEqual(terrain.slope((0, 0), (0, 1)), 2.0)

    def test_slope_diagonal_step(self):
        # rise = 3.0 over a diagonal run of sqrt(2) -> slope = 3 / sqrt(2)
        elevation = [
            [0.0, 0.0],
            [0.0, 3.0],
        ]
        terrain = Terrain(elevation=elevation, cell_size=1.0)
        expected = 3.0 / (2 ** 0.5)
        self.assertAlmostEqual(terrain.slope((0, 0), (1, 1)), expected, places=6)

    def test_flat_terrain_has_zero_slope(self):
        elevation = [[1.0, 1.0], [1.0, 1.0]]
        terrain = Terrain(elevation=elevation)
        self.assertEqual(terrain.slope((0, 0), (1, 1)), 0.0)

    def test_impassable_marks_obstacles_and_out_of_bounds(self):
        elevation = [[0.0, 0.0], [0.0, 0.0]]
        terrain = Terrain(elevation=elevation, obstacles={(0, 1)})
        self.assertTrue(terrain.is_impassable((0, 1)))     # rock
        self.assertTrue(terrain.is_impassable((5, 5)))      # out of bounds
        self.assertFalse(terrain.is_impassable((0, 0)))     # clear cell

    def test_sand_increases_terrain_cost(self):
        elevation = [[0.0, 0.0]]
        terrain = Terrain(elevation=elevation, sand={(0, 1)})
        self.assertGreater(terrain.terrain_cost((0, 1)), terrain.terrain_cost((0, 0)))


if __name__ == "__main__":
    unittest.main()
