import unittest

from rover_stack.rover import STUCK_TICKS_TO_TRIGGER, Rover
from rover_stack.terrain import Terrain


def build_terrain(rows, cols, elevation=None, sand=None, obstacles=None):
    elevation = elevation or [[0.0] * cols for _ in range(rows)]
    return Terrain(elevation=elevation, sand=sand or set(), obstacles=obstacles or set())


class TestRoverMovement(unittest.TestCase):
    def test_move_onto_easy_terrain_succeeds(self):
        terrain = build_terrain(3, 3)
        rover = Rover(terrain=terrain, position=(1, 1))
        self.assertTrue(rover.attempt_move((1, 2)))
        self.assertEqual(rover.position, (1, 2))
        self.assertFalse(rover.is_stuck())
        self.assertEqual(rover.tick, 1)

    def test_idle_tick_advances_time_without_moving(self):
        terrain = build_terrain(2, 2)
        rover = Rover(terrain=terrain, position=(0, 0))
        start_battery = rover.battery
        rover.idle_tick(event="task:sample")
        self.assertEqual(rover.position, (0, 0))
        self.assertEqual(rover.tick, 1)
        self.assertLess(rover.battery, start_battery)


class TestWheelSlipDetection(unittest.TestCase):
    def test_high_difficulty_terrain_triggers_wheel_slip(self):
        # A crafted patch: the destination cell is sand AND sits on a
        # noticeable local slope, pushing risk (0.55 sand + 0.25 slope =
        # 0.80) above the 0.75 stuck-risk threshold, while the slope itself
        # (0.25) is still within the planner's max-slope limit.
        elevation = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.25], [0.0, 0.0, 0.0]]
        terrain = build_terrain(3, 3, elevation=elevation, sand={(1, 2)})
        rover = Rover(terrain=terrain, position=(1, 1))

        moved = rover.attempt_move((1, 2))

        self.assertFalse(moved)
        self.assertEqual(rover.position, (1, 1))  # made no progress

    def test_repeated_slip_marks_rover_stuck(self):
        elevation = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.25], [0.0, 0.0, 0.0]]
        terrain = build_terrain(3, 3, elevation=elevation, sand={(1, 2)})
        rover = Rover(terrain=terrain, position=(1, 1))

        for _ in range(STUCK_TICKS_TO_TRIGGER):
            rover.attempt_move((1, 2))

        self.assertTrue(rover.is_stuck())

    def test_easy_terrain_does_not_trigger_wheel_slip(self):
        terrain = build_terrain(3, 3)
        rover = Rover(terrain=terrain, position=(1, 1))
        for _ in range(5):
            rover.attempt_move((1, 2) if rover.position == (1, 1) else (1, 1))
        self.assertFalse(rover.is_stuck())


if __name__ == "__main__":
    unittest.main()
