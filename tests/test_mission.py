import unittest

from rover_stack.mission import MissionPlan, MissionSequencer, Waypoint
from rover_stack.rover import Rover
from rover_stack.terrain import Terrain


class TestMissionSequencer(unittest.TestCase):
    def test_mission_completed_when_all_waypoints_and_tasks_done(self):
        terrain = Terrain(elevation=[[0.0] * 5 for _ in range(5)])
        rover = Rover(terrain=terrain, position=(0, 0))
        sequencer = MissionSequencer(terrain, rover)
        plan = MissionPlan(
            waypoints=[
                Waypoint((2, 2), name="site-a"),
                Waypoint((4, 4), name="site-b", task_ticks=2),
            ]
        )

        report = sequencer.run(plan)

        self.assertEqual(report.status, "completed")
        self.assertEqual(report.waypoints_completed, 2)
        self.assertEqual(rover.position, (4, 4))

    def test_science_task_consumes_configured_ticks_before_advancing(self):
        # Rover starts exactly on the waypoint, so travel takes 0 ticks and
        # every tick consumed is attributable to the task itself.
        terrain = Terrain(elevation=[[0.0] * 3 for _ in range(3)])
        rover = Rover(terrain=terrain, position=(1, 1))
        sequencer = MissionSequencer(terrain, rover)
        plan = MissionPlan(waypoints=[Waypoint((1, 1), name="drill-site", task_ticks=3)])

        report = sequencer.run(plan)

        self.assertEqual(report.status, "completed")
        self.assertEqual(rover.tick, 3)
        task_events = [rec for rec in rover.log if rec.event.startswith("task:")]
        self.assertEqual(len(task_events), 3)

    def test_recovery_backs_up_and_replans_to_get_rover_unstuck(self):
        # A 3-row-wide crossing at column 2: the direct (row-aligned, and
        # therefore cheapest) cell (1, 2) is sand sitting on a local slope
        # bump, which pushes wheel-slip risk over the stuck threshold. The
        # flanking sand cells (0, 2) and (2, 2) are flat, so once the
        # planner is forced to avoid (1, 2) it can still get across.
        rows, cols = 3, 6
        elevation = [[0.0] * cols for _ in range(rows)]
        elevation[1][2] = 0.25
        sand = {(0, 2), (1, 2), (2, 2)}
        terrain = Terrain(elevation=elevation, sand=sand)

        rover = Rover(terrain=terrain, position=(1, 0))
        sequencer = MissionSequencer(terrain, rover)
        plan = MissionPlan(waypoints=[Waypoint((1, 5), name="goal")])

        report = sequencer.run(plan)

        self.assertEqual(report.status, "completed")
        self.assertEqual(rover.position, (1, 5))
        self.assertTrue(any(e.kind == "stuck" for e in report.events))
        self.assertTrue(any(e.kind == "replan" for e in report.events))

    def test_unrecoverable_walled_off_target_reports_stuck_or_aborted(self):
        rows, cols = 5, 5
        # The goal is completely walled off by obstacles on all 8 sides -
        # no route, direct or recovered, can ever reach it.
        obstacles = {
            (1, 1), (1, 2), (1, 3),
            (2, 1),         (2, 3),
            (3, 1), (3, 2), (3, 3),
        }
        elevation = [[0.0] * cols for _ in range(rows)]
        terrain = Terrain(elevation=elevation, obstacles=obstacles)
        rover = Rover(terrain=terrain, position=(0, 0))
        sequencer = MissionSequencer(terrain, rover)
        plan = MissionPlan(waypoints=[Waypoint((2, 2), name="unreachable")])

        report = sequencer.run(plan)

        self.assertIn(report.status, ("stuck", "aborted"))
        self.assertEqual(report.waypoints_completed, 0)
        self.assertNotEqual(rover.position, (2, 2))


if __name__ == "__main__":
    unittest.main()
