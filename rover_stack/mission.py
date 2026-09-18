"""Mission sequencer: executes an ordered mission plan of waypoints and
optional science-sample tasks over the simulated terrain.

The sequencer uses the hazard-aware A* planner to route between
waypoints. If the rover gets stuck (repeated wheel-slip on a path
segment), it triggers a recovery behavior: back up a few steps, mark the
offending cell as temporarily blocked, and replan around it. Only after
recovery is exhausted (or no route exists at all) does the mission report
failure, rather than failing on the first hazard.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Set

from .planner import DEFAULT_MAX_SLOPE, plan_path
from .rover import Rover
from .terrain import Cell, Terrain

RECOVERY_BACKUP_STEPS = 2
MAX_RECOVERIES_PER_WAYPOINT = 4
MAX_TICKS_PER_WAYPOINT = 500


@dataclass
class Waypoint:
    """A single stop in a mission plan.

    If `task_ticks` > 0, reaching this waypoint triggers a simulated
    science-sample task that holds the rover in place for that many ticks
    before the sequencer moves on to the next waypoint.
    """

    position: Cell
    task_ticks: int = 0
    name: str = ""

    def label(self) -> str:
        return self.name or str(self.position)


@dataclass
class MissionPlan:
    waypoints: List[Waypoint]


@dataclass
class MissionEvent:
    tick: int
    kind: str
    detail: str

    def __str__(self) -> str:
        return f"[t={self.tick:03d}] {self.kind}: {self.detail}"


@dataclass
class MissionReport:
    status: str = "in-progress"  # "completed" | "stuck" | "aborted"
    events: List[MissionEvent] = field(default_factory=list)
    waypoints_completed: int = 0
    total_ticks: int = 0

    def summary(self) -> str:
        lines = [
            f"Mission status: {self.status}",
            f"Waypoints completed: {self.waypoints_completed}",
            f"Total ticks: {self.total_ticks}",
        ]
        lines.extend(f"  {e}" for e in self.events)
        return "\n".join(lines)


class MissionSequencer:
    """Executes a MissionPlan against a Rover over a Terrain."""

    def __init__(self, terrain: Terrain, rover: Rover, max_slope: float = DEFAULT_MAX_SLOPE):
        self.terrain = terrain
        self.rover = rover
        self.max_slope = max_slope
        self.permanently_blocked: Set[Cell] = set()

    def run(self, plan: MissionPlan) -> MissionReport:
        report = MissionReport(status="in-progress")
        for wp in plan.waypoints:
            ok = self._travel_to(wp.position, report)
            if not ok:
                report.total_ticks = self.rover.tick
                return report

            if wp.task_ticks > 0:
                for _ in range(wp.task_ticks):
                    self.rover.idle_tick(event=f"task:{wp.label()}")
                report.events.append(
                    MissionEvent(self.rover.tick, "task-complete", f"{wp.label()} ({wp.task_ticks} ticks)")
                )

            report.waypoints_completed += 1
            report.events.append(MissionEvent(self.rover.tick, "waypoint-reached", wp.label()))

        report.status = "completed"
        report.total_ticks = self.rover.tick
        return report

    def _travel_to(self, goal: Cell, report: MissionReport) -> bool:
        """Drive the rover to `goal`, recovering from stuck events along the way.

        Returns True on success. On failure, sets report.status to either
        "aborted" (no route exists at all) or "stuck" (recovery exhausted).
        """
        blocked = set(self.permanently_blocked)
        path = plan_path(self.terrain, self.rover.position, goal, self.max_slope, blocked=blocked)
        if path is None:
            report.status = "aborted"
            report.events.append(MissionEvent(self.rover.tick, "no-path", f"no valid route to {goal}"))
            return False

        recoveries = 0
        idx = 1  # path[0] is the rover's current position
        visited_history: List[Cell] = [self.rover.position]

        while self.rover.position != goal:
            if self.rover.tick > MAX_TICKS_PER_WAYPOINT:
                report.status = "stuck"
                report.events.append(MissionEvent(self.rover.tick, "timeout", f"exceeded tick budget en route to {goal}"))
                return False

            if idx >= len(path):
                path = plan_path(self.terrain, self.rover.position, goal, self.max_slope, blocked=blocked)
                if path is None:
                    report.status = "aborted"
                    report.events.append(MissionEvent(self.rover.tick, "no-path", f"no valid route to {goal}"))
                    return False
                idx = 1
                continue

            next_cell = path[idx]
            moved = self.rover.attempt_move(next_cell)
            if moved:
                visited_history.append(next_cell)
                idx += 1
                continue

            if not self.rover.is_stuck():
                # A single wheel-slip isn't yet a "stuck" event - retry the
                # same segment next tick (the rover may creep forward on a
                # later attempt in a less deterministic model; here it will
                # keep failing deterministically until the stuck threshold
                # triggers recovery below).
                continue

            recoveries += 1
            report.events.append(
                MissionEvent(self.rover.tick, "stuck", f"wheel-slip at {next_cell}, recovery attempt #{recoveries}")
            )
            if recoveries > MAX_RECOVERIES_PER_WAYPOINT:
                report.status = "stuck"
                report.events.append(MissionEvent(self.rover.tick, "abandon", f"could not recover en route to {goal}"))
                return False

            # Recovery behavior: mark the hazard cell as temporarily
            # blocked, back the rover up a few steps, and replan.
            blocked.add(next_cell)
            steps_back = min(RECOVERY_BACKUP_STEPS, len(visited_history) - 1)
            for _ in range(steps_back):
                visited_history.pop()
                self.rover.position = visited_history[-1]
                self.rover.idle_tick(event="recovery-backup")
            self.rover.reset_stuck()

            path = plan_path(self.terrain, self.rover.position, goal, self.max_slope, blocked=blocked)
            if path is None:
                report.status = "aborted"
                report.events.append(MissionEvent(self.rover.tick, "no-path", f"no route around hazard to {goal}"))
                return False
            idx = 1
            report.events.append(MissionEvent(self.rover.tick, "replan", f"new route around {next_cell}"))

        return True
