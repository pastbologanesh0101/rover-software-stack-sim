#!/usr/bin/env python3
"""Demo CLI: runs a full simulated rover mission over hand-authored rough
terrain and prints a telemetry report plus the final mission status.

IMPORTANT: this is a pure software simulation. There is no physical
rover, no real sensors, and no real terrain here - it is inspired by the
operational flavor of NASA's Human Exploration Rover Challenge (mission
telemetry, terrain hazard avoidance, command sequencing) but implemented
entirely as an offline numeric model. Nothing in this repository controls
or communicates with real hardware.

Usage:
    python demo.py            # condensed summary telemetry
    python demo.py --verbose  # full tick-by-tick telemetry log
"""

from __future__ import annotations

import argparse
import math

from rover_stack.mission import MissionPlan, MissionSequencer, Waypoint
from rover_stack.rover import Rover
from rover_stack.terrain import Terrain

ROWS, COLS = 14, 16


def build_sample_terrain() -> Terrain:
    """Author a sample rough-terrain map: two hills, a rock cluster that
    blocks the direct route between waypoints, and a sand pit region."""
    elevation = [[0.0] * COLS for _ in range(ROWS)]
    for r in range(ROWS):
        for c in range(COLS):
            hill_a = 1.5 * math.exp(-(((r - 4) ** 2 + (c - 5) ** 2) / 40.0))
            hill_b = 1.2 * math.exp(-(((r - 9) ** 2 + (c - 11) ** 2) / 40.0))
            ridge = 0.02 * c
            elevation[r][c] = round(hill_a + hill_b + ridge, 3)

    # Hand-placed rock obstacles: two clusters that force the planner to
    # route around them rather than straight through.
    obstacles = {
        (5, 7), (5, 8), (6, 7), (6, 8), (7, 8),
        (8, 3), (9, 3), (10, 3),
    }

    # A sand pit: passable, but slow and slip-prone. Deliberately overlaps
    # the "sand-pit-sample" waypoint below.
    sand = {(r, c) for r in range(2, 7) for c in range(9, 13)}

    return Terrain(elevation=elevation, obstacles=obstacles, sand=sand)


def build_mission() -> MissionPlan:
    return MissionPlan(
        waypoints=[
            Waypoint((3, 2), name="ridge-overlook"),
            Waypoint((6, 12), name="sand-pit-sample", task_ticks=4),
            Waypoint((12, 13), name="far-outcrop", task_ticks=6),
            Waypoint((1, 1), name="return-to-base"),
        ]
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a simulated, HERC-inspired rover mission.")
    parser.add_argument("--verbose", action="store_true", help="print every tick instead of a condensed summary")
    args = parser.parse_args()

    terrain = build_sample_terrain()
    rover = Rover(terrain=terrain, position=(0, 0))
    sequencer = MissionSequencer(terrain, rover)
    plan = build_mission()

    print("=" * 72)
    print("NASA HERC Rover Software Stack - Simulated Mission Demo")
    print("(software simulation only - no real rover or hardware involved)")
    print("=" * 72)
    print(f"Terrain: {ROWS}x{COLS} grid | {len(terrain.obstacles)} rock cells | {len(terrain.sand)} sand cells")
    print(f"Mission plan: {[wp.label() for wp in plan.waypoints]}")
    print("-" * 72)

    report = sequencer.run(plan)

    if args.verbose:
        for rec in rover.log:
            print(rec.summary())
    else:
        log = rover.log
        step = max(1, len(log) // 20)
        shown = log[::step]
        for rec in shown:
            print(rec.summary())
        if log and shown[-1] is not log[-1]:
            print(log[-1].summary())

    print("-" * 72)
    print(report.summary())


if __name__ == "__main__":
    main()
