# NASA HERC Rover Software Stack (Simulated)

**This is a pure software simulation. There is no real rover, no real sensors,
and no real terrain anywhere in this repository.** Every "wheel," "hazard
scan," and "battery" below is a number produced by deterministic Python code
over a synthetic heightmap. Nothing here controls, communicates with, or was
tested against physical hardware.

The project is inspired by the operational flavor of **NASA's Human
Exploration Rover Challenge (HERC)** — a student engineering competition where
teams build human-powered rovers and drive them over an obstacle course
designed to mimic rough planetary terrain, while a "mission control" style
team handles telemetry and task sequencing. This repo borrows that framing
(terrain hazards, telemetry, a command/mission sequencer) and reimplements it
as an offline software model: a heightmap terrain, an A* hazard-avoidance
planner, a simulated rover telemetry loop, and a mission sequencer that
executes an ordered list of waypoints and science-sample tasks.

It is a distinct project from other rover/robot simulators in this account —
it is not a warehouse grid multi-robot simulator and not a ROS2 pub/sub +
navigation-stack demo. The angle here is specifically: rough-terrain
elevation modeling, slope-based hazard rejection, and a mission-plan executor
with a stuck/recovery loop.

## What's actually simulated

| Concept | What it means here |
|---|---|
| Terrain | A 2D elevation grid (heightmap) you author in code, with a set of impassable "rock" cells and a higher-cost "sand pit" region |
| Telemetry | A `Rover` object whose position, heading, speed, battery, wheel-slip risk, stuck state, and terrain difficulty are recomputed every simulated tick |
| Hazard detection | Before a path segment is used, its edges are checked against a max-slope threshold and the impassable-cell set (`rover_stack/planner.py::edge_allowed`, `segment_is_safe`) |
| Path planning | A* over the terrain grid, cost-weighted so sand is avoided when a comparable alternative exists, and slope/obstacle violations are forbidden outright |
| Mission sequencing | An ordered list of waypoints (optionally with a science-sample "task" that holds the rover in place for N ticks) executed by `MissionSequencer`, which reports `completed`, `stuck`, or `aborted` |
| Stuck recovery | On repeated wheel-slip, the rover backs up a few cells, marks the hazard cell as temporarily blocked, and replans around it, rather than the mission simply failing |

## Terrain and hazard model

`Terrain` (`rover_stack/terrain.py`) is a rectangular grid of elevation
values plus two overlays:

- **Obstacles** — a set of `(row, col)` cells representing rocks. These are
  always impassable, regardless of cost.
- **Sand** — a set of cells that are passable but carry a `3x` traversal
  cost multiplier (vs. `1x` for normal terrain), slow the rover down, and
  raise wheel-slip risk.

**Slope** between two adjacent cells is `rise / run`: the absolute elevation
difference divided by the horizontal distance between cell centers (`1.0` for
an orthogonal step, `sqrt(2)` for a diagonal step, scaled by `cell_size`).

The planner (`rover_stack/planner.py`) rejects any single-step move
(`edge_allowed`) where:

1. the destination is an obstacle or out of the grid bounds, or
2. the slope between the current cell and the destination exceeds a
   configurable `max_slope` threshold (default `0.6`).

Sand is *not* rejected — it's only penalized in the A* cost function
(`distance * terrain_cost(next_cell)`), so the search naturally prefers flat,
non-sand terrain when a comparable route exists, but will still cross sand if
that's the only (or clearly best) way through.

`scan_lookahead` / `segment_is_safe` model the rover's onboard hazard scan:
before a path segment is "committed to," its edges are checked in a lookahead
window rather than just trusting the plan blindly.

## Wheel-slip / stuck model

Each move attempt (`Rover.attempt_move`) computes a deterministic slip-risk
score for the destination cell:

```
risk = (0.55 if destination is sand else 0.0) + slope(current, destination)
```

If `risk >= 0.75`, the wheels "slip" — the tick still elapses (battery
drains, telemetry is logged) but the rover makes no progress. Three
consecutive slip ticks flip the rover into a `stuck` state.

## Mission sequencer

A `MissionPlan` is an ordered list of `Waypoint`s. Each waypoint has a
`position` and an optional `task_ticks` — if set, reaching that waypoint
triggers a simulated science-sample task that holds the rover in place for
that many ticks before the sequencer advances to the next waypoint.

`MissionSequencer.run(plan)`:

1. Plans a route to each waypoint with A* (a fresh plan per waypoint, so it
   always reflects any temporarily-blocked hazard cells from prior recovery).
2. Steps the rover along that route one cell per tick.
3. On a `stuck` event, triggers **recovery**: mark the offending cell as
   temporarily blocked, back the rover up a couple of cells, and replan
   around the hazard — rather than failing the mission outright.
4. If recovery is exhausted (too many stuck events on one leg) or no route
   exists at all (even before or after blocking a hazard cell), the leg
   fails and the mission reports `"stuck"` or `"aborted"` respectively.
5. Once every waypoint (and its task, if any) is done, the mission reports
   `"completed"`.

## Running the demo

```bash
python demo.py            # condensed telemetry summary
python demo.py --verbose  # full tick-by-tick telemetry log
```

The demo authors a 14x16 sample terrain with two hills, two rock clusters
that force the planner to route around them, and a sand pit, then runs a
4-waypoint mission (including a science-sample task) end to end.

### Example output (condensed)

```
========================================================================
NASA HERC Rover Software Stack - Simulated Mission Demo
(software simulation only - no real rover or hardware involved)
========================================================================
Terrain: 14x16 grid | 8 rock cells | 20 sand cells
Mission plan: ['ridge-overlook', 'sand-pit-sample', 'far-outcrop', 'return-to-base']
------------------------------------------------------------------------
t=001 pos=( 1, 1) hdg=  45.0 spd=0.89 batt= 99.5 difficulty=0.00 slip=0.21 stuck=False move
t=003 pos=( 3, 2) hdg=  45.0 spd=0.89 batt= 98.5 difficulty=0.00 slip=0.22 stuck=False move
...
t=015 pos=( 6,12) hdg= 270.0 spd=0.00 batt= 92.0 difficulty=1.00 slip=0.00 stuck=False task:sand-pit-sample
t=017 pos=( 6,12) hdg= 270.0 spd=0.00 batt= 91.0 difficulty=1.00 slip=0.00 stuck=False task:sand-pit-sample
...
t=042 pos=( 1, 1) hdg= 225.0 spd=0.88 batt= 78.5 difficulty=0.00 slip=0.23 stuck=False move
------------------------------------------------------------------------
Mission status: completed
Waypoints completed: 4
Total ticks: 42
  [t=003] waypoint-reached: ridge-overlook
  [t=018] task-complete: sand-pit-sample (4 ticks)
  [t=018] waypoint-reached: sand-pit-sample
  [t=030] task-complete: far-outcrop (6 ticks)
  [t=030] waypoint-reached: far-outcrop
  [t=042] waypoint-reached: return-to-base
```

## Project layout

```
rover_stack/
  terrain.py   # heightmap grid, slope calc, obstacle/sand overlays
  planner.py   # A* planner, slope/obstacle hazard rejection, lookahead scan
  rover.py     # rover state, telemetry, movement + wheel-slip model
  mission.py   # waypoints/tasks, mission sequencer, stuck recovery
demo.py        # CLI demo: builds sample terrain, runs a full mission
tests/         # unit tests (unittest / pytest-compatible)
```

## Running the tests

Pure standard library — no dependencies required.

```bash
python -m unittest discover -s tests -v
# or, if you have pytest installed:
pytest tests/ -v
```

21 unit tests cover: slope calculation on a known crafted patch, max-slope
segment rejection, A* routing around obstacles and preferring cheaper
non-sand terrain, no-path failure (not a hang), wheel-slip/stuck detection on
a crafted high-difficulty patch, the back-up-and-replan recovery behavior
actually getting the rover unstuck and progressing again, science-sample
tasks consuming their configured tick count, and the mission sequencer
correctly reporting `completed` / `stuck` / `aborted` (including a fully
walled-off, unrecoverable target).

## CI

`.github/workflows/tests.yml` runs the test suite on every push and pull
request against Python 3.11 and 3.12.

## License

MIT — see [LICENSE](LICENSE).
