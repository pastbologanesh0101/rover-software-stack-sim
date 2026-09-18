"""Simulated rover state, telemetry, and low-level movement model.

Everything here is a numeric simulation - there is no physical rover,
motor, or sensor. Movement, wheel-slip risk, and battery drain are simple
deterministic models driven by the Terrain grid, updated once per
simulation "tick".
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

from .terrain import Cell, Terrain

BASE_SPEED = 1.0            # cells/tick on flat, non-sand terrain
SAND_SPEED_FACTOR = 0.4     # speed multiplier while moving onto sand
SLOPE_SPEED_FACTOR = 0.5    # extra slowdown per unit of grade

# Wheel-slip risk model: sand and slope each contribute risk. If the total
# risk for a candidate move meets/exceeds STUCK_RISK_THRESHOLD, the move
# attempt fails (the wheels slip and the rover makes no progress this
# tick). STUCK_TICKS_TO_TRIGGER consecutive failed attempts escalate that
# into a "stuck" event that the mission sequencer must recover from.
SAND_SLIP_RISK = 0.55
SLOPE_SLIP_RISK_SCALE = 1.0
STUCK_RISK_THRESHOLD = 0.75
STUCK_TICKS_TO_TRIGGER = 3

BATTERY_DRAIN_PER_TICK = 0.5
BATTERY_DRAIN_SAND_EXTRA = 0.5
START_BATTERY = 100.0


@dataclass
class TelemetryRecord:
    """One tick's worth of simulated rover telemetry."""

    tick: int
    position: Cell
    heading_deg: float
    speed: float
    battery: float
    terrain_difficulty: float
    slip_risk: float
    stuck: bool
    event: str = ""

    def summary(self) -> str:
        r, c = self.position
        return (
            f"t={self.tick:03d} pos=({r:2d},{c:2d}) hdg={self.heading_deg:6.1f} "
            f"spd={self.speed:4.2f} batt={self.battery:5.1f} "
            f"difficulty={self.terrain_difficulty:.2f} slip={self.slip_risk:.2f} "
            f"stuck={self.stuck!s:<5} {self.event}"
        )


@dataclass
class Rover:
    """Simulated rover state machine over a Terrain grid."""

    terrain: Terrain
    position: Cell
    heading_deg: float = 0.0
    speed: float = 0.0
    battery: float = START_BATTERY
    consecutive_stuck_ticks: int = 0
    tick: int = 0
    log: List[TelemetryRecord] = field(default_factory=list)

    def _heading_to(self, target: Cell) -> float:
        (r1, c1), (r2, c2) = self.position, target
        return math.degrees(math.atan2(r2 - r1, c2 - c1)) % 360.0

    def slip_risk_for(self, next_cell: Cell) -> float:
        """Deterministic wheel-slip risk for moving from `position` onto `next_cell`."""
        risk = 0.0
        if self.terrain.is_sand(next_cell):
            risk += SAND_SLIP_RISK
        risk += self.terrain.slope(self.position, next_cell) * SLOPE_SLIP_RISK_SCALE
        return risk

    def is_stuck(self) -> bool:
        return self.consecutive_stuck_ticks >= STUCK_TICKS_TO_TRIGGER

    def reset_stuck(self) -> None:
        self.consecutive_stuck_ticks = 0

    def idle_tick(self, event: str = "idle") -> TelemetryRecord:
        """Advance one tick without attempting to move (e.g. running a task,
        or holding position during a recovery back-up maneuver)."""
        self.tick += 1
        self.battery = max(0.0, self.battery - BATTERY_DRAIN_PER_TICK)
        self.speed = 0.0
        return self._append_record(risk=0.0, event=event)

    def attempt_move(self, next_cell: Cell) -> bool:
        """Attempt to move from the current position onto `next_cell`.

        Returns True if the move succeeded. Returns False if wheel-slip
        caused the attempt to fail (the rover stays put but a tick still
        elapses and telemetry is still recorded, and the consecutive
        stuck-tick counter increments).
        """
        self.tick += 1
        risk = self.slip_risk_for(next_cell)
        self.heading_deg = self._heading_to(next_cell)

        drain = BATTERY_DRAIN_PER_TICK
        if self.terrain.is_sand(next_cell):
            drain += BATTERY_DRAIN_SAND_EXTRA
        self.battery = max(0.0, self.battery - drain)

        if risk >= STUCK_RISK_THRESHOLD:
            self.consecutive_stuck_ticks += 1
            self.speed = 0.0
            self._append_record(risk=risk, event="wheel-slip")
            return False

        self.consecutive_stuck_ticks = 0
        slope = self.terrain.slope(self.position, next_cell)
        speed = BASE_SPEED
        if self.terrain.is_sand(next_cell):
            speed *= SAND_SPEED_FACTOR
        speed *= max(0.2, 1.0 - slope * SLOPE_SPEED_FACTOR)
        self.speed = speed
        self.position = next_cell

        self._append_record(risk=risk, event="move")
        return True

    def _append_record(self, risk: float, event: str) -> TelemetryRecord:
        rec = TelemetryRecord(
            tick=self.tick,
            position=self.position,
            heading_deg=self.heading_deg,
            speed=self.speed,
            battery=self.battery,
            terrain_difficulty=self.terrain.difficulty_at(self.position),
            slip_risk=risk,
            stuck=self.is_stuck(),
            event=event,
        )
        self.log.append(rec)
        return rec
