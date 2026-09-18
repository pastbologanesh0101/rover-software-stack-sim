"""Terrain model for the simulated rover environment.

Represents rough terrain as a 2D elevation (heightmap) grid, plus overlays
for impassable obstacles ("rocks") and a higher-traversal-cost region (a
"sand pit"). This is a pure software simulation - there is no real rover,
sensor, or physical terrain involved anywhere in this project.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Set, Tuple

Cell = Tuple[int, int]

# Base traversal cost multipliers per terrain type. These scale the A*
# edge cost, so a planner naturally prefers NORMAL terrain over SAND when
# a comparable-length alternative route exists.
NORMAL_COST = 1.0
SAND_COST = 3.0


@dataclass
class Terrain:
    """A rectangular elevation grid with obstacle and sand overlays.

    elevation[row][col] gives the height of that grid cell. obstacles and
    sand are sets of (row, col) cells layered on top of the heightmap.
    """

    elevation: List[List[float]]
    obstacles: Set[Cell] = field(default_factory=set)
    sand: Set[Cell] = field(default_factory=set)
    cell_size: float = 1.0  # meters per grid cell, used for slope/distance

    def __post_init__(self) -> None:
        self.rows = len(self.elevation)
        self.cols = len(self.elevation[0]) if self.rows else 0

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_obstacle(self, cell: Cell) -> bool:
        return cell in self.obstacles

    def is_sand(self, cell: Cell) -> bool:
        return cell in self.sand

    def is_impassable(self, cell: Cell) -> bool:
        """A cell is impassable if it is out of bounds or a rock obstacle."""
        return (not self.in_bounds(cell)) or self.is_obstacle(cell)

    def elevation_at(self, cell: Cell) -> float:
        r, c = cell
        return self.elevation[r][c]

    def horizontal_distance(self, a: Cell, b: Cell) -> float:
        (r1, c1), (r2, c2) = a, b
        return math.hypot(r2 - r1, c2 - c1) * self.cell_size

    def slope(self, a: Cell, b: Cell) -> float:
        """Local grade (rise / run) between two cells, as a positive ratio.

        This is the core hazard-detection primitive: the planner rejects
        any move whose slope exceeds a configured maximum grade.
        """
        run = self.horizontal_distance(a, b)
        if run == 0:
            return 0.0
        rise = abs(self.elevation_at(b) - self.elevation_at(a))
        return rise / run

    def terrain_cost(self, cell: Cell) -> float:
        """Difficulty multiplier for traversing into `cell`."""
        return SAND_COST if self.is_sand(cell) else NORMAL_COST

    def difficulty_at(self, cell: Cell) -> float:
        """Normalized terrain difficulty in [0, 1] for telemetry/risk use."""
        cost = self.terrain_cost(cell)
        span = SAND_COST - NORMAL_COST
        if span <= 0:
            return 0.0
        return min(1.0, max(0.0, (cost - NORMAL_COST) / span))

    def neighbors(self, cell: Cell) -> Iterable[Cell]:
        """8-connected in-bounds neighbors of `cell` (does not filter obstacles)."""
        r, c = cell
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                n = (r + dr, c + dc)
                if self.in_bounds(n):
                    yield n
