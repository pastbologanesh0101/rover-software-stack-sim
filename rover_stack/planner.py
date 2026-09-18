"""A* path planner over the Terrain grid.

Before a path segment is committed to, the rover "scans" a lookahead
window of upcoming terrain (`scan_lookahead` / `segment_is_safe`) and the
planner rejects any edge whose destination is impassable (a rock, or out
of bounds) or whose local slope exceeds a maximum grade. Difficult but
passable terrain (sand) is not forbidden - it is only penalized via its
higher traversal cost, so the search naturally prefers easier terrain
when a comparable alternative route exists.
"""

from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Set, Tuple

from .terrain import Cell, Terrain

DEFAULT_MAX_SLOPE = 0.6  # maximum allowed grade (rise/run) for a single move


def edge_allowed(terrain: Terrain, a: Cell, b: Cell, max_slope: float = DEFAULT_MAX_SLOPE) -> bool:
    """Return True if moving from `a` directly to `b` is a legal path segment.

    Rejects the move if the destination is impassable (rock/out-of-bounds)
    or if the local slope between the two cells exceeds `max_slope`.
    """
    if terrain.is_impassable(b):
        return False
    if terrain.slope(a, b) > max_slope:
        return False
    return True


def scan_lookahead(path: List[Cell], from_index: int, window: int) -> List[Cell]:
    """Return the next `window` cells of `path` starting at `from_index`.

    Models the rover's onboard hazard-scan sensor sweeping the terrain
    immediately ahead before the rover commits to that path segment.
    """
    return path[from_index: from_index + window]


def segment_is_safe(terrain: Terrain, segment: List[Cell], max_slope: float = DEFAULT_MAX_SLOPE) -> bool:
    """Check every edge in a lookahead segment against the hazard rules."""
    for i in range(len(segment) - 1):
        if not edge_allowed(terrain, segment[i], segment[i + 1], max_slope):
            return False
    return True


def _heuristic(terrain: Terrain, a: Cell, b: Cell) -> float:
    # Euclidean distance scaled by the minimum possible per-cell cost
    # (NORMAL_COST == 1.0), keeping the heuristic admissible.
    return terrain.horizontal_distance(a, b)


def plan_path(
    terrain: Terrain,
    start: Cell,
    goal: Cell,
    max_slope: float = DEFAULT_MAX_SLOPE,
    blocked: Optional[Set[Cell]] = None,
) -> Optional[List[Cell]]:
    """A* search from `start` to `goal` over the terrain grid.

    `blocked` is an extra set of cells to treat as temporarily impassable,
    on top of the terrain's own rock obstacles - used by the mission
    recovery behavior to route around a cell that caused the rover to get
    stuck, without permanently mutating the terrain model.

    Returns the path as a list of cells (including start and goal), or
    None if no valid path exists. The search space is a finite grid with a
    closed set, so this always terminates - it cannot hang even when no
    path exists.
    """
    blocked = blocked or set()
    if terrain.is_impassable(start) or start in blocked:
        return None
    if terrain.is_impassable(goal) or goal in blocked:
        return None
    if start == goal:
        return [start]

    open_heap: List[Tuple[float, Cell]] = [(0.0, start)]
    g_score: Dict[Cell, float] = {start: 0.0}
    came_from: Dict[Cell, Cell] = {}
    closed: Set[Cell] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct(came_from, current)
        closed.add(current)

        for nxt in terrain.neighbors(current):
            if nxt in closed or nxt in blocked:
                continue
            if not edge_allowed(terrain, current, nxt, max_slope):
                continue
            step_cost = terrain.horizontal_distance(current, nxt) * terrain.terrain_cost(nxt)
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(nxt, float("inf")):
                g_score[nxt] = tentative_g
                came_from[nxt] = current
                f_score = tentative_g + _heuristic(terrain, nxt, goal)
                heapq.heappush(open_heap, (f_score, nxt))

    return None


def _reconstruct(came_from: Dict[Cell, Cell], current: Cell) -> List[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
