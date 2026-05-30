"""Shared A* pathfinding over the occupancy grid.

Used by both the return planner (route home) and the frontier explorer (route to the next
unmapped area). Occupied cells are blocked; unknown cells are allowed but penalized, so
paths prefer explored, known-free routes while still being able to cut through gaps.
"""

from __future__ import annotations

import heapq
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import Waypoint  # noqa: E402

_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def astar(
    grid,
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    unknown_cost: float = 4.0,
) -> Optional[List[Tuple[int, int]]]:
    """A* from start_cell to goal_cell over the grid. Returns cells start->goal, or None.

    Blocks occupied cells; adds unknown_cost to stepping into an unobserved cell.
    """
    if grid.width == 0:
        return None
    sc, gc = tuple(start_cell), tuple(goal_cell)
    if grid.state_at(*sc) == 100 or grid.state_at(*gc) == 100:
        return None  # an endpoint is inside an obstacle

    def heuristic(c: Tuple[int, int]) -> float:
        return math.hypot(c[0] - gc[0], c[1] - gc[1])

    open_heap: List[Tuple[float, Tuple[int, int]]] = [(heuristic(sc), sc)]
    came: dict = {}
    gscore = {sc: 0.0}
    found = False
    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == gc:
            found = True
            break
        for dc, dr in _NEIGHBORS:
            nb = (cur[0] + dc, cur[1] + dr)
            if not grid.in_bounds(*nb):
                continue
            state = grid.state_at(*nb)
            if state == 100:
                continue
            step = math.hypot(dc, dr)
            if state == -1:
                step += unknown_cost
            tentative = gscore[cur] + step
            if tentative < gscore.get(nb, math.inf):
                came[nb] = cur
                gscore[nb] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(nb), nb))
    if not found:
        return None

    cells = [gc]
    c = gc
    while c in came:
        c = came[c]
        cells.append(c)
    cells.reverse()
    return cells


def simplify(cells: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """Drop collinear interior cells so a path is a few waypoints, not every cell."""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    for i in range(1, len(cells) - 1):
        ax, ay = out[-1]
        bx, by = cells[i]
        cx, cy = cells[i + 1]
        cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        if cross != 0:
            out.append(cells[i])
    out.append(cells[-1])
    return out


def to_waypoints(grid, cells: List[Tuple[int, int]]) -> List[Waypoint]:
    """Convert grid cells to world-frame Waypoints (cell centers)."""
    return [Waypoint(*grid.cell_center(c, r)) for c, r in cells]
