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

# Default obstacle inflation (cells). At 0.05 m/cell this keeps ~0.1 m of clearance so the
# rover is never routed flush against a wall/obstacle (the cause of getting wedged).
INFLATION_CELLS = 2


def astar(
    grid,
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    unknown_cost: float = 4.0,
    inflation: int = 0,
    block_unknown: bool = True,
) -> Optional[List[Tuple[int, int]]]:
    """A* from start_cell to goal_cell over the grid. Returns cells start->goal, or None.

    Blocks occupied cells (and, if inflation > 0, cells within that many of an obstacle).
    With block_unknown=True (default) it also refuses to route through unobserved cells, so
    the rover stays on confirmed-free ground and never plans through walls or unmapped/
    occluded space. With block_unknown=False, unknown cells are allowed but cost extra
    (used only as a last-resort fallback). The goal cell is always allowed as a target.
    """
    if grid.width == 0:
        return None
    sc, gc = tuple(start_cell), tuple(goal_cell)
    if grid.state_at(*sc) == 100 or grid.state_at(*gc) == 100:
        return None  # an endpoint is inside an obstacle

    blocked = grid.blocked_array(inflation) if inflation > 0 else None

    def is_blocked(c: int, r: int) -> bool:
        if blocked is not None and grid.in_bounds(c, r) and blocked[r, c]:
            return True
        return grid.state_at(c, r) == 100

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
            if nb != gc:
                if is_blocked(*nb):
                    continue
                if block_unknown and state == -1:
                    continue  # stay on confirmed-free ground
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


def plan(grid, start_cell: Tuple[int, int], goal_cell: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """Tiered A*, safest-first. Returns cells start->goal, or None.

    1) clearance + known-free only (the normal case - keeps the rover safe),
    2) known-free only (no clearance, for tight spots),
    3) allow unknown (last resort, only if the rover is otherwise cut off).
    """
    for inflation, block_unknown in ((INFLATION_CELLS, True), (0, True), (0, False)):
        cells = astar(grid, start_cell, goal_cell, inflation=inflation, block_unknown=block_unknown)
        if cells:
            return cells
    return None


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
