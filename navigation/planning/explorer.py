"""Autonomous frontier exploration: the rover drives itself into the unknown.

A frontier is a known-free cell next to unknown space - the edge of what has been mapped.
The explorer repeatedly routes the rover to the nearest sizable frontier, which reveals
more of the space, until no reachable frontiers remain (the area is covered). Then the
mission hands off to the return planner to drive home.

This replaces human teleoperation: nobody drives the rover during exploration.
"""

from __future__ import annotations

import sys
from collections import deque
from pathlib import Path
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import Pose, Waypoint  # noqa: E402

from planning import pathfind  # noqa: E402

# Ignore frontier clusters smaller than this (noise / single-cell gaps), so exploration
# terminates instead of chasing slivers forever.
_MIN_FRONTIER_CELLS = 4
_CLUSTER_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


class FrontierExplorer:
    """Chooses where to explore next and plans a path to it."""

    def next_path(self, grid, pose: Pose) -> Optional[List[Waypoint]]:
        """Plan a route to the nearest reachable frontier.

        Inputs: grid, the map so far; pose, the rover's current pose.
        Output: a list of Waypoint to the chosen frontier, or None when there are no
                reachable sizable frontiers left (exploration is complete).
        """
        frontiers = grid.frontier_cells()
        if not frontiers:
            return None
        clusters = [c for c in self._cluster(frontiers) if len(c) >= _MIN_FRONTIER_CELLS]
        if not clusters:
            return None

        rc = grid.world_to_cell(pose.x, pose.y)
        # Try frontier targets nearest-first; return the first one A* can reach.
        targets = sorted(
            (self._medoid(cl) for cl in clusters),
            key=lambda t: (t[0] - rc[0]) ** 2 + (t[1] - rc[1]) ** 2,
        )
        for target in targets:
            # Prefer a path with obstacle clearance; if wedged, retry without it.
            cells = pathfind.astar(grid, rc, target, inflation=pathfind.INFLATION_CELLS)
            if not cells:
                cells = pathfind.astar(grid, rc, target, inflation=0)
            if cells:
                return pathfind.to_waypoints(grid, pathfind.simplify(cells))
        return None

    @staticmethod
    def _cluster(cells: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        """Group frontier cells into connected components (8-connected)."""
        cellset = set(cells)
        seen = set()
        clusters = []
        for cell in cells:
            if cell in seen:
                continue
            comp = []
            dq = deque([cell])
            seen.add(cell)
            while dq:
                cur = dq.popleft()
                comp.append(cur)
                for dc, dr in _CLUSTER_NEIGHBORS:
                    nb = (cur[0] + dc, cur[1] + dr)
                    if nb in cellset and nb not in seen:
                        seen.add(nb)
                        dq.append(nb)
            clusters.append(comp)
        return clusters

    @staticmethod
    def _medoid(cluster: List[Tuple[int, int]]) -> Tuple[int, int]:
        """The frontier cell nearest the cluster centroid (guaranteed free and reachable)."""
        cx = sum(c for c, _ in cluster) / len(cluster)
        cy = sum(r for _, r in cluster) / len(cluster)
        return min(cluster, key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)
