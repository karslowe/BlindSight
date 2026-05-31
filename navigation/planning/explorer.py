"""Autonomous frontier exploration: the rover drives itself into the unknown.

A frontier is a known-free cell next to unknown space - the edge of what has been mapped.
The explorer routes the rover to the frontier with the best UTILITY, revealing more of the
space each time, until no reachable frontiers remain (the area is covered). Then the mission
hands off to the return planner to drive home. This replaces human teleoperation.

Strategy: information-gain frontier selection (the standard upgrade over nearest-frontier).
Each frontier cluster is scored by

    utility = INFO_WEIGHT * information_gain  -  travel_cost  +  hysteresis_bonus

where:
  - information_gain ~ how much UNKNOWN space sits around the frontier (unknown cells within
    a radius) plus the cluster's own size. A frontier opening into a big void is worth more
    than one tucked in an explored nook -> the rover goes where it learns the most.
  - travel_cost ~ distance to the frontier (cheap Euclidean for ranking; the real reachable
    path is confirmed with A* on the top candidates).
  - hysteresis_bonus ~ a small reward for staying near the previous target, so the rover does
    not dither back and forth between two equally good frontiers.

A short blacklist suppresses frontiers A* repeatedly cannot reach (e.g. behind a wall), so
the rover stops re-targeting a spot it can never get to and moves on. All of this is
classical and CPU-light - no ML, no training, runs in milliseconds on the edge device.
"""

from __future__ import annotations

import math
import sys
import time
from collections import deque
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import Pose, Waypoint  # noqa: E402

from planning import pathfind  # noqa: E402

# Ignore frontier clusters smaller than this (noise / single-cell gaps), so exploration
# terminates instead of chasing slivers forever.
_MIN_FRONTIER_CELLS = 4
_CLUSTER_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

# ---- utility tuning -----------------------------------------------------------------
# Radius (cells) around a frontier in which to count unknown cells = its information gain.
# At 0.05 m/cell, 10 cells ~ 0.5 m of surrounding space.
_INFO_RADIUS = 10
# How strongly information gain outweighs travel cost. Higher -> chase big unexplored voids
# even if a bit farther; lower -> behave more like nearest-frontier.
_INFO_WEIGHT = 0.6
# Reward (in the same units as utility) for picking the frontier nearest the previous target,
# scaled by how close it is. Stops dithering between two similar frontiers.
_HYSTERESIS_BONUS = 8.0
_HYSTERESIS_RADIUS = 12  # cells; the bonus fades to 0 beyond this from the old target
# Run the real A* reachability check on at most this many top-ranked candidates per call
# (A* is the expensive part; ranking by cheap utility first keeps us fast with many frontiers).
_MAX_ASTAR_CANDIDATES = 6
# A frontier that A* fails to reach is blacklisted for this long (seconds), so we don't keep
# re-picking an unreachable spot every replan.
_BLACKLIST_S = 8.0


class FrontierExplorer:
    """Chooses where to explore next (by information-gain utility) and plans a path to it."""

    def __init__(self) -> None:
        self._last_target: "Optional[Tuple[int, int]]" = None
        # cell -> wall-clock time until which it is blacklisted (unreachable).
        self._blacklist: "dict[Tuple[int, int], float]" = {}

    def next_path(self, grid, pose: Pose) -> Optional[List[Waypoint]]:
        """Plan a route to the highest-utility reachable frontier.

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
        now = time.time()
        self._prune_blacklist(now)
        unknown = grid.unknown_mask()  # (h, w) bool, or None

        # Score every cluster by utility, best first.
        scored: List[Tuple[float, Tuple[int, int]]] = []
        for cl in clusters:
            target = self._medoid(cl)
            if self._is_blacklisted(target, now):
                continue
            util = self._utility(target, cl, rc, unknown)
            scored.append((util, target))
        scored.sort(key=lambda s: s[0], reverse=True)

        # Confirm reachability with real A* on the top candidates (cheap-rank, then verify).
        for _util, target in scored[:_MAX_ASTAR_CANDIDATES]:
            cells = pathfind.plan(grid, rc, target)  # tiered: clearance + known-free first
            if cells:
                self._last_target = target
                return pathfind.to_waypoints(grid, pathfind.simplify(cells))
            self._blacklist[target] = now + _BLACKLIST_S  # unreachable -> suppress briefly

        # None of the top-utility frontiers are reachable; try any remaining candidate as a
        # fallback before declaring exploration complete (so a single bad ranking doesn't end
        # the mission prematurely).
        for _util, target in scored[_MAX_ASTAR_CANDIDATES:]:
            cells = pathfind.plan(grid, rc, target)
            if cells:
                self._last_target = target
                return pathfind.to_waypoints(grid, pathfind.simplify(cells))
        return None

    # ---- utility scoring ----

    def _utility(self, target, cluster, rc, unknown) -> float:
        """utility = INFO_WEIGHT*info_gain - travel_cost + hysteresis_bonus."""
        info = self._information_gain(target, cluster, unknown)
        travel = math.hypot(target[0] - rc[0], target[1] - rc[1])
        bonus = 0.0
        if self._last_target is not None:
            d = math.hypot(target[0] - self._last_target[0], target[1] - self._last_target[1])
            if d < _HYSTERESIS_RADIUS:
                bonus = _HYSTERESIS_BONUS * (1.0 - d / _HYSTERESIS_RADIUS)
        return _INFO_WEIGHT * info - travel + bonus

    @staticmethod
    def _information_gain(target, cluster, unknown) -> float:
        """How much unknown space this frontier would likely reveal.

        Unknown cells within _INFO_RADIUS of the frontier (the void behind it), plus the
        cluster's own boundary size (a longer frontier edge exposes more at once). If the
        grid has no unknown mask yet, fall back to cluster size alone.
        """
        size_term = float(len(cluster))
        if unknown is None:
            return size_term
        h, w = unknown.shape
        c, r = target
        r0, r1 = max(0, r - _INFO_RADIUS), min(h, r + _INFO_RADIUS + 1)
        c0, c1 = max(0, c - _INFO_RADIUS), min(w, c + _INFO_RADIUS + 1)
        unknown_near = float(np.count_nonzero(unknown[r0:r1, c0:c1]))
        return unknown_near + size_term

    # ---- blacklist ----

    def _is_blacklisted(self, cell, now: float) -> bool:
        until = self._blacklist.get(cell)
        return until is not None and until > now

    def _prune_blacklist(self, now: float) -> None:
        expired = [k for k, t in self._blacklist.items() if t <= now]
        for k in expired:
            del self._blacklist[k]

    # ---- frontier clustering ----

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
