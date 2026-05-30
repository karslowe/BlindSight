"""Occupancy grid mapping: fuse a batched scan of rays into a 2D metric map.

The grid is the fusion layer (see navigation/bridge/AGENT.MD). It is sensor-agnostic: it
consumes rays in the rover frame, whatever sensor produced them (phone depth wedge, side
ToF, ultrasonic). Where each sensor points is the caller's job, not the grid's.

A "scan" is a list of (angle_offset_rad, range_m) tuples:
  - angle_offset_rad: radians relative to the rover heading (pose.theta), CCW positive.
  - range_m: meters to the hit. A range at or beyond max_range_m is the "no return"
    sentinel: free along the ray, with no occupied cell at the end.

Cells follow the schema (see docs/message-schemas.md): -1 unknown, 0 free, 100 occupied;
row-major with row 0 at the bottom; origin is the map-frame coordinate of cell (0,0), the
grid's lower-left corner.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import MapUpdate, Pose, Waypoint  # noqa: E402

# Simple log-odds hit/miss update (POC; not gold-plated). A cell's accumulated evidence is
# thresholded to the schema's -1/0/100 on output. 0 means "never observed".
_L_FREE = -1
_L_OCC = 3
_L_MIN = -12
_L_MAX = 12
_OCC_THRESH = 2  # evidence >= this -> occupied (100)
_FREE_THRESH = -1  # evidence <= this -> free (0); strictly between -> unknown (-1)

_INIT_CELLS = 40  # initial grid is this many cells square, centered on the first pose
_GROW_MARGIN = 4  # extra cells added on each side when the grid must grow


class OccupancyGrid:
    """A 2D occupancy grid built incrementally from scans, growable in the map frame."""

    def __init__(self, resolution_m: float = 0.05, max_range_m: float = 4.0) -> None:
        """Inputs: resolution_m, the edge length of one cell; max_range_m, the sensor max
        range used as the no-return sentinel.
        """
        self.resolution_m = resolution_m
        self.max_range_m = max_range_m
        self.width = 0
        self.height = 0
        self.origin = {"x": 0.0, "y": 0.0}
        self._log: Optional[np.ndarray] = None  # int16, shape (height, width); 0 = unobserved

    # ---- coordinate helpers ----

    def _world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        c = int(math.floor((wx - self.origin["x"]) / self.resolution_m))
        r = int(math.floor((wy - self.origin["y"]) / self.resolution_m))
        return c, r

    def _init_grid(self, pose: Pose) -> None:
        n = _INIT_CELLS
        self._log = np.zeros((n, n), dtype=np.int16)
        self.width = self.height = n
        half = (n // 2) * self.resolution_m
        self.origin = {"x": pose.x - half, "y": pose.y - half}

    def _ensure_bounds(self, points: List[Tuple[float, float]]) -> None:
        """Grow the grid (and shift origin) so every world point fits, with a margin."""
        cells = [self._world_to_cell(x, y) for x, y in points]
        cols = [c for c, _ in cells]
        rows = [r for _, r in cells]
        pad_left = max(0, -min(cols))
        pad_bottom = max(0, -min(rows))
        pad_right = max(0, max(cols) - (self.width - 1))
        pad_top = max(0, max(rows) - (self.height - 1))
        if pad_left or pad_right or pad_bottom or pad_top:
            m = _GROW_MARGIN
            pad_left += m
            pad_right += m
            pad_bottom += m
            pad_top += m
            self._log = np.pad(
                self._log,
                ((pad_bottom, pad_top), (pad_left, pad_right)),
                mode="constant",
                constant_values=0,
            )
            self.height += pad_bottom + pad_top
            self.width += pad_left + pad_right
            # Adding cells below/left moves cell (0,0) down/left in the map frame.
            self.origin["x"] -= pad_left * self.resolution_m
            self.origin["y"] -= pad_bottom * self.resolution_m

    # ---- the update ----

    def update(self, pose: Pose, scan: List[Tuple[float, float]]) -> None:
        """Fuse one batched scan, taken at `pose`, into the grid.

        Inputs:
            pose: the rover pose all rays in this scan are stamped against.
            scan: list of (angle_offset_rad, range_m). angle relative to pose.theta (CCW+);
                  range >= max_range_m is the no-return sentinel (free along the ray only).
        Output: none. Mutates the grid in place, growing it as rays exit current bounds.
        """
        if not scan:
            return
        if self._log is None:
            self._init_grid(pose)

        points: List[Tuple[float, float]] = [(pose.x, pose.y)]
        rays: List[Tuple[float, float, bool]] = []
        for angle_offset, range_m in scan:
            rng = min(range_m, self.max_range_m)
            ang = pose.theta + angle_offset
            ex = pose.x + rng * math.cos(ang)
            ey = pose.y + rng * math.sin(ang)
            points.append((ex, ey))
            occupied = rng < self.max_range_m - 1e-6  # sentinel -> no occupied cell
            rays.append((ex, ey, occupied))

        self._ensure_bounds(points)

        sc, sr = self._world_to_cell(pose.x, pose.y)
        for ex, ey, occupied in rays:
            ec, er = self._world_to_cell(ex, ey)
            self._cast(sc, sr, ec, er, occupied)

    def _cast(self, x0: int, y0: int, x1: int, y1: int, occupied_end: bool) -> None:
        line = _bresenham(x0, y0, x1, y1)
        for c, r in line[:-1]:
            self._bump(c, r, _L_FREE)
        ec, er = line[-1]
        self._bump(ec, er, _L_OCC if occupied_end else _L_FREE)

    def _bump(self, c: int, r: int, delta: int) -> None:
        if 0 <= r < self.height and 0 <= c < self.width:
            v = int(self._log[r, c]) + delta
            self._log[r, c] = max(_L_MIN, min(_L_MAX, v))

    # ---- queries (used by the planner) ----

    def in_bounds(self, c: int, r: int) -> bool:
        return 0 <= c < self.width and 0 <= r < self.height

    def world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        return self._world_to_cell(wx, wy)

    def cell_center(self, c: int, r: int) -> Tuple[float, float]:
        """World coordinate of the center of cell (c, r)."""
        return (
            self.origin["x"] + (c + 0.5) * self.resolution_m,
            self.origin["y"] + (r + 0.5) * self.resolution_m,
        )

    def state_at(self, c: int, r: int) -> int:
        """Cell state: -1 unknown, 0 free, 100 occupied. Out of bounds / unmapped = unknown."""
        if self._log is None or not self.in_bounds(c, r):
            return -1
        v = int(self._log[r, c])
        if v >= _OCC_THRESH:
            return 100
        if v <= _FREE_THRESH:
            return 0
        return -1

    def frontier_cells(self) -> List[Tuple[int, int]]:
        """Cells on the frontier: known-free cells adjacent to unknown space.

        These are the boundary between what is mapped and what is not - the places worth
        driving to in order to reveal more. Returns a list of (c, r). Vectorized with numpy.
        """
        if self._log is None:
            return []
        free = self._log <= _FREE_THRESH
        occ = self._log >= _OCC_THRESH
        unknown = ~(free | occ)
        # A cell has an unknown 4-neighbor if any shifted unknown mask lands on it.
        neigh_unknown = np.zeros_like(unknown)
        neigh_unknown[1:, :] |= unknown[:-1, :]
        neigh_unknown[:-1, :] |= unknown[1:, :]
        neigh_unknown[:, 1:] |= unknown[:, :-1]
        neigh_unknown[:, :-1] |= unknown[:, 1:]
        mask = free & neigh_unknown
        rs, cs = np.where(mask)
        return list(zip(cs.tolist(), rs.tolist()))  # (c, r)

    def blocked_array(self, inflate_cells: int = 0):
        """Boolean (height, width) mask: True where occupied, dilated by inflate_cells.

        Used by the planner to keep a safety margin from walls/obstacles so the rover is
        never routed flush against them. None if nothing is mapped yet.
        """
        if self._log is None:
            return None
        blocked = self._log >= _OCC_THRESH
        for _ in range(max(0, inflate_cells)):
            b = blocked.copy()
            b[1:, :] |= blocked[:-1, :]
            b[:-1, :] |= blocked[1:, :]
            b[:, 1:] |= blocked[:, :-1]
            b[:, :-1] |= blocked[:, 1:]
            blocked = b
        return blocked

    def proximity_cost(self, radius: int = 5):
        """Cost field that is high right next to obstacles and fades to 0 in open space.

        Added to A* step costs so paths prefer the MIDDLE of free corridors instead of
        hugging walls. None if nothing is mapped yet.
        """
        if self._log is None:
            return None
        occupied = self._log >= _OCC_THRESH
        cost = np.zeros(occupied.shape, dtype=np.float32)
        mask = occupied.copy()
        for k in range(radius):
            b = mask.copy()
            b[1:, :] |= mask[:-1, :]
            b[:-1, :] |= mask[1:, :]
            b[:, 1:] |= mask[:, :-1]
            b[:, :-1] |= mask[:, 1:]
            ring = b & ~mask  # cells exactly (k+1) cells from an obstacle
            cost[ring] = float(radius - k)  # closer to a wall -> higher cost
            mask = b
        return cost

    # ---- serialization ----

    def to_map_update(
        self,
        pose: Pose,
        return_path: Optional[list] = None,
        targets: Optional[list] = None,
        start: Optional[dict] = None,
    ) -> MapUpdate:
        """Serialize the current grid into a MapUpdate for the server / viz.

        Flattens the evidence grid to a row-major int list of -1/0/100 (row 0 first, which
        is the bottom row, matching the schema and the viewer). targets carries any detected
        objects of interest; start is the mission home position for the viewer's marker.
        """
        path: List[Waypoint] = return_path or []
        if self._log is None:
            cells: List[int] = []
        else:
            out = np.full(self._log.shape, -1, dtype=np.int16)
            out[self._log <= _FREE_THRESH] = 0
            out[self._log >= _OCC_THRESH] = 100
            cells = out.flatten().astype(int).tolist()
        return MapUpdate(
            width=self.width,
            height=self.height,
            resolution_m=self.resolution_m,
            origin=self.origin,
            cells=cells,
            pose=pose,
            return_path=path,
            targets=targets or [],
            start=start,
        )


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Integer Bresenham line from (x0,y0) to (x1,y1), inclusive of both endpoints.
    Here x = column, y = row."""
    cells: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells
