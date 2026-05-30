"""Occupancy grid mapping: fuse Pose + range readings into a 2D metric map.

Interface stub only. Produces the grid carried by MapUpdate (see message-schemas.md):
row-major cells of -1 unknown, 0 free, 100 occupied.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import MapUpdate, Pose, Waypoint  # noqa: E402


class OccupancyGrid:
    """A 2D occupancy grid built incrementally from poses and range readings."""

    def __init__(self, resolution_m: float = 0.05) -> None:
        """Inputs: resolution_m, the edge length of one cell in meters.

        TODO: allocate the grid (a growable 2D array of -1/0/100), and track its origin
              in the map frame. Grow the grid as the rover explores past its bounds.
        """
        self.resolution_m = resolution_m
        self.width = 0
        self.height = 0
        self.origin = {"x": 0.0, "y": 0.0}
        # TODO: self.cells backing store, e.g. a numpy int8 array initialized to -1.

    def update(self, pose: Pose, range_m: Optional[float]) -> None:
        """Fold one range reading taken at `pose` into the grid.

        Inputs:
            pose: the rover pose when the reading was taken.
            range_m: forward range in meters from the ultrasonic / ToF, or None.
        Output: none. Mutates the grid in place.
        TODO: ray-cast from the pose along its heading; mark cells along the ray free and
              the hit cell occupied (log-odds update). Grow the grid if the ray exits it.
        """
        raise NotImplementedError("occupancy grid update not implemented yet")

    def to_map_update(self, pose: Pose, return_path: Optional[list] = None) -> MapUpdate:
        """Serialize the current grid into a MapUpdate for the server / viz.

        Inputs:
            pose: the rover's current pose to embed.
            return_path: optional list of Waypoint for the planned return route.
        Output: a MapUpdate matching the schema (width, height, resolution_m, origin,
                cells, pose, return_path).
        TODO: flatten the backing store to a row-major int list of -1/0/100.
        """
        path: list[Waypoint] = return_path or []
        return MapUpdate(
            width=self.width,
            height=self.height,
            resolution_m=self.resolution_m,
            origin=self.origin,
            cells=[],  # TODO: flatten the backing store here
            pose=pose,
            return_path=path,
        )
