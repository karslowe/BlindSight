"""Return planner: route the rover back to its start over the grid it just built.

Interface stub only. Primary strategy is A* over the occupancy grid to the start cell.
Fallback, when the grid is too sparse to plan, is to reverse the logged drive path
(the breadcrumb trail of poses recorded during teleop).
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import DriveCommand, Pose, Waypoint  # noqa: E402


class ReturnPlanner:
    """Plans and then follows the route home."""

    def __init__(self) -> None:
        self._path: List[Waypoint] = []
        self._cursor = 0

    def plan(self, grid, start: Pose, driven_path: List[Pose]) -> List[Waypoint]:
        """Compute a return route from the current pose to the start.

        Inputs:
            grid: the OccupancyGrid built so far.
            start: the Pose recorded at mission start (the goal to return to).
            driven_path: the breadcrumb list of poses recorded during teleop.
        Output:
            an ordered list of Waypoint, from the next point of travel to the goal.
        TODO: run A* over grid.cells from the current cell to the start cell, treating
              100 as blocked and -1 as unknown (configurable: optimistic or pessimistic).
              If A* fails or the grid is too sparse, fall back to reversing driven_path
              into waypoints.
        """
        raise NotImplementedError("return planning not implemented yet")

    def set_path(self, path: List[Waypoint]) -> None:
        """Install a planned path for next_command() to follow. Resets the cursor."""
        self._path = path
        self._cursor = 0

    def current_path(self) -> List[Waypoint]:
        """The installed return path, for embedding in MapUpdate. Empty if none yet."""
        return self._path

    def next_command(self, pose: Pose) -> Optional[DriveCommand]:
        """Produce the next DriveCommand to follow the installed path.

        Inputs: pose, the current rover pose.
        Output: a DriveCommand steering toward the next waypoint, or a stop command once
                the goal is reached, or None if no path is installed.
        TODO: pure-pursuit or carrot-following controller toward self._path[self._cursor];
              advance the cursor as waypoints are reached.
        """
        if not self._path:
            return None
        raise NotImplementedError("path following not implemented yet")
