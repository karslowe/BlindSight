"""Return planner: route the rover back to its start over the grid it just built.

This is the core of the exploratory mission: the route home is NOT known in advance. The
rover drives into an unknown space building a map, and only when "return" is requested does
the planner compute a path across that just-built map.

Primary strategy: A* over the occupancy grid from the current cell to the start cell,
treating occupied cells as blocked and penalizing unknown cells (so it prefers explored,
known-free routes but can cut through unmapped gaps if it must). Fallback, when the grid is
too sparse or A* finds nothing: reverse the logged breadcrumb trail of poses.

next_command() then follows the planned waypoints with a simple carrot-follow controller,
emitting DriveCommands until the goal is reached.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import DriveCommand, Pose, Waypoint  # noqa: E402

from planning import pathfind  # noqa: E402

# Planner cost: extra cost to traverse an unobserved cell (prefers known-free routes).
_UNKNOWN_COST = 4.0
# Controller gains/limits.
_REACH_RADIUS_M = 0.12  # a waypoint counts as reached within this distance
_MAX_V = 0.22  # m/s, forward speed
_MAX_W = 1.0  # rad/s, turn rate
_K_ANG = 1.6  # proportional steering gain
# Breadcrumb downsample spacing for the fallback path.
_BREADCRUMB_SPACING_M = 0.15


def _wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class ReturnPlanner:
    """Plans the route home over the explored grid, then follows it."""

    def __init__(self) -> None:
        self._path: List[Waypoint] = []
        self._cursor = 0

    # ---- planning ----

    def plan(self, grid, start: Pose, driven_path: List[Pose]) -> List[Waypoint]:
        """Compute a return route from the current position to the start.

        Inputs:
            grid: the OccupancyGrid built so far.
            start: the Pose recorded at mission start (the goal to return to).
            driven_path: breadcrumb poses recorded during teleop; the last is "now".
        Output:
            an ordered list of Waypoint from the current position to the goal (home).
        """
        current = driven_path[-1] if driven_path else start
        route = self._astar(grid, current, start)
        if route:
            return route
        # Fallback: retrace the breadcrumb trail in reverse back to the start.
        return self._reverse_breadcrumbs(driven_path)

    def _astar(self, grid, start_pose: Pose, goal_pose: Pose) -> Optional[List[Waypoint]]:
        """A* over the grid from start_pose's cell to goal_pose's cell. None if no path."""
        sc = grid.world_to_cell(start_pose.x, start_pose.y)
        gc = grid.world_to_cell(goal_pose.x, goal_pose.y)
        cells = pathfind.plan(grid, sc, gc)  # tiered: clearance + known-free first
        if not cells:
            return None
        return pathfind.to_waypoints(grid, pathfind.simplify(cells))

    def _reverse_breadcrumbs(self, driven_path: List[Pose]) -> List[Waypoint]:
        """Retrace the driven poses in reverse (current -> start), downsampled."""
        out: List[Waypoint] = []
        last: Optional[Pose] = None
        for p in reversed(driven_path):
            if last is None or math.hypot(p.x - last.x, p.y - last.y) >= _BREADCRUMB_SPACING_M:
                out.append(Waypoint(p.x, p.y))
                last = p
        if driven_path:
            home = driven_path[0]
            if not out or (out[-1].x, out[-1].y) != (home.x, home.y):
                out.append(Waypoint(home.x, home.y))
        return out

    # ---- following ----

    def set_path(self, path: List[Waypoint]) -> None:
        """Install a planned path for next_command() to follow. Resets the cursor."""
        self._path = path
        self._cursor = 0

    def current_path(self) -> List[Waypoint]:
        """The installed path, for embedding in MapUpdate. Empty if none yet."""
        return self._path

    def finished(self) -> bool:
        """True once every waypoint of the installed path has been reached."""
        return self._cursor >= len(self._path)

    def next_command(self, pose: Pose) -> Optional[DriveCommand]:
        """Produce the next DriveCommand to follow the installed path.

        Carrot-follow: advance past waypoints already reached, steer toward the next one,
        and slow the forward speed when it has to turn hard. Returns a stop command once the
        goal is reached, or None if no path is installed.
        """
        if not self._path:
            return None
        # Skip waypoints we are already close to.
        while self._cursor < len(self._path):
            wp = self._path[self._cursor]
            if math.hypot(wp.x - pose.x, wp.y - pose.y) < _REACH_RADIUS_M:
                self._cursor += 1
            else:
                break
        if self._cursor >= len(self._path):
            return DriveCommand(0.0, 0.0, stop=1)  # arrived home

        target = self._path[self._cursor]
        desired = math.atan2(target.y - pose.y, target.x - pose.x)
        err = _wrap(desired - pose.theta)
        angular = max(-_MAX_W, min(_MAX_W, _K_ANG * err))
        # Drive forward only when roughly facing the target; turn in place otherwise.
        linear = _MAX_V * max(0.0, math.cos(err))
        return DriveCommand(linear, angular, 0)
