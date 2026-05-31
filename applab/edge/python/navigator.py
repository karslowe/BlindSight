"""Autonomy core: turn each (pose, scan) into a DriveCommand.

Pulled out of main.py so it imports without arduino.app_utils and is unit/sim-testable.
Mirrors the proven sequencing in navigation/main.py Orchestrator: explore the nearest
frontier, re-plan periodically, and on the return request (UI button or battery failsafe)
plan a route home over the just-built map and follow it.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
for _p in (HERE / "navigation", HERE / "shared"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from schemas.schemas import DriveCommand, Pose  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from planning.explorer import FrontierExplorer  # noqa: E402
from planning.return_planner import ReturnPlanner  # noqa: E402

MISSION_BUDGET_S = 45 * 60                          # est. usable runtime (orchestrator value)
FAILSAFE_RETURN_S = int(0.65 * MISSION_BUDGET_S)    # auto-return with margin left


class Navigator:
    """Owns the grid + planners and produces a DriveCommand per (pose, scan)."""

    def __init__(self) -> None:
        self.grid = OccupancyGrid()
        self.explorer = FrontierExplorer()
        self.planner = ReturnPlanner()
        self.driven_path: list[Pose] = []
        self.start_pose: "Pose | None" = None
        self.returning = False
        self.mission_start: "float | None" = None
        self._explore_ticks = 0

    def step(self, pose: Pose, scan) -> "DriveCommand | None":
        if scan:
            self.grid.update(pose, scan)
        if self.start_pose is None:
            self.start_pose = pose
            self.mission_start = time.time()

        if (not self.returning and self.mission_start is not None
                and time.time() - self.mission_start >= FAILSAFE_RETURN_S):
            self.request_return()

        if self.returning:
            return self.planner.next_command(pose)
        self.driven_path.append(pose)
        return self._explore(pose)

    def _explore(self, pose: Pose) -> "DriveCommand | None":
        self._explore_ticks += 1
        need_target = not self.planner.current_path() or self.planner.finished()
        if need_target or self._explore_ticks % 15 == 0:
            path = self.explorer.next_path(self.grid, pose)
            if path is None:
                self.request_return()
                return self.planner.next_command(pose)
            self.planner.set_path(path)
        return self.planner.next_command(pose)

    def request_return(self) -> None:
        if self.start_pose is None or self.returning:
            return
        path = self.planner.plan(self.grid, self.start_pose, self.driven_path)
        self.planner.set_path(path)
        self.returning = True
