"""Autonomy core: turn each (pose, scan) into a DriveCommand.

Pulled out of main.py so it imports without arduino.app_utils and is unit/sim-testable.
Mirrors the proven sequencing in navigation/main.py Orchestrator: explore the nearest
frontier, re-plan periodically, and on the return request (UI button or battery failsafe)
plan a route home over the just-built map and follow it.

Reactive recovery: plan-and-follow alone parks the rover at a wall (the planner finishes or
re-targets the same unreachable frontier and emits stop / turns in place forever). A small
recovery FSM detects "blocked ahead" or "no progress", then reverses, turns toward the most
open side, and forces a fresh frontier replan so the rover backs out and tries elsewhere.
"""

from __future__ import annotations

import math
import sys
import time
from collections import deque
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

# ---- reactive recovery tunables -----------------------------------------------------
FWD_CONE_RAD = 0.26     # rays within +/- this of straight-ahead count as "forward"
FWD_CLEAR_M = 0.45      # forward min-range below this (while wanting to go forward) = blocked
STUCK_WINDOW_S = 2.0    # window over which to measure progress
STUCK_DIST_M = 0.06     # net displacement under this over the window (while driving) = stuck
REV_V = 0.12            # reverse speed during recovery, m/s
REVERSE_S = 1.0         # reverse phase duration
TURN_W = 0.8            # turn rate during recovery, rad/s (+ = CCW/left)
TURN_S = 1.2            # turn phase duration
# Sentinel: a scan range at/above this is "no return" (free), not an obstacle.
NO_RETURN_M = 4.0


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

        # Recovery state.
        self._recovery: "str | None" = None      # None | "REVERSE" | "TURN"
        self._recovery_until = 0.0               # wall-clock deadline for the current phase
        self._turn_sign = 1.0                    # +1 turn left (CCW), -1 turn right
        self._last_scan = None                   # most recent scan, for trigger checks
        self._pose_hist: "deque[tuple[float, float, float]]" = deque()  # (t, x, y)

    def step(self, pose: Pose, scan) -> "DriveCommand | None":
        if scan:
            self.grid.update(pose, scan)
            self._last_scan = scan
        if self.start_pose is None:
            self.start_pose = pose
            self.mission_start = time.time()

        now = time.time()
        self._pose_hist.append((now, pose.x, pose.y))
        while self._pose_hist and now - self._pose_hist[0][0] > STUCK_WINDOW_S:
            self._pose_hist.popleft()

        if (not self.returning and self.mission_start is not None
                and now - self.mission_start >= FAILSAFE_RETURN_S):
            self.request_return()

        if self.returning:
            # Return-home is a planned route; recovery applies to exploration only.
            return self.planner.next_command(pose)

        self.driven_path.append(pose)

        # Already recovering? run the FSM to completion before anything else.
        if self._recovery is not None:
            return self._run_recovery(now)

        cmd = self._explore(pose)

        # Trigger recovery if the explore command wants forward motion but we're blocked or
        # making no progress. (Turning in place with linear==0 is normal reorientation, not
        # stuck — so both triggers are gated on the command intending forward motion.)
        wants_forward = cmd is not None and cmd.stop == 0 and cmd.linear_velocity > 1e-3
        if wants_forward and (self._blocked_ahead() or self._stuck(now)):
            return self._begin_recovery(now)
        # A plain stop at a wall (finished path on an unreachable frontier) is also "stuck":
        # recover so we don't park there.
        if cmd is not None and cmd.stop == 1 and self._blocked_ahead():
            return self._begin_recovery(now)
        return cmd

    # ---- exploration ----

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

    # ---- recovery FSM ----

    def _blocked_ahead(self) -> bool:
        """True if the nearest return in the forward cone of the latest scan is too close."""
        if not self._last_scan:
            return False
        fwd = [r for a, r in self._last_scan
               if abs(a) < FWD_CONE_RAD and r < NO_RETURN_M]
        return bool(fwd) and min(fwd) < FWD_CLEAR_M

    def _stuck(self, now: float) -> bool:
        """True if, over the last STUCK_WINDOW_S, net displacement is below STUCK_DIST_M.

        Only meaningful once the window is full; otherwise we have too little history.
        """
        if len(self._pose_hist) < 2:
            return False
        t0, x0, y0 = self._pose_hist[0]
        if now - t0 < STUCK_WINDOW_S * 0.9:  # window not yet full enough to judge
            return False
        return math.hypot(self._pose_hist[-1][1] - x0, self._pose_hist[-1][2] - y0) < STUCK_DIST_M

    def _open_side_sign(self) -> float:
        """+1 to turn left (CCW) if the left side has more open space, else -1 (right).

        Left rays are angle>0, right rays angle<0 (matches the +CCW convention). Compares the
        max range on each side; ties / no scan default to left.
        """
        if not self._last_scan:
            return 1.0
        left = [r for a, r in self._last_scan if a > 0]
        right = [r for a, r in self._last_scan if a < 0]
        lmax = max(left) if left else 0.0
        rmax = max(right) if right else 0.0
        return -1.0 if rmax > lmax else 1.0

    def _begin_recovery(self, now: float) -> DriveCommand:
        """Enter recovery: pick the open side, start the REVERSE phase, return its command."""
        self._turn_sign = self._open_side_sign()
        self._recovery = "REVERSE"
        self._recovery_until = now + REVERSE_S
        return DriveCommand(-REV_V, 0.0, 0)

    def _run_recovery(self, now: float) -> DriveCommand:
        """Advance the recovery FSM: REVERSE -> TURN -> exit (force a fresh replan)."""
        if now < self._recovery_until:
            if self._recovery == "REVERSE":
                return DriveCommand(-REV_V, 0.0, 0)
            return DriveCommand(0.0, self._turn_sign * TURN_W, 0)  # TURN

        if self._recovery == "REVERSE":
            self._recovery = "TURN"
            self._recovery_until = now + TURN_S
            return DriveCommand(0.0, self._turn_sign * TURN_W, 0)

        # TURN finished -> exit recovery. Clear the path so _explore picks a fresh frontier
        # from the new pose/heading, and reset progress history so we don't immediately
        # re-trigger "stuck" on stale samples.
        self._recovery = None
        self.planner.set_path([])
        self._pose_hist.clear()
        return DriveCommand(0.0, 0.0, 0)

    # ---- return home ----

    def request_return(self) -> None:
        if self.start_pose is None or self.returning:
            return
        path = self.planner.plan(self.grid, self.start_pose, self.driven_path)
        self.planner.set_path(path)
        self.returning = True
        self._recovery = None  # cancel any in-progress recovery when switching to return
