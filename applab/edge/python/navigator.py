"""Autonomy core: turn each (pose, scan, tof) into a DriveCommand.

Pulled out of main.py so it imports without arduino.app_utils and is unit/sim-testable.
Explore the nearest high-information frontier, re-plan periodically, and on the return
request (UI button or battery failsafe) plan a route home and follow it.

Reactive recovery: plan-and-follow alone parks the rover at a wall. A recovery FSM detects
"blocked ahead" or "no progress", reverses, then turns toward the most-open side and forces a
fresh frontier replan. The turn is AGGRESSIVE-FIRST: it commits to a large angle (~90 deg) and
ESCALATES (up to ~170 deg, and flips side) on repeated consecutive failures, so it doesn't
nibble at a corner — then normal explore steering refines the heading once it's roughly free.

Two front ToF sensors at +/-60 deg (left/right) inform the turn direction (the phone's depth
wedge is narrow and misses the sides) and are folded into the occupancy grid as extra rays.
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

MISSION_BUDGET_S = 45 * 60
FAILSAFE_RETURN_S = int(0.65 * MISSION_BUDGET_S)

# ---- reactive recovery tunables -----------------------------------------------------
FWD_CONE_RAD = 0.26     # rays within +/- this of straight-ahead count as "forward"
FWD_CLEAR_M = 0.45      # forward min-range below this (while wanting forward) = blocked
STUCK_WINDOW_S = 2.0    # window over which to measure progress
STUCK_DIST_M = 0.06     # net displacement under this over the window (while driving) = stuck
REV_V = 0.08            # reverse speed, m/s — GENTLE: the rover is blind behind (no rear sensor)
REVERSE_S = 0.6         # reverse phase duration (~5 cm); only used as a last resort, see below
TURN_W = 1.0            # turn rate during recovery, rad/s (+ = CCW/left); brisk + decisive
# Medium-first, then refine: start with a moderate turn toward the open side and increase it
# in small steps on each consecutive recovery (and flip side if it keeps failing), so it homes
# in on the clear heading instead of overshooting with a big swing.
TURN_BASE_RAD = math.radians(50)
TURN_STEP_RAD = math.radians(18)
TURN_MAX_RAD = math.radians(170)
RECOVERY_RESET_M = 0.30  # if the rover moved this far since the last recovery, it "worked"

# ---- reactive obstacle avoidance (sidestep small obstacles instead of reversing) ----
AVOID_DIST_M = 0.90     # obstacle in the forward cone nearer than this -> steer around it
AVOID_CONE_RAD = 0.40   # +/- this is "ahead" for avoidance (~23 deg)
AVOID_W = 1.0           # max added turn rate when dodging, rad/s
AVOID_SLOW = 0.85       # cut forward speed by up to this at closest range (near-stop + steer,
                        # so it doesn't plow into a wall before the turn takes effect)
SIDE_ARC_RAD = 1.40     # side clearance is measured out to +/- this (~80 deg)
BOX_DIST_M = 0.35       # something this close ahead, AND no open side, = boxed -> reverse
SIDE_OPEN_M = 0.60      # a side counts as "open enough to slip past" if clearer than this

# ---- corridor centering: keep parallel/centred so it reaches the end of a corridor instead
# of drifting into a side wall when heading down it at an angle ----
CORRIDOR_DIST_M = 1.6   # a side ray closer than this = a wall is there -> center against it
CENTER_GAIN = 0.9       # steer-back strength per metre of left/right clearance imbalance
CENTER_SIDE_RAD = 0.30  # rays beyond +/- this graze the side walls (used for centering)

# ---- 360 survey spin: stop and turn a full circle to map the surroundings -----------
SURVEY_W = 1.0              # spin rate during a survey, rad/s
SURVEY_MIN_GAP_M = 1.5      # don't survey again until at least this much new ground is covered.
                            # Surveys fire at DECISION POINTS (the current path is done — a
                            # junction / "another path to consider"), NOT on a fixed interval,
                            # so the rover commits to driving a direction as far as it goes
                            # first; this gap just stops it spinning after a trivially short hop.
SURVEY_SWEEP_RAD = 2 * math.pi * 0.97  # count a survey done at ~360 deg (allow slight under)
SURVEY_MAX_S = 15.0        # hard cap: give up the survey after this long so a rover that
                           # isn't physically rotating can't spin-command forever (it falls
                           # through to exploration instead of being trapped).

# Front ToF sensor angles (rad), +/-60 deg: left = +60, right = -60 (CCW positive).
TOF_LEFT_RAD = math.radians(60)
TOF_RIGHT_RAD = math.radians(-60)
TOF_MAX_M = 4.0          # treat ToF >= this (or None) as "open / no return"
NO_RETURN_M = 4.0        # scan range at/above this is free, not an obstacle


class Navigator:
    """Owns the grid + planners and produces a DriveCommand per (pose, scan, tof)."""

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
        self._recovery_until = 0.0               # wall-clock deadline for the REVERSE phase
        self._turn_sign = 1.0                    # +1 turn left (CCW), -1 turn right
        self._turn_target = TURN_BASE_RAD        # how far to turn this recovery (rad)
        self._turn_start_theta = 0.0             # heading captured when the TURN phase begins
        self._recovery_count = 0                 # consecutive recoveries without progress
        self._last_recovery_pos: "tuple[float, float] | None" = None
        self._last_scan = None                   # most recent scan (incl. ToF rays)
        self._tof = (None, None)                 # latest (left_m, right_m) at +/-60 deg
        self._pose_hist: "deque[tuple[float, float, float]]" = deque()

        # 360 survey state.
        self._surveying = False
        self._survey_accum = 0.0                 # radians swept so far this survey
        self._survey_prev_theta = 0.0
        self._survey_start_t = 0.0               # wall-clock when this survey began (timeout)
        self._dist_since_survey = 0.0            # new ground covered since the last survey
        self._prev_xy: "tuple[float, float] | None" = None

    def step(self, pose: Pose, scan, tof=None) -> "DriveCommand | None":
        """tof = (left_m, right_m) at +/-60 deg, or None. None entries = no reading."""
        if tof is not None:
            self._tof = (tof[0], tof[1])
        # Fold the two front ToF in as extra scan rays so the map gets side coverage the
        # phone's narrow forward wedge misses.
        scan = self._augment_with_tof(scan)
        if scan:
            self.grid.update(pose, scan)
            self._last_scan = scan
        if self.start_pose is None:
            self.start_pose = pose
            self.mission_start = time.time()
            self._begin_survey(pose)   # look around before the first move

        now = time.time()
        self._pose_hist.append((now, pose.x, pose.y))
        while self._pose_hist and now - self._pose_hist[0][0] > STUCK_WINDOW_S:
            self._pose_hist.popleft()

        if (not self.returning and self.mission_start is not None
                and now - self.mission_start >= FAILSAFE_RETURN_S):
            self.request_return()

        # Recovery runs in BOTH explore and return — so the rover never follows a path blindly
        # into a wall (the return path has no idea about obstacles the planner couldn't see).
        if self._recovery is not None:
            return self._run_recovery(now, pose)

        if self.returning:
            cmd = self.planner.next_command(pose)   # follow the route home...
        else:
            self.driven_path.append(pose)
            # Track new ground covered (exploring only) to time periodic surveys.
            if self._prev_xy is not None and not self._surveying:
                self._dist_since_survey += math.hypot(pose.x - self._prev_xy[0],
                                                       pose.y - self._prev_xy[1])
            self._prev_xy = (pose.x, pose.y)
            # 360 survey at DECISION POINTS: when the current path is finished (reached the
            # frontier it was driving to — a junction / end of this path) AND we've covered
            # some ground since the last survey. So it drives a direction as far as the path
            # goes, then spins to check for other paths before choosing the next one.
            at_decision = not self.planner.current_path() or self.planner.finished()
            # Only survey at an OPEN junction — never spin a 360 while blocked ahead (a wall /
            # corridor side/end); there, fall through to explore so recovery turns it out.
            if (not self._surveying and at_decision and not self._blocked_ahead()
                    and self._dist_since_survey >= SURVEY_MIN_GAP_M):
                self._begin_survey(pose)
            if self._surveying:
                return self._run_survey(pose)
            cmd = self._explore(pose)

        # ---- reactive layer, applied to whichever command above (explore OR return) ----
        # Center in a corridor (stay parallel, reach the end), then veer around small
        # obstacles; only reverse/turn when truly boxed or stuck. This is what stops RETURN
        # (and explore) from plowing into walls and parking there.
        cmd = self._corridor_center(cmd)
        cmd = self._avoid(cmd)
        wants_forward = cmd is not None and cmd.stop == 0 and cmd.linear_velocity > 1e-3
        if wants_forward and (self._boxed() or self._stuck(now)):
            return self._begin_recovery(now, pose)
        if cmd is not None and cmd.stop == 1 and self._boxed():
            return self._begin_recovery(now, pose)
        return cmd

    # ---- ToF fusion ----

    def _augment_with_tof(self, scan):
        left, right = self._tof
        extra = []
        if left is not None and left > 0:
            extra.append([TOF_LEFT_RAD, min(float(left), TOF_MAX_M)])
        if right is not None and right > 0:
            extra.append([TOF_RIGHT_RAD, min(float(right), TOF_MAX_M)])
        if not extra:
            return scan
        return (list(scan) if scan else []) + extra

    # ---- exploration ----

    def _explore(self, pose: Pose) -> "DriveCommand | None":
        # Commit to the chosen path: pick a NEW frontier only when the current one is finished
        # (or there is none) — no mid-path re-planning — so the rover keeps heading the same
        # way as far as the path goes instead of dithering between frontiers each tick.
        if not self.planner.current_path() or self.planner.finished():
            path = self.explorer.next_path(self.grid, pose)
            if path is None:
                self.request_return()
                return self.planner.next_command(pose)
            self.planner.set_path(path)
        return self.planner.next_command(pose)

    # ---- reactive obstacle avoidance ----

    def _avoid(self, cmd):
        """Steer a forward command around a near obstacle toward the more-open side.

        If something's in the forward cone closer than AVOID_DIST_M, add a turn bias away from
        it (toward whichever side has more clearance) and trim forward speed — proportional to
        how close it is. The carrot controller still pulls toward the goal, so the net effect
        is the rover curves around a chair/leg and carries on, no stop or reverse."""
        if cmd is None or cmd.stop or cmd.linear_velocity <= 1e-3 or not self._last_scan:
            return cmd
        fwd = [r for a, r in self._last_scan if abs(a) < AVOID_CONE_RAD and 0 < r < NO_RETURN_M]
        if not fwd:
            return cmd
        min_r = min(fwd)
        if min_r >= AVOID_DIST_M:
            return cmd
        # clearance on each side (nearest return in the side arc = worst case)
        left = [r for a, r in self._last_scan if AVOID_CONE_RAD * 0.5 < a < SIDE_ARC_RAD and 0 < r < NO_RETURN_M]
        right = [r for a, r in self._last_scan if -SIDE_ARC_RAD < a < -AVOID_CONE_RAD * 0.5 and 0 < r < NO_RETURN_M]
        lc = min(left) if left else NO_RETURN_M
        rc = min(right) if right else NO_RETURN_M
        side = 1.0 if lc >= rc else -1.0                 # turn toward the more open side
        urgency = max(0.0, 1.0 - min_r / AVOID_DIST_M)   # 0 (far) .. 1 (right on it)
        w = cmd.angular_velocity + side * AVOID_W * urgency
        w = max(-AVOID_W, min(AVOID_W, w))
        v = cmd.linear_velocity * (1.0 - AVOID_SLOW * urgency)
        return DriveCommand(v, w, 0)

    def _corridor_center(self, cmd):
        """In a corridor, steer to balance left vs right wall clearance — keeps the rover
        parallel and centred so it drives to the END instead of veering into a side wall when
        it enters at an angle. Uses the outer FOV rays (they graze the side walls). No-op in
        open space (both sides far) so it doesn't fight the goal-seeker."""
        if cmd is None or cmd.stop or cmd.linear_velocity <= 1e-3 or not self._last_scan:
            return cmd
        left = [r for a, r in self._last_scan if a > CENTER_SIDE_RAD and 0 < r < NO_RETURN_M]
        right = [r for a, r in self._last_scan if a < -CENTER_SIDE_RAD and 0 < r < NO_RETURN_M]
        if not left or not right:
            return cmd
        lc, rc = min(left), min(right)
        if lc > CORRIDOR_DIST_M and rc > CORRIDOR_DIST_M:
            return cmd  # open both sides — not a corridor
        # steer toward the more-open side, away from the nearer wall (+w = left/CCW)
        w = cmd.angular_velocity + CENTER_GAIN * (lc - rc)
        w = max(-AVOID_W, min(AVOID_W, w))
        return DriveCommand(cmd.linear_velocity, w, cmd.stop)

    def _boxed(self) -> bool:
        """True only when something is close ahead AND neither side is open enough to slip
        past — i.e. avoidance can't help and a reverse-and-turn is warranted."""
        if not self._last_scan:
            return False
        fwd = [r for a, r in self._last_scan if abs(a) < FWD_CONE_RAD and 0 < r < NO_RETURN_M]
        if not fwd or min(fwd) >= BOX_DIST_M:
            return False
        left = [r for a, r in self._last_scan if 0.15 < a < SIDE_ARC_RAD and 0 < r < NO_RETURN_M]
        right = [r for a, r in self._last_scan if -SIDE_ARC_RAD < a < -0.15 and 0 < r < NO_RETURN_M]
        left_open = max(left) if left else NO_RETURN_M   # best opening on each side
        right_open = max(right) if right else NO_RETURN_M
        return left_open < SIDE_OPEN_M and right_open < SIDE_OPEN_M

    # ---- recovery FSM ----

    def _blocked_ahead(self) -> bool:
        if not self._last_scan:
            return False
        fwd = [r for a, r in self._last_scan if abs(a) < FWD_CONE_RAD and r < NO_RETURN_M]
        return bool(fwd) and min(fwd) < FWD_CLEAR_M

    def _stuck(self, now: float) -> bool:
        if len(self._pose_hist) < 2:
            return False
        t0, x0, y0 = self._pose_hist[0]
        if now - t0 < STUCK_WINDOW_S * 0.9:
            return False
        return math.hypot(self._pose_hist[-1][1] - x0, self._pose_hist[-1][2] - y0) < STUCK_DIST_M

    def _open_side_sign(self) -> float:
        """+1 (turn left/CCW) if the left side is more open, else -1. Prefer the ToF (they
        point at +/-60 deg, exactly the turn directions); fall back to the phone scan."""
        left, right = self._tof
        if not (left is None and right is None):
            lv = TOF_MAX_M if left is None else float(left)
            rv = TOF_MAX_M if right is None else float(right)
            if abs(lv - rv) > 0.10:           # clear winner -> turn to the more open side
                return 1.0 if lv > rv else -1.0
        if self._last_scan:                    # otherwise use the depth scan's two halves.
            # Compare the NEAREST obstacle on each side (not the max — which ties at the 4 m
            # 'open' sentinel and wrongly defaulted to left). Turn toward the side whose nearest
            # obstacle is farther away — i.e. away from the wall we're stuck against.
            lmin = min([r for a, r in self._last_scan if a > 0] or [NO_RETURN_M])
            rmin = min([r for a, r in self._last_scan if a < 0] or [NO_RETURN_M])
            if abs(lmin - rmin) > 0.10:
                return 1.0 if lmin > rmin else -1.0
        return 1.0

    def _begin_recovery(self, now: float, pose: Pose) -> DriveCommand:
        # Consecutive failures (didn't move far since the last recovery) -> escalate the turn.
        moved = (self._last_recovery_pos is None or
                 math.hypot(pose.x - self._last_recovery_pos[0],
                            pose.y - self._last_recovery_pos[1]) > RECOVERY_RESET_M)
        self._recovery_count = 1 if moved else self._recovery_count + 1
        self._last_recovery_pos = (pose.x, pose.y)
        self._turn_target = min(TURN_BASE_RAD + (self._recovery_count - 1) * TURN_STEP_RAD,
                                TURN_MAX_RAD)
        self._turn_sign = self._open_side_sign()
        if self._recovery_count >= 3:          # chosen side clearly isn't working -> flip
            self._turn_sign = -self._turn_sign
        # SAFETY: no rear collision sensor, so DON'T blindly reverse by default. Turn in place
        # first (no translation) — that escapes most situations using the surveyed surroundings.
        # Only back out (gently, briefly) once we're clearly trapped in a pocket (repeated
        # recoveries without progress), where turning alone can't free the rover.
        if self._recovery_count >= 3:
            self._recovery = "REVERSE"
            self._recovery_until = now + REVERSE_S
            return DriveCommand(-REV_V, 0.0, 0)
        self._recovery = "TURN"
        self._turn_start_theta = pose.theta
        return DriveCommand(0.0, self._turn_sign * TURN_W, 0)

    def _run_recovery(self, now: float, pose: Pose) -> DriveCommand:
        if self._recovery == "REVERSE":
            if now < self._recovery_until:
                return DriveCommand(-REV_V, 0.0, 0)
            self._recovery = "TURN"               # start the (angle-based) turn
            self._turn_start_theta = pose.theta
            return DriveCommand(0.0, self._turn_sign * TURN_W, 0)

        # TURN phase: turn until we've swept _turn_target radians (aggressive, committed).
        if abs(_wrap(pose.theta - self._turn_start_theta)) < self._turn_target:
            return DriveCommand(0.0, self._turn_sign * TURN_W, 0)

        # Done -> exit; reset stuck history. Re-plan from the new heading: a fresh frontier
        # when exploring, or a fresh route home when returning (recovery cleared the old path).
        self._recovery = None
        self._pose_hist.clear()
        if self.returning and self.start_pose is not None:
            self.planner.set_path(self.planner.plan(self.grid, self.start_pose, [pose]))
        else:
            self.planner.set_path([])
        return DriveCommand(0.0, 0.0, 0)

    # ---- 360 survey ----

    def _begin_survey(self, pose: Pose) -> None:
        self._surveying = True
        self._survey_accum = 0.0
        self._survey_prev_theta = pose.theta
        self._survey_start_t = time.time()
        self._dist_since_survey = 0.0

    def _run_survey(self, pose: Pose) -> DriveCommand:
        """Spin in place, accumulating heading until a full turn, then resume exploring.

        Accumulates the (wrapped) per-step heading change, so it works regardless of the
        absolute heading wrapping. The grid keeps fusing scans during the spin, so the rover
        builds a 360 picture of where it is before committing to a direction. Bails out after
        SURVEY_MAX_S so a rover that isn't actually rotating doesn't spin-command forever."""
        self._survey_accum += abs(_wrap(pose.theta - self._survey_prev_theta))
        self._survey_prev_theta = pose.theta
        timed_out = time.time() - self._survey_start_t > SURVEY_MAX_S
        if self._survey_accum < SURVEY_SWEEP_RAD and not timed_out:
            return DriveCommand(0.0, SURVEY_W, 0)
        self._surveying = False
        self.planner.set_path([])   # replan from the freshly-mapped surroundings
        self._pose_hist.clear()     # spinning isn't displacement — don't trip "stuck" next tick
        return DriveCommand(0.0, 0.0, 0)

    # ---- return home ----

    def request_return(self) -> None:
        if self.start_pose is None or self.returning:
            return
        path = self.planner.plan(self.grid, self.start_pose, self.driven_path)
        self.planner.set_path(path)
        self.returning = True
        self._recovery = None
        self._surveying = False


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
