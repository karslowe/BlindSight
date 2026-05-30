"""Full autonomy demo: the rover explores an unknown room ON ITS OWN, then drives home.

No human driving, no scripted path. The frontier explorer picks where to go, A* plans the
route, the follower drives it, and the map fills in - all from the brain's own decisions.
When the space is covered (or a tick budget is hit), it plans a route home and returns.

This runs the REAL grid + explorer + planner + server with a kinematic rover sim and a
synthetic room sensor model, so you watch genuine autonomy in the browser with no hardware.

Usage:
    cd navigation
    pip install -r requirements.txt
    python server/autonomy_demo.py        # then open http://localhost:8000/?live
"""

from __future__ import annotations

import math
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from schemas.schemas import DriveCommand, Pose, Target  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from planning.explorer import FrontierExplorer  # noqa: E402
from planning.return_planner import ReturnPlanner  # noqa: E402
from server.grid_demo import ROOM_HALF, MAX_RANGE, OBSTACLE, synthetic_scan, _ray_box  # noqa: E402

DT = 0.05


def _in_obstacle(x: float, y: float) -> bool:
    """True if (x, y) is inside the solid interior obstacle (exact box, no margin)."""
    xmin, xmax, ymin, ymax = OBSTACLE
    return xmin <= x <= xmax and ymin <= y <= ymax


def _blocked_cell(grid, x: float, y: float) -> bool:
    """True if (x, y) falls on a cell the MAP marks occupied.

    Collision uses the same occupancy the planner avoids, so a path the planner calls free
    is always actually drivable - no stalling on a phantom boundary near the obstacle.
    """
    occ = grid.blocked_array(0)
    if occ is None:
        return False
    c, r = grid.world_to_cell(x, y)
    return grid.in_bounds(c, r) and bool(occ[r, c])


def _path_blocked(grid, planner) -> bool:
    """True if any waypoint of the current path now sits on an occupied cell.

    The map refines as the rover drives, so a path planned earlier can go stale (a cell it
    routes through becomes a wall). When that happens we re-plan immediately.
    """
    occ = grid.blocked_array(0)
    if occ is None:
        return False
    for wp in planner.current_path():
        c, r = grid.world_to_cell(wp.x, wp.y)
        if grid.in_bounds(c, r) and occ[r, c]:
            return True
    return False
EXPLORE_BUDGET_TICKS = 1500  # force the return after this many ticks (safety / demo cap)

# An imaginary target object in the room. The rover's simulated YOLO "sees" it when it is
# within range, in the forward camera field of view, and not occluded by the obstacle.
TARGET = (1.6, 0.0, "person")  # (x, y, label) - behind the obstacle, so the rover must
#                                explore and round the obstacle before it can see it.
DETECT_RANGE_M = 2.0  # the camera recognizes the target within this distance
# False: explore the WHOLE space, marking the target when seen, then return when done.
# True: head home the moment the target is found (the "search and return" base case).
RETURN_ON_TARGET = True


def _detect_target(x: float, y: float, theta: float):
    """Geometric stand-in for YOLO. Detected when the rover is within range of the target
    and has clear line of sight (not occluded by the obstacle). FoV is intentionally NOT
    required here so the sim reliably "sees" the target as the rover passes near it; the
    real camera has a field of view, handled by detector.detect(frame). Returns the target
    tuple if detected, else None."""
    tx, ty, _label = TARGET
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist > DETECT_RANGE_M:
        return None
    bearing = math.atan2(dy, dx)
    blocked = _ray_box(x, y, bearing, *OBSTACLE)  # occluded by the obstacle?
    if blocked is not None and blocked < dist:
        return None
    return TARGET


def step(state, grid, explorer, planner):
    """Advance the autonomy one tick. Returns the updated (x, y, theta, mode, ticks)."""
    x, y, theta, mode, ticks = state
    pose = Pose(x=x, y=y, theta=theta, timestamp=time.time())

    # Sense + map.
    grid.update(pose, synthetic_scan(x, y, theta))
    if state.start is None:
        state.start = Pose(x=x, y=y, theta=theta, timestamp=0.0)
    state.driven.append(Pose(x=x, y=y, theta=theta, timestamp=0.0))

    # Simulated YOLO: the first time the rover sees the target, MARK it on the map. It
    # keeps exploring the rest of the space and only returns home when exploration is done
    # (set RETURN_ON_TARGET = True to instead head home the moment the target is found).
    if state.target_found is None:
        hit = _detect_target(x, y, theta)
        if hit is not None:
            state.target_found = Target(x=hit[0], y=hit[1], label=hit[2], confidence=0.9)
            if RETURN_ON_TARGET and mode == "explore":
                planner.set_path(planner.plan(grid, state.start, state.driven))
                mode = "return"

    # Decide.
    # The "Return to start" button: cut exploration short and head home now.
    if state.return_requested and mode == "explore" and state.start is not None:
        planner.set_path(planner.plan(grid, state.start, state.driven))
        mode = "return"
        state.return_requested = False

    if state.recovery > 0:
        # Unstick: reverse (and turn a little) for a few ticks, then force a re-plan.
        state.recovery -= 1
        cmd = DriveCommand(-0.14, 0.6, 0)
        if state.recovery == 0:
            planner.set_path([])
    elif mode == "explore":
        need = (not planner.current_path()) or planner.finished() or _path_blocked(grid, planner)
        if need or ticks % 15 == 0:
            path = explorer.next_path(grid, pose)
            if path is None or ticks > EXPLORE_BUDGET_TICKS:
                planner.set_path(planner.plan(grid, state.start, state.driven))
                mode = "return"
            else:
                planner.set_path(path)
        cmd = planner.next_command(pose)
    else:  # return
        # Direct route home; re-plan if it has no path or the route goes stale.
        if (not planner.current_path()) or _path_blocked(grid, planner):
            planner.set_path(planner.plan(grid, state.start, state.driven))
        if math.hypot(x - state.start.x, y - state.start.y) < 0.15:
            cmd = DriveCommand(0.0, 0.0, stop=1)  # arrived home - stop directly at start
        else:
            cmd = planner.next_command(pose)

    # Act (kinematic integration, clamped inside the room and out of the obstacle).
    if cmd is not None and not cmd.stop:
        theta += cmd.angular_velocity * DT
        nx = x + cmd.linear_velocity * math.cos(theta) * DT
        ny = y + cmd.linear_velocity * math.sin(theta) * DT
        cand_x = nx if abs(nx) < ROOM_HALF - 0.05 else x
        cand_y = ny if abs(ny) < ROOM_HALF - 0.05 else y
        if state.recovery > 0:
            x, y = cand_x, cand_y  # recovery moves freely (reverse) to escape a wall
        elif not (_in_obstacle(cand_x, cand_y) or _blocked_cell(grid, cand_x, cand_y)):
            x, y = cand_x, cand_y  # normal driving stops at the obstacle, never enters it

    # Stuck detection: only while actually trying to drive forward. Turning in place (which
    # does not change position) is intentional, not stuck, so it must not trigger recovery.
    if cmd is not None and not cmd.stop and cmd.linear_velocity > 0.05:
        state.recent.append((x, y))
        if state.recovery == 0 and len(state.recent) == state.recent.maxlen:
            if math.hypot(x - state.recent[0][0], y - state.recent[0][1]) < 0.08:
                state.recovery = 12
                state.recent.clear()
                planner.set_path([])  # drop the stale path now; re-plan after recovery
    else:
        state.recent.clear()  # stopped or turning in place - not stuck

    state.x, state.y, state.theta, state.mode, state.ticks = x, y, theta, mode, ticks + 1
    return cmd


class _State:
    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.mode = "explore"
        self.ticks = 0
        self.start = None
        self.driven = []
        self.recent = deque(maxlen=25)  # recent positions, for stuck detection
        self.recovery = 0  # ticks remaining in a back-up-and-turn recovery
        self.return_requested = False  # set by the "Return to start" button
        self.target_found = None  # a Target once the simulated YOLO detects it

    def __iter__(self):
        return iter((self.x, self.y, self.theta, self.mode, self.ticks))


def main() -> None:
    try:
        from app import MapServer  # when run as: python server/autonomy_demo.py
    except ImportError:  # pragma: no cover
        from server.app import MapServer
    grid = OccupancyGrid(resolution_m=0.05, max_range_m=MAX_RANGE)
    explorer = FrontierExplorer()
    planner = ReturnPlanner()
    server = MapServer()
    state = _State()
    # Wire the phone's "Return to start" button to force an early return.
    server.on_return_request = lambda: setattr(state, "return_requested", True)
    server.run_in_thread(port=8000)
    print("Serving on http://localhost:8000  (open /?live)")
    announced = ""
    try:
        while True:
            step(state, grid, explorer, planner)
            if state.mode != announced:
                print(f"[autonomy] mode -> {state.mode}")
                announced = state.mode
            # return_path carries the route home ONLY while returning, so the viewer can
            # tell exploring from returning (and the green "Route home" only shows then).
            route = planner.current_path() if state.mode == "return" else []
            targets = [state.target_found] if state.target_found is not None else []
            server.publish(grid.to_map_update(
                Pose(x=state.x, y=state.y, theta=state.theta, timestamp=time.time()),
                route,
                targets,
            ))
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
