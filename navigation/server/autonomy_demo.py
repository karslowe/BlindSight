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
from schemas.schemas import DriveCommand, Pose  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from planning.explorer import FrontierExplorer  # noqa: E402
from planning.return_planner import ReturnPlanner  # noqa: E402
from server.grid_demo import ROOM_HALF, MAX_RANGE, synthetic_scan  # noqa: E402

DT = 0.05
EXPLORE_BUDGET_TICKS = 1500  # force the return after this many ticks (safety / demo cap)


def step(state, grid, explorer, planner):
    """Advance the autonomy one tick. Returns the updated (x, y, theta, mode, ticks)."""
    x, y, theta, mode, ticks = state
    pose = Pose(x=x, y=y, theta=theta, timestamp=time.time())

    # Sense + map.
    grid.update(pose, synthetic_scan(x, y, theta))
    if state.start is None:
        state.start = Pose(x=x, y=y, theta=theta, timestamp=0.0)
    state.driven.append(Pose(x=x, y=y, theta=theta, timestamp=0.0))

    # Decide.
    if state.recovery > 0:
        # Unstick: back up and turn for a few ticks, then force a re-plan off the obstacle.
        state.recovery -= 1
        cmd = DriveCommand(-0.12, 0.8, 0)
        if state.recovery == 0:
            planner.set_path([])
    elif mode == "explore":
        need = (not planner.current_path()) or planner.finished()
        if need or ticks % 15 == 0:
            path = explorer.next_path(grid, pose)
            if path is None or ticks > EXPLORE_BUDGET_TICKS:
                planner.set_path(planner.plan(grid, state.start, state.driven))
                mode = "return"
            else:
                planner.set_path(path)
        cmd = planner.next_command(pose)
    else:  # return
        if not planner.current_path():
            planner.set_path(planner.plan(grid, state.start, state.driven))
        cmd = planner.next_command(pose)

    # Act (kinematic integration, clamped inside the room).
    if cmd is not None and not cmd.stop:
        theta += cmd.angular_velocity * DT
        nx = x + cmd.linear_velocity * math.cos(theta) * DT
        ny = y + cmd.linear_velocity * math.sin(theta) * DT
        if abs(nx) < ROOM_HALF - 0.05:
            x = nx
        if abs(ny) < ROOM_HALF - 0.05:
            y = ny

    # Stuck detection: if it should be moving but barely has over the window, recover.
    if cmd is not None and cmd.stop:
        state.recent.clear()  # legitimately stopped (arrived) - not stuck
    else:
        state.recent.append((x, y))
        if state.recovery == 0 and len(state.recent) == state.recent.maxlen:
            if math.hypot(x - state.recent[0][0], y - state.recent[0][1]) < 0.08:
                state.recovery = 10
                state.recent.clear()

    state.x, state.y, state.theta, state.mode, state.ticks = x, y, theta, mode, ticks + 1
    return cmd


class _State:
    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.mode = "explore"
        self.ticks = 0
        self.start = None
        self.driven = []
        self.recent = deque(maxlen=40)  # recent positions, for stuck detection
        self.recovery = 0  # ticks remaining in a back-up-and-turn recovery

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
    server.run_in_thread(port=8000)
    print("Serving on http://localhost:8000  (open /?live)")

    state = _State()
    announced = ""
    try:
        while True:
            step(state, grid, explorer, planner)
            if state.mode != announced:
                print(f"[autonomy] mode -> {state.mode}")
                announced = state.mode
            server.publish(grid.to_map_update(
                Pose(x=state.x, y=state.y, theta=state.theta, timestamp=time.time()),
                planner.current_path(),
            ))
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
