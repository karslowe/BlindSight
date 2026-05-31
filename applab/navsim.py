"""Closed-loop sim for the Navigator recovery FSM (wall-ahead escape).

Validates the obstacle-recovery fix in edge/python/navigator.py WITHOUT hardware: a synthetic
square room with a wall placed directly ahead of the rover. A simple kinematic model
integrates the DriveCommands the Navigator emits, and a synthetic LiDAR produces the (angle,
range) scan each step. The test asserts the rover does NOT park at the wall: it must reverse
(net backward motion away from the wall at some point) and then move away laterally / change
heading and keep exploring — i.e. its final position is clear of the wall, not pinned to it.

Run (needs numpy + the vendored navigation/ + shared/ on the path):
    python3 applab/navsim.py            # from the repo root, OR
    python3 navsim.py                   # from applab/ next to a python/navigation tree

Exit code 0 = pass.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

# Locate navigator.py + the navigation/shared modules it imports. Try, in order:
#   1) the App Lab app layout: applab/edge/python/{navigator.py, navigation, shared}
#   2) a vendored tree next to this file
#   3) the repo-root navigation/ + shared/ (the source the app vendors)
_HERE = Path(__file__).resolve().parent
_CANDIDATES = [
    _HERE / "edge" / "python",                 # applab/edge/python (navigator + maybe vendored)
    _HERE.parent / "applab" / "edge" / "python",
]
for _c in _CANDIDATES:
    if (_c / "navigator.py").exists():
        sys.path.insert(0, str(_c))
        break
else:
    sys.path.insert(0, str(_HERE / "edge" / "python"))

# navigator.py adds HERE/navigation and HERE/shared to sys.path on import. If those aren't
# vendored locally (they live on the board), fall back to the repo-root source tree.
_repo_root = _HERE.parent
for _p in (_repo_root / "navigation", _repo_root / "shared"):
    if _p.exists():
        sys.path.insert(0, str(_repo_root))          # for `navigation`/`shared` as packages
        sys.path.insert(0, str(_repo_root / "shared"))  # for `schemas.schemas`
        sys.path.insert(0, str(_p.parent / "navigation"))

from navigator import Navigator  # noqa: E402
from schemas.schemas import Pose  # noqa: E402

# ---- synthetic world: square room, with a wall right in front of the start ----------
ROOM_HALF = 2.0          # room is [-2, 2] x [-2, 2] meters
WALL_X = 0.6             # a wall plane at x = 0.6, blocking straight-ahead travel
SENSOR_MAX = 4.0         # matches NO_RETURN_M / DEPTH_MAX_M
SCAN_RAYS = 41
SCAN_HALF_FOV = math.radians(50)
DT = 0.1                 # 10 Hz sim


def _raycast(x: float, y: float, world_ang: float) -> float:
    """Distance from (x,y) along world_ang to the first surface (room walls + the WALL_X
    plane). Returns SENSOR_MAX if nothing within range."""
    dx, dy = math.cos(world_ang), math.sin(world_ang)
    best = SENSOR_MAX
    # room bounds
    for bound, comp, d in ((ROOM_HALF, x, dx), (-ROOM_HALF, x, dx),
                           (ROOM_HALF, y, dy), (-ROOM_HALF, y, dy)):
        if abs(d) > 1e-9:
            t = (bound - comp) / d
            if 0 < t < best:
                hx, hy = x + dx * t, y + dy * t
                if -ROOM_HALF - 1e-6 <= hx <= ROOM_HALF + 1e-6 and \
                   -ROOM_HALF - 1e-6 <= hy <= ROOM_HALF + 1e-6:
                    best = t
    # the obstacle wall at x = WALL_X (a finite-ish vertical segment spanning the room)
    if abs(dx) > 1e-9 and x < WALL_X:
        t = (WALL_X - x) / dx
        if 0 < t < best:
            hy = y + dy * t
            if -ROOM_HALF <= hy <= ROOM_HALF:
                best = t
    return best


def _scan(pose: Pose):
    out = []
    for i in range(SCAN_RAYS):
        ang = -SCAN_HALF_FOV + 2 * SCAN_HALF_FOV * i / (SCAN_RAYS - 1)  # +ang = CCW/left
        rng = _raycast(pose.x, pose.y, pose.theta + ang)
        out.append([round(ang, 5), round(min(rng, SENSOR_MAX), 4)])
    return out


def main() -> int:
    nav = Navigator()
    pose = Pose(x=-0.5, y=0.0, theta=0.0, timestamp=0.0)  # facing +x, toward the wall

    reached_wall = False     # got within blocked distance of the wall at some point
    reversed_away = False    # moved backward (away from the wall, -x) at some point
    escaped = False          # ended clear of the wall AND not facing straight into it
    min_dist_to_wall = 9.9

    t = 0.0
    prev_x = pose.x
    for step in range(400):  # 40 s of sim
        scan = _scan(pose)
        cmd = nav.step(pose, scan)
        if cmd is None:
            cmd_v, cmd_w, cmd_stop = 0.0, 0.0, 1
        else:
            cmd_v, cmd_w, cmd_stop = cmd.linear_velocity, cmd.angular_velocity, cmd.stop

        # integrate kinematics (skip motion on hard stop)
        if not cmd_stop:
            pose = Pose(
                x=pose.x + cmd_v * math.cos(pose.theta) * DT,
                y=pose.y + cmd_v * math.sin(pose.theta) * DT,
                theta=pose.theta + cmd_w * DT,
                timestamp=t,
            )
        # clamp inside the room and don't let it pass through the wall
        pose = Pose(
            x=max(-ROOM_HALF, min(WALL_X - 0.02, pose.x)) if pose.x < WALL_X else pose.x,
            y=max(-ROOM_HALF, min(ROOM_HALF, pose.y)),
            theta=pose.theta, timestamp=t,
        )

        dist_to_wall = WALL_X - pose.x
        min_dist_to_wall = min(min_dist_to_wall, abs(dist_to_wall))
        if abs(dist_to_wall) < 0.45:
            reached_wall = True
        if reached_wall and pose.x < prev_x - 1e-3:  # moved -x (backed away)
            reversed_away = True
        # escaped = backed off the wall and turned away from straight-into-it
        if reached_wall and reversed_away and (WALL_X - pose.x) > 0.5:
            escaped = True

        prev_x = pose.x
        t += DT

    print(f"reached_wall={reached_wall} reversed_away={reversed_away} escaped={escaped} "
          f"final=({pose.x:.2f},{pose.y:.2f},θ={pose.theta:.2f}) min_wall_dist={min_dist_to_wall:.2f}")

    ok = reached_wall and reversed_away and escaped
    if ok:
        print("PASS: rover reached the wall, reversed, and escaped (did not park).")
        return 0
    print("FAIL: rover did not recover from the wall.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
