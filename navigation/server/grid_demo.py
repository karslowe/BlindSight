"""Step 1 demo: drive synthetic scans through the REAL occupancy grid + server.

Watch a room build itself in the browser from a few beams, with no hardware. This is the
first gating win from bridge/AGENT.MD: it exercises the real OccupancyGrid (scan contract,
ray-casting, growth) and the real server/viz pipeline using only synthetic sensor data.

Usage:
    cd navigation
    pip install -r requirements.txt
    python server/grid_demo.py        # then open http://localhost:8000/?live

A simulated rover drives a loop in a virtual room; each tick we synthesize the side ToF
beams + a forward depth wedge by ray-casting against the room, fuse them into the grid, and
publish. The map fills in as the rover passes the walls.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # navigation/ for the mapping import
from schemas.schemas import Pose  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402

MAX_RANGE = 4.0
ROOM_HALF = 1.8  # virtual room: walls at +/- this (meters)
OBSTACLE = (0.6, 1.4, -0.4, 0.4)  # interior block: xmin, xmax, ymin, ymax

# Rover path: a loop inside the room (world meters).
PATH = [(-1.2, -1.0), (1.2, -1.0), (1.2, 1.0), (-1.2, 1.0)]

# The same scan shape the real orchestrator builds: 3 side ToF + a forward depth wedge.
TOF_ANGLES = [-1.22, 1.22, 1.57]
DEPTH_WEDGE = [round(-0.6 + i * 0.15, 3) for i in range(9)]  # ~ +/-35 deg, 9 samples


def _ray_room(x: float, y: float, a: float) -> float:
    """Distance from (x,y) heading a to the room walls (box of half-size ROOM_HALF)."""
    dx, dy = math.cos(a), math.sin(a)
    ts = []
    if dx > 1e-9:
        ts.append((ROOM_HALF - x) / dx)
    elif dx < -1e-9:
        ts.append((-ROOM_HALF - x) / dx)
    if dy > 1e-9:
        ts.append((ROOM_HALF - y) / dy)
    elif dy < -1e-9:
        ts.append((-ROOM_HALF - y) / dy)
    ts = [t for t in ts if t > 0]
    return min(ts) if ts else MAX_RANGE


def _ray_box(x, y, a, xmin, xmax, ymin, ymax):
    """Nearest forward intersection distance of the ray with an axis-aligned box, or None."""
    dx, dy = math.cos(a), math.sin(a)
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((x, dx, xmin, xmax), (y, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
    if tmax < max(tmin, 0.0):
        return None
    t = tmin if tmin > 0 else (tmax if tmax > 0 else None)
    return t


def ray_range(x: float, y: float, a: float) -> float:
    """Synthetic range: nearest of the room walls and the obstacle, capped at MAX_RANGE."""
    r = _ray_room(x, y, a)
    b = _ray_box(x, y, a, *OBSTACLE)
    if b is not None and b < r:
        r = b
    return min(r, MAX_RANGE)


def synthetic_scan(x: float, y: float, theta: float):
    """Build one scan (the side ToF + forward depth wedge) by ray-casting the room."""
    scan = []
    for off in TOF_ANGLES + DEPTH_WEDGE:
        scan.append((off, ray_range(x, y, theta + off)))
    return scan


def pose_along_path(distance: float):
    """Position + heading at `distance` traveled along the looping PATH."""
    segs = []
    total = 0.0
    for i in range(len(PATH)):
        a, b = PATH[i], PATH[(i + 1) % len(PATH)]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        segs.append((a, b, length, total))
        total += length
    d = distance % total
    for a, b, length, start in segs:
        if d <= start + length:
            t = (d - start) / length
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            theta = math.atan2(b[1] - a[1], b[0] - a[0])
            return x, y, theta
    return PATH[0][0], PATH[0][1], 0.0


def main() -> None:
    try:
        from app import MapServer  # when run as: python server/grid_demo.py
    except ImportError:  # pragma: no cover
        from server.app import MapServer
    server = MapServer()
    server.run_in_thread(port=8000)
    grid = OccupancyGrid(resolution_m=0.05, max_range_m=MAX_RANGE)
    print("Serving on http://localhost:8000  (open /?live)")

    distance = 0.0
    speed = 0.4  # m/s
    try:
        while True:
            x, y, theta = pose_along_path(distance)
            pose = Pose(x=x, y=y, theta=theta, timestamp=time.time())
            grid.update(pose, synthetic_scan(x, y, theta))
            server.publish(grid.to_map_update(pose, []))
            distance += speed * 0.1
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
