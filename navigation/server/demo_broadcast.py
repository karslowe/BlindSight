"""Run the REAL server with synthetic MapUpdates, end to end.

This proves the broadcast path: the actual FastAPI server serves the real visualization
and pushes real MapUpdate frames over the websocket, with no rover, no SLAM, no car. It is
the server-side mirror of the browser's dev fake feed, and it is how you verify the live
map flows through the genuine pipeline before any hardware exists.

Usage:
    cd navigation
    pip install -r requirements.txt
    python server/demo_broadcast.py
    # then open http://localhost:8000/?live  (the ?live forces the real socket)

Press the "Return to start" button in the browser and watch this process log that it
received the ReturnHome command, confirming the inbound path works too.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import MapUpdate, Pose, Waypoint  # noqa: E402

# Import the sibling module whether run as a script or as a module.
try:
    from app import MapServer  # when run as: python server/demo_broadcast.py
except ImportError:  # pragma: no cover
    from server.app import MapServer

WIDTH, HEIGHT, RES = 64, 48, 0.05
ORIGIN = {"x": -1.6, "y": -1.2}

# Persistent known grid that accumulates as the synthetic rover "discovers" the room.
_known = [-1] * (WIDTH * HEIGHT)


def _truth(c: int, r: int) -> int:
    border = c == 0 or c == WIDTH - 1 or r == 0 or r == HEIGHT - 1
    obstacle = 40 <= c <= 48 and 18 <= r <= 30
    return 100 if (border or obstacle) else 0


def synth_frame(t: float) -> MapUpdate:
    """Build one synthetic MapUpdate: a rover circling while the map fills in."""
    cx = math.cos(t * 0.5) * 0.9
    cy = math.sin(t * 0.5) * 0.6
    radius = 0.55
    # Reveal cells near the rover (accumulating into the persistent grid).
    for r in range(HEIGHT):
        for c in range(WIDTH):
            wx = ORIGIN["x"] + (c + 0.5) * RES
            wy = ORIGIN["y"] + (r + 0.5) * RES
            if (wx - cx) ** 2 + (wy - cy) ** 2 <= radius * radius:
                _known[r * WIDTH + c] = _truth(c, r)
    pose = Pose(x=cx, y=cy, theta=t * 0.5 + math.pi / 2, timestamp=time.time())
    return MapUpdate(WIDTH, HEIGHT, RES, dict(ORIGIN), list(_known), pose, [])


def main() -> None:
    server = MapServer()
    server.on_return_request = lambda: print("[demo] ReturnHome received from a client")
    server.run_in_thread(port=8000)
    print("Serving on http://localhost:8000  (open /?live to force the real socket)")

    t = 0.0
    try:
        while True:
            server.publish(synth_frame(t))
            t += 0.1
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
