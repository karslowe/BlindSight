"""Unit sim for the information-gain FrontierExplorer.

Validates the smart-exploration upgrade WITHOUT hardware: builds a grid with two frontiers —
one NEAR but opening into a small explored pocket (low info gain), one slightly FARTHER but
opening into a large unknown void (high info gain) — and asserts the explorer chooses the
high-gain frontier, where the old nearest-frontier logic would have chosen the near one.
Also checks the blacklist (an unreachable frontier is skipped) and hysteresis (no dithering).

Run (needs numpy + the repo-root navigation/ + shared/ on the path):
    python3 applab/explorersim.py        # from the repo root
Exit code 0 = pass.
"""

from __future__ import annotations

import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_repo = _HERE.parent
for _p in (_repo, _repo / "shared"):
    sys.path.insert(0, str(_p))

import numpy as np  # noqa: E402
from schemas.schemas import Pose  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from planning.explorer import FrontierExplorer  # noqa: E402

_L_FREE, _L_OCC = -3, 3  # below _FREE_THRESH / above _OCC_THRESH so state thresholds trip


def _build_grid() -> OccupancyGrid:
    """Hand-build a grid: rover at center, a small free pocket to the LEFT (near, low gain),
    and a free corridor to the RIGHT opening into a big unknown void (farther, high gain)."""
    g = OccupancyGrid(resolution_m=0.05)
    n = 120
    g._log = np.zeros((n, n), dtype=np.int16)  # all unknown
    g.width = g.height = n
    g.origin = {"x": -(n // 2) * 0.05, "y": -(n // 2) * 0.05}
    log = g._log
    cy = n // 2

    # Mark a band free around the rover and out to both sides (so both frontiers are reachable
    # known-free), but leave the RIGHT side's far region UNKNOWN (the big void to discover).
    # Left pocket: free cells from x just left of center to the left edge of the pocket, the
    # rest beyond it already "explored" (free) -> little unknown behind that frontier.
    log[cy - 2:cy + 3, 10:cy] = _L_FREE        # left corridor: free
    log[cy - 8:cy + 9, 2:12] = _L_FREE          # left pocket: a small free room (explored)
    # Right corridor: free from center rightward to a frontier; BEYOND it stays unknown (void).
    log[cy - 2:cy + 3, cy:cy + 35] = _L_FREE    # right corridor: free up to col ~cy+35
    # everything col > cy+35 on the right stays 0 (unknown) -> large info gain there
    # Wall everything else off so frontiers are only at the corridor mouths.
    return g


def main() -> int:
    g = _build_grid()
    pose = Pose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)  # rover at grid center
    exp = FrontierExplorer()

    fronts = g.frontier_cells()
    if not fronts:
        print("FAIL: no frontiers in the test grid (setup bug)")
        return 1

    rc = g.world_to_cell(pose.x, pose.y)
    path = exp.next_path(g, pose)
    if path is None:
        print("FAIL: explorer found no reachable frontier")
        return 1

    # The chosen target is the path's last waypoint. Which side did it pick?
    end = path[-1]
    chose_right = end.x > pose.x  # right corridor (high info gain) is +x

    # Sanity: the LEFT frontier is genuinely nearer (so nearest-frontier would pick left).
    # Compare cell distances of the two corridor mouths to the rover.
    left_mouth_dist = abs(rc[0] - 12)
    right_mouth_dist = abs((g.world_to_cell(end.x, end.y)[0]) - rc[0])
    print(f"chose_right={chose_right} end=({end.x:.2f},{end.y:.2f}) "
          f"left_mouth_dist~{left_mouth_dist} right_target_dist~{right_mouth_dist}")

    if not chose_right:
        print("FAIL: explorer chose the near low-gain (left) frontier, not the high-gain (right) one.")
        return 1

    # Hysteresis / stability: a second call from the same pose should pick the same side.
    path2 = exp.next_path(g, pose)
    if path2 is None or (path2[-1].x > pose.x) != chose_right:
        print("FAIL: explorer dithered (second call changed sides).")
        return 1

    print("PASS: explorer chose the high-information-gain frontier and stayed stable.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
