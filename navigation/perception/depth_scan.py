"""Reduce a phone depth frame to a 2D obstacle scan for the occupancy grid (NAVIGATION).

depth_to_scan() turns the full depth image into the list of (angle_offset_rad, range_m) rays
the OccupancyGrid expects - a "virtual laser scan." It is what lets the brain MAP from the
phone as it drives, instead of the single forward ultrasonic ray.

How it works:
  1. Project the depth to map-frame 3D points with the FULL 6-DoF transform (so it is correct
     even when the phone/rover is pitched - reuses perception.pointcloud.project_depth_grid).
  2. Keep only points in an obstacle HEIGHT BAND above the floor (ignore the floor itself and
     the ceiling), turning the 3D depth into a 2D collision slice.
  3. For each azimuth bin around the rover, take the NEAREST such point -> (angle_offset, range).

Unlike the rest of perception/pointcloud.py (viz only), THIS output is consumed by navigation.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from .pointcloud import project_depth_grid


def depth_to_scan(
    depth,
    intrinsics,
    cam_to_world,
    pose,
    stride: int = 4,
    min_range_m: float = 0.2,
    max_range_m: float = 4.0,
    obstacle_min_h: float = 0.06,
    obstacle_max_h: float = 0.8,
    num_bins: int = 72,
    floor_z: Optional[float] = None,
    floor_percentile: float = 5.0,
) -> List[Tuple[float, float]]:
    """Depth frame -> [(angle_offset_rad, range_m), ...] in the rover body frame.

    angle_offset is relative to pose.theta (CCW+); range is the horizontal distance to the
    nearest obstacle-height point at that bearing. Returns [] if nothing qualifies.

    floor_z: map-frame height of the floor. If None it is estimated per-frame as the
        `floor_percentile` percentile of point heights (auto - no mount constant needed). On
        the real rover with a fixed mount, pass a stable floor (camera height - mount height)
        for less frame-to-frame jitter.
    obstacle_min_h / obstacle_max_h: the band ABOVE the floor that counts as an obstacle (m).
    num_bins: azimuth resolution over the full circle (72 -> 5 degrees per bin).
    """
    mx, my, mz, valid, _ = project_depth_grid(
        depth, intrinsics, cam_to_world, stride, min_range_m, max_range_m
    )
    if not valid.any():
        return []
    x = mx[valid]
    y = my[valid]
    z = mz[valid]

    floor = float(np.percentile(z, floor_percentile)) if floor_z is None else float(floor_z)
    band = (z >= floor + obstacle_min_h) & (z <= floor + obstacle_max_h)
    if not band.any():
        return []
    x = x[band]
    y = y[band]

    dx = x - pose.x
    dy = y - pose.y
    rng = np.hypot(dx, dy)
    rel = np.arctan2(dy, dx) - pose.theta
    rel = (rel + math.pi) % (2.0 * math.pi) - math.pi  # wrap to [-pi, pi]

    bins = np.clip(((rel + math.pi) / (2.0 * math.pi) * num_bins).astype(int), 0, num_bins - 1)
    nearest = np.full(num_bins, np.inf, dtype=np.float64)
    np.minimum.at(nearest, bins, rng)  # min range per azimuth bin

    bin_w = 2.0 * math.pi / num_bins
    scan: List[Tuple[float, float]] = []
    for b in range(num_bins):
        if np.isfinite(nearest[b]):
            ang = -math.pi + (b + 0.5) * bin_w  # bin-center angle_offset (rel. to pose.theta)
            scan.append((float(ang), float(nearest[b])))
    return scan
