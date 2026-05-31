"""Turn a phone depth frame into a map-frame 3D point cloud (for the 3D viz only).

`depth_to_points()` is the REAL producer: it back-projects each (sub-sampled) depth pixel
through the camera intrinsics into a 3D camera-frame point, then transforms it into the map
frame using the rover pose. This is what the orchestrator calls once the phone (Record3D) is
streaming depth + intrinsics + pose.

The demo can't call this (it has no real depth frame), so it uses a SYNTHETIC producer in
server/autonomy_demo.py instead. Both feed the same MapUpdate.point_cloud field and the same
viewer, so swapping synthetic -> real is a one-line change in the producer.

Navigation never uses this - the point cloud is purely for the 3D visualization.
"""

from __future__ import annotations

import math
from typing import List

import numpy as np


def depth_to_points(
    depth,
    intrinsics,
    pose,
    stride: int = 8,
    min_range_m: float = 0.2,
    max_range_m: float = 4.0,
) -> List[float]:
    """Back-project a depth image to a flat map-frame point list [x0,y0,z0, x1,y1,z1, ...].

    Inputs:
        depth: HxW array of metric depths (meters); 0 or NaN = invalid.
        intrinsics: 3x3 camera matrix [[fx,0,cx],[0,fy,cy],[0,0,1]].
        pose: rover/camera Pose (x, y, theta) in the map frame (ground-plane 2D).
        stride: sub-sample every `stride` pixels to downsample for bandwidth.
        min_range_m / max_range_m: keep only depths in this band.
    Output: flat list of floats (map-frame x, y, z per point), z = height above the floor.

    NOTE: this assumes a simple ground-plane pose (x, y, theta) with the camera looking along
    the heading. When phone_link provides the full 6-DoF camera transform, replace the 2D
    transform below with the full rotation + translation (the mount calibration there).
    """
    d = np.asarray(depth, dtype=np.float32)
    if d.ndim != 2:
        return []
    h, w = d.shape
    fx, fy = float(intrinsics[0][0]), float(intrinsics[1][1])
    cx, cy = float(intrinsics[0][2]), float(intrinsics[1][2])

    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    z = d[ys, xs]
    valid = np.isfinite(z) & (z > min_range_m) & (z < max_range_m)
    z = z[valid]
    xs = xs[valid].astype(np.float32)
    ys = ys[valid].astype(np.float32)
    if z.size == 0:
        return []

    # Camera frame: X right, Y down, Z forward.
    cam_x = (xs - cx) * z / fx
    cam_y = (ys - cy) * z / fy
    cam_z = z

    # Map ground plane: camera Z (forward) along the heading, camera X lateral, camera Y up.
    ct, st = math.cos(pose.theta), math.sin(pose.theta)
    mx = pose.x + cam_z * ct - cam_x * st
    my = pose.y + cam_z * st + cam_x * ct
    mz = -cam_y  # camera Y points down -> world up

    pts = np.empty((mx.size, 3), dtype=np.float32)
    pts[:, 0] = mx
    pts[:, 1] = my
    pts[:, 2] = mz
    return pts.reshape(-1).tolist()
