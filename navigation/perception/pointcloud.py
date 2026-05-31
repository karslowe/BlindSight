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


# ======================================================================================
# Full 6-DoF projection. Unlike depth_to_points() above (which uses only the 2D pose and
# assumes a LEVEL camera, so a tilted phone ramps vertical surfaces), this uses the phone's
# COMPLETE orientation (the 4x4 camera->world transform), so pitch and roll are handled and
# walls stay vertical at any hold/mount angle. This is the projection the live demos use.
# ======================================================================================

# Record3D's depth + intrinsics use the vision camera convention (X right, Y DOWN, Z forward).
# ARKit's camera->world is in the ARKit camera convention (X right, Y UP, Z backward). Convert
# vision-camera points to ARKit-camera by flipping Y and Z (a 180 deg rotation about X).
_VISION_TO_ARKIT_CAM = np.array([1.0, -1.0, -1.0], dtype=np.float32)

# CALIBRATION KNOBS - flip a sign here if the first real scan comes out wrong, then re-run:
#   * whole scene UPSIDE DOWN -> set _UP_SIGN = -1.0
#   * whole scene MIRRORED    -> negate _Y_SIGN
# ARKit world is Y-up; we map it to our ground plane (map x, map y) plus height (map z).
_UP_SIGN = 1.0
_Y_SIGN = -1.0  # ARKit forward (-Z) -> +map y, by default


def _world_to_map(wx, wy, wz):
    """ARKit world (X, Y-up, Z) -> our map frame (x, y on the ground, z = height). Linear, so
    it applies to both positions and direction vectors. The single place the axis convention
    lives, shared by the point cloud, the mesh, and the pose."""
    return wx, _Y_SIGN * wz, _UP_SIGN * wy


def project_depth_grid(depth, intrinsics, cam_to_world, stride=4,
                       min_range_m=0.2, max_range_m=4.0):
    """Back-project a depth frame to MAP-FRAME points using the full 6-DoF camera transform,
    KEEPING the pixel-grid structure so callers can both list points and stitch triangles.

    Inputs: depth HxW meters; intrinsics 3x3; cam_to_world 4x4 (PhoneFrame.extrinsic, ARKit
    world, Y up); stride; range band.
    Returns (mx, my, mz, valid, z), each shaped (gh, gw). mz is height above the floor.
    """
    d = np.asarray(depth, dtype=np.float32)
    h, w = d.shape
    fx, fy = float(intrinsics[0][0]), float(intrinsics[1][1])
    cx, cy = float(intrinsics[0][2]), float(intrinsics[1][2])

    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    z = d[ys, xs]
    valid = np.isfinite(z) & (z > min_range_m) & (z < max_range_m)
    # Invalid pixels are zeroed (masked out by `valid` on return); errstate keeps numpy's
    # float32-matmul FPE flag from emitting spurious divide/overflow warnings on them.
    with np.errstate(invalid="ignore", divide="ignore", over="ignore"):
        zs = np.where(valid, z, 0.0).astype(np.float32)
        # Vision camera frame (X right, Y down, Z forward) -> ARKit camera (Y up, Z back).
        cam = np.stack([
            ((xs - cx) * zs / fx) * _VISION_TO_ARKIT_CAM[0],
            ((ys - cy) * zs / fy) * _VISION_TO_ARKIT_CAM[1],
            zs * _VISION_TO_ARKIT_CAM[2],
        ], axis=0).reshape(3, -1)
        M = np.asarray(cam_to_world, dtype=np.float32)
        world = M[:3, :3] @ cam + M[:3, 3:4]  # 3 x N in ARKit world (Y up)

    gh, gw = z.shape
    mx, my, mz = _world_to_map(world[0], world[1], world[2])
    return (mx.reshape(gh, gw).astype(np.float32),
            my.reshape(gh, gw).astype(np.float32),
            mz.reshape(gh, gw).astype(np.float32), valid, z)


def depth_to_points_6dof(depth, intrinsics, cam_to_world, stride=8,
                         min_range_m=0.2, max_range_m=4.0) -> List[float]:
    """Flat [x0,y0,z0, ...] map-frame point list via the full 6-DoF transform. The 6-DoF
    counterpart of depth_to_points(); use with PhoneFrame.extrinsic so tilted views render
    correctly instead of ramping vertical surfaces."""
    if np.asarray(depth).ndim != 2:
        return []
    mx, my, mz, valid, _ = project_depth_grid(
        depth, intrinsics, cam_to_world, stride, min_range_m, max_range_m
    )
    if not valid.any():
        return []
    pts = np.stack([mx[valid], my[valid], mz[valid]], axis=1).astype(np.float32)
    return pts.reshape(-1).tolist()


def sample_rgb_grid(rgb, depth_shape, stride=4):
    """Sample the camera image at the SAME (stride) pixel grid project_depth_grid uses, scaling
    from depth resolution up to RGB resolution. Returns (gh, gw, 3) uint8 (RGB).

    Assumes RGB and depth share orientation + field of view (Record3D presents them as one
    aligned RGBD frame; only the resolution differs). If colors come out rotated/mirrored, that
    assumption is off for your capture and the index scaling here is where to fix it.
    """
    h, w = depth_shape
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    r = np.asarray(rgb)
    hc, wc = r.shape[0], r.shape[1]
    rys = np.clip((ys * hc) // h, 0, hc - 1)
    rxs = np.clip((xs * wc) // w, 0, wc - 1)
    return r[rys, rxs]


def depth_to_points_rgb_6dof(depth, intrinsics, cam_to_world, rgb, stride=8,
                             min_range_m=0.2, max_range_m=4.0):
    """Like depth_to_points_6dof, but also returns per-point color sampled from the camera image.
    Returns (points_flat [x,y,z,...] floats, colors_flat [r,g,b,...] as 0..255 ints)."""
    if np.asarray(depth).ndim != 2:
        return [], []
    mx, my, mz, valid, _ = project_depth_grid(
        depth, intrinsics, cam_to_world, stride, min_range_m, max_range_m
    )
    if not valid.any():
        return [], []
    pts = np.stack([mx[valid], my[valid], mz[valid]], axis=1).astype(np.float32)
    cols = sample_rgb_grid(rgb, np.asarray(depth).shape, stride)[valid].astype(np.int32)
    return pts.reshape(-1).tolist(), cols.reshape(-1).tolist()


def pose_from_extrinsic(cam_to_world):
    """Project the full camera transform to a ground-plane (x, y, theta) IN THE SAME map frame
    as the cloud/mesh, so the rover marker, grid, and 3D geometry all line up. Position is the
    camera's mapped location; theta is its viewing direction (ARKit camera -Z) on the ground.
    Returns (x, y, theta)."""
    M = np.asarray(cam_to_world, dtype=np.float32)
    px, py, _h = _world_to_map(float(M[0, 3]), float(M[1, 3]), float(M[2, 3]))
    fx_w, fy_w, fz_w = -float(M[0, 2]), -float(M[1, 2]), -float(M[2, 2])  # camera forward in world
    fmx, fmy, _fh = _world_to_map(fx_w, fy_w, fz_w)
    return float(px), float(py), float(math.atan2(fmy, fmx))
