"""Turn phone depth frames into a triangle mesh ("3D scan") - for the 3D viz only.

This is the surface-reconstruction counterpart to perception/pointcloud.py. Where that one
emits loose points, this one emits a connected mesh (vertices + triangle faces), so the 3D
viewer can render solid surfaces instead of dots.

Approach: ORGANIZED DEPTH MESHING with voxel vertex-welding. A depth frame is a grid, so
neighboring valid pixels are already adjacent on a surface - we back-project each to the map
frame (the same projection pointcloud.depth_to_points uses, already validated on-device) and
stitch each 2x2 block of pixels into two triangles. To stop overlapping frames from piling up
duplicate surfaces, every vertex is snapped to a voxel grid and shared by index (welded), so
revisiting a spot reuses the same vertices and the surfaces merge. Triangles that straddle a
depth discontinuity (an occlusion edge) are dropped so we do not stretch skin over gaps.

Why not Open3D TSDF/Poisson: no Open3D wheel for this Python, and TSDF needs the full 6-DoF
camera-to-world pose in Open3D's exact convention (an untested calibration). This reuses the
2D-pose projection that already produced a good point cloud, so it has no new unknowns. It is
a laptop-demo tool, not a rover thing - see phone_mesh_demo.py for why.

Navigation never uses this. Output: {"vertices": [x,y,z, ...], "faces": [i,j,k, ...]} (flat).
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .pointcloud import project_depth_grid, sample_rgb_grid


class DepthMesher:
    """Accumulates phone depth frames into one welded triangle mesh."""

    def __init__(
        self,
        voxel_m: float = 0.035,
        stride: int = 4,
        min_range_m: float = 0.2,
        max_range_m: float = 4.0,
        edge_jump_m: float = 0.12,
        max_vertices: int = 80000,
        max_faces: int = 160000,
    ) -> None:
        self.voxel_m = voxel_m
        self.stride = stride
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.edge_jump_m = edge_jump_m  # drop a triangle if its pixels span more depth than this
        self.max_vertices = max_vertices
        self.max_faces = max_faces

        self._vmap: dict = {}  # voxel key (kx,ky,kz) -> vertex index
        self._verts: list = []  # list of (x, y, z) map-frame vertices
        self._vcols: list = []  # list of (r, g, b) per-vertex colors, parallel to _verts
        self._faces: list = []  # list of (i, j, k) vertex-index triangles
        self.full = False

    def _vertex(self, key, x: float, y: float, z: float, color) -> int:
        """Return the index of the welded vertex at `key`, creating it (with `color`) if new.
        Returns -1 if the vertex cap is hit. The first color seen for a voxel is kept."""
        idx = self._vmap.get(key)
        if idx is not None:
            return idx
        if len(self._verts) >= self.max_vertices:
            self.full = True
            return -1
        idx = len(self._verts)
        self._verts.append((x, y, z))
        self._vcols.append((int(color[0]), int(color[1]), int(color[2])))
        self._vmap[key] = idx
        return idx

    def _tri(self, a: int, b: int, c: int) -> None:
        """Append a triangle, skipping degenerate ones (a welded duplicate corner)."""
        if a < 0 or b < 0 or c < 0 or a == b or b == c or a == c:
            return
        if len(self._faces) < self.max_faces:
            self._faces.append((a, b, c))

    def add_frame(self, depth, intrinsics, cam_to_world, rgb=None) -> None:
        """Fold one depth frame (HxW meters) plus its full 6-DoF camera transform into the mesh.
        If `rgb` (the camera image) is given, each vertex is colored from it."""
        if self.full:
            return
        if np.asarray(depth).ndim != 2:
            return
        # Full 6-DoF back-projection, keeping the pixel grid so we can stitch triangles.
        mx, my, mz, valid, z = project_depth_grid(
            depth, intrinsics, cam_to_world, self.stride, self.min_range_m, self.max_range_m
        )
        # Per-pixel color (sampled from the camera image), or a neutral gray if no RGB.
        cols = sample_rgb_grid(rgb, np.asarray(depth).shape, self.stride) if rgb is not None else None

        # Voxel keys for welding.
        v = self.voxel_m
        kx = np.round(mx / v).astype(np.int64)
        ky = np.round(my / v).astype(np.int64)
        kz = np.round(mz / v).astype(np.int64)

        gh, gw = z.shape
        vidx = np.full((gh, gw), -1, dtype=np.int64)
        for i in range(gh):
            for j in range(gw):
                if valid[i, j]:
                    color = cols[i, j] if cols is not None else (200, 200, 200)
                    vidx[i, j] = self._vertex(
                        (int(kx[i, j]), int(ky[i, j]), int(kz[i, j])),
                        float(mx[i, j]), float(my[i, j]), float(mz[i, j]), color,
                    )

        # Stitch each 2x2 pixel block into two triangles (skip occlusion-edge quads).
        ej = self.edge_jump_m
        for i in range(gh - 1):
            for j in range(gw - 1):
                # int() so face indices stay plain Python ints (JSON-serializable), not int64.
                a, b = int(vidx[i, j]), int(vidx[i, j + 1])
                c, e = int(vidx[i + 1, j + 1]), int(vidx[i + 1, j])
                if a < 0 or b < 0 or c < 0 or e < 0:
                    continue
                zq = (z[i, j], z[i, j + 1], z[i + 1, j + 1], z[i + 1, j])
                if max(zq) - min(zq) > ej:
                    continue  # straddles a depth jump - do not skin over the gap
                self._tri(a, b, c)
                self._tri(a, c, e)

    def mesh_dict(self) -> Optional[dict]:
        """Export the accumulated mesh as flat lists, or None if nothing is built yet."""
        if not self._faces:
            return None
        verts: list = []
        for x, y, z in self._verts:
            verts.append(x)
            verts.append(y)
            verts.append(z)
        faces: list = []
        for a, b, c in self._faces:
            faces.append(a)
            faces.append(b)
            faces.append(c)
        out = {"vertices": verts, "faces": faces}
        # Per-vertex colors (flat [r,g,b,...], 0..255), present only if RGB frames were fed in.
        if self._vcols:
            cols: list = []
            for r, g, b in self._vcols:
                cols.append(r)
                cols.append(g)
                cols.append(b)
            out["colors"] = cols
        return out

    def stats(self) -> tuple:
        return len(self._verts), len(self._faces)
