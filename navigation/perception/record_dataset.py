"""Record live phone (Record3D) frames to a Nerfstudio/DN-Splatter dataset on disk.

This is the CAPTURE half of the offline photorealistic-reconstruction path. The rover streams
RGBD + ARKit pose to the laptop over Record3D (bridge/phone_link.py); this turns that live
stream into a folder of posed RGB + LiDAR-depth frames that DN-Splatter (a Nerfstudio 3D
Gaussian Splatting method built for iPhone RGBD) trains on OFFLINE on the GPU box. Nothing
here needs the GPU or real-time compute - it just writes files.

It is the recorder analog of server/phone_cloud_demo.py: same PhoneLink, but instead of
projecting + publishing to a viewer, it writes a training dataset.

Output (Nerfstudio "nerfstudio" dataparser layout, one folder per ARKit tracking session):
    <out>/session_01/
        transforms.json        # intrinsics (RGB res) + per-frame camera->world transform
        images/frame_00001.jpg # RGB, full camera resolution
        depths/frame_00001.png # uint16 millimeters, upsampled to RGB res, low-conf pixels = 0

Two conventions make this clean (see docs/reconstruction.md):
  * ARKit's camera frame (+X right, +Y up, -Z forward) IS Nerfstudio's transform_matrix
    convention, so PhoneFrame.extrinsic (the RAW ARKit cam->world, before _world_to_map)
    drops straight in - no axis remapping, no COLMAP pose solve.
  * RGB and depth share the camera FOV (Record3D presents them as one aligned RGBD frame), so
    the depth intrinsics scale to RGB resolution by the resolution ratio alone. This matches
    the assumption already made in perception.pointcloud.sample_rgb_grid.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np


def _rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Geodesic angle (rad) between two 3x3 rotation matrices. Used by the keyframe gate so a
    pure rotation in place still triggers a new keyframe even with no translation."""
    rel = a.T @ b
    cos = (float(np.trace(rel)) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, cos)))


@dataclass
class DatasetRecorder:
    """Selects keyframes from the live stream and writes them as a Nerfstudio dataset.

    Call on_epoch(link.epoch) then add(frame) each loop. A change in epoch means ARKit
    restarted with a NEW world origin, so poses before and after are in incompatible frames -
    the recorder finalizes the current session and opens a fresh one (one continuous walk =
    one session). One trains on a single session folder.
    """

    out_root: Path
    trans_thresh_m: float = 0.05   # keep a frame after this much camera translation...
    rot_thresh_deg: float = 5.0    # ...or this much rotation (whichever comes first)
    depth_conf_min: int = 2        # zero out depth pixels below this ARKit confidence (0/1/2)
    jpg_quality: int = 95
    depth_scale_mm: float = 1000.0  # meters -> the uint16 PNG unit (mm); train side divides back

    _epoch: Optional[int] = field(default=None, init=False)
    _session_idx: int = field(default=0, init=False)
    _session_dir: Optional[Path] = field(default=None, init=False)
    _frames: List[dict] = field(default_factory=list, init=False)
    _intr: Optional[dict] = field(default=None, init=False)
    _last_M: Optional[np.ndarray] = field(default=None, init=False)
    _kept: int = field(default=0, init=False)
    total_kept: int = field(default=0, init=False)

    def on_epoch(self, epoch: int) -> None:
        """Roll over to a new session folder when the ARKit origin changes (or on first call)."""
        if epoch == self._epoch:
            return
        self._finalize()  # write transforms.json for the session that just ended (if any)
        self._epoch = epoch
        self._session_idx += 1
        self._session_dir = self.out_root / f"session_{self._session_idx:02d}"
        (self._session_dir / "images").mkdir(parents=True, exist_ok=True)
        (self._session_dir / "depths").mkdir(parents=True, exist_ok=True)
        self._frames = []
        self._intr = None
        self._last_M = None
        self._kept = 0
        print(f"[record] new tracking session -> {self._session_dir}")

    def _is_keyframe(self, M: np.ndarray) -> bool:
        """True if the camera has moved/turned enough since the last kept frame."""
        if self._last_M is None:
            return True
        dt = float(np.linalg.norm(M[:3, 3] - self._last_M[:3, 3]))
        if dt >= self.trans_thresh_m:
            return True
        return math.degrees(_rotation_angle(self._last_M[:3, :3], M[:3, :3])) >= self.rot_thresh_deg

    def add(self, frame) -> bool:
        """Consider one PhoneFrame; write it if it clears the keyframe gate. Returns kept?"""
        if self._session_dir is None:
            raise RuntimeError("call on_epoch() before add()")
        if frame.rgb is None:
            return False  # need color for a photorealistic splat
        M = np.asarray(frame.extrinsic, dtype=np.float64)
        if not self._is_keyframe(M):
            return False

        rgb = np.asarray(frame.rgb)
        hc, wc = rgb.shape[0], rgb.shape[1]

        # Depth (+ confidence) share the camera FOV but are lower-res; upsample to RGB res with
        # nearest-neighbor so we never invent depth by interpolating across an occlusion edge.
        depth = np.asarray(frame.depth, dtype=np.float32)
        depth_up = cv2.resize(depth, (wc, hc), interpolation=cv2.INTER_NEAREST)
        valid = np.isfinite(depth_up) & (depth_up > 0.0)
        if frame.confidence is not None:
            conf = np.asarray(frame.confidence)
            conf_up = cv2.resize(conf, (wc, hc), interpolation=cv2.INTER_NEAREST)
            valid &= conf_up >= self.depth_conf_min  # drop phantom low-confidence LiDAR returns
        depth_mm = np.where(valid, depth_up * self.depth_scale_mm, 0.0)
        depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)

        # Intrinsics: Record3D's matrix matches the DEPTH resolution (that is how
        # perception.pointcloud uses it). Scale to RGB resolution by the resolution ratio,
        # valid because RGB and depth share the field of view.
        if self._intr is None:
            hd, wd = depth.shape
            K = np.asarray(frame.intrinsics, dtype=np.float64)
            sx, sy = wc / float(wd), hc / float(hd)
            self._intr = {
                "w": int(wc), "h": int(hc), "camera_model": "OPENCV",
                "fl_x": float(K[0, 0]) * sx, "fl_y": float(K[1, 1]) * sy,
                "cx": float(K[0, 2]) * sx, "cy": float(K[1, 2]) * sy,
            }

        n = self._kept + 1
        img_rel = f"images/frame_{n:05d}.jpg"
        depth_rel = f"depths/frame_{n:05d}.png"
        # Record3D gives RGB; cv2 writes BGR, so flip channels.
        cv2.imwrite(str(self._session_dir / img_rel), rgb[..., ::-1],
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpg_quality])
        cv2.imwrite(str(self._session_dir / depth_rel), depth_mm)
        self._frames.append({
            "file_path": img_rel,
            "depth_file_path": depth_rel,
            "transform_matrix": M.tolist(),  # cam->world, raw ARKit (= Nerfstudio convention)
        })
        self._last_M = M
        self._kept += 1
        self.total_kept += 1
        return True

    def _finalize(self) -> None:
        """Write transforms.json for the current session (no-op if nothing was kept)."""
        if self._session_dir is None or not self._frames or self._intr is None:
            return
        doc = dict(self._intr)
        # meters = pixel_value / depth_scale_mm; Nerfstudio reads this to scale the depth PNGs.
        doc["depth_unit_scale_factor"] = 1.0 / self.depth_scale_mm
        doc["frames"] = self._frames
        (self._session_dir / "transforms.json").write_text(json.dumps(doc, indent=2))
        print(f"[record] wrote {len(self._frames)} frames -> {self._session_dir}/transforms.json")

    def close(self) -> None:
        """Flush the final session. Call once at shutdown."""
        self._finalize()
