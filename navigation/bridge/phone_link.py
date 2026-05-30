"""Phone perception source via Record3D (iPhone/iPad Pro: camera + LiDAR + pose).

This replaces the mono webcam + Modulino IMU + Modulino ToF as the SLAM/odometry source.
The phone's AR stack does the hard SLAM on-device; Record3D streams RGBD + camera pose to
us over USB or WiFi, and we reshape each frame into the project's contract (a Pose plus a
depth frame for mapping).

This is a hardware bridge, the perception analog of car_link.py. record3d is imported
lazily so the module stays importable before deps are installed.

Setup:
  - Install the Record3D iOS app on an iPhone/iPad Pro; enable "USB Streaming" in Settings.
  - pip install record3d
  - Connect the phone (USB recommended for the heavy RGBD stream).
Docs: https://record3d.app/features  |  https://github.com/marek-simonik/record3d
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import Pose  # noqa: E402

try:
    from record3d import Record3DStream

    _RECORD3D = True
except ImportError:  # pragma: no cover - exercised only without the dep
    _RECORD3D = False


@dataclass
class PhoneFrame:
    """One synchronized frame from the phone."""

    pose: Pose  # camera pose in the map frame (already reshaped to our 2D Pose)
    depth: np.ndarray  # HxW float32, meters; 0 or NaN = invalid
    intrinsics: np.ndarray  # 3x3 camera matrix, for projecting depth into the grid
    timestamp: float


def _pose_from_camera(cam, timestamp: float) -> Pose:
    """Convert a Record3D camera transform into our ground-plane 2D Pose.

    Record3D gives a 6-DoF transform (quaternion qx,qy,qz,qw + translation tx,ty,tz) in the
    phone's AR world frame (typically Y-up). We project it onto the ground plane.

    TODO: confirm the axis mapping for YOUR mounting (how the phone sits on the rover).
          The mapping below assumes the phone is upright with the camera looking forward:
          x_map = tz (forward), y_map = tx (lateral), theta = yaw about the up (Y) axis.
          Calibrate this once on the bench by driving a known path and checking the Pose.
    """
    qx, qy, qz, qw = cam.qx, cam.qy, cam.qz, cam.qw
    # Yaw about the up (Y) axis. Approximate; verify against your mounting convention.
    yaw = math.atan2(2.0 * (qw * qy + qz * qx), 1.0 - 2.0 * (qy * qy + qx * qx))
    return Pose(x=float(cam.tz), y=float(cam.tx), theta=float(yaw), timestamp=timestamp)


class PhoneLink:
    """Owns the Record3D stream and yields PhoneFrames."""

    def __init__(self, device_index: int = 0) -> None:
        self.device_index = device_index
        self._stream = None
        self._new = False

    def connect(self) -> None:
        """Connect to the first available Record3D device. Raises if none / no dep."""
        if not _RECORD3D:
            raise RuntimeError(
                "record3d not installed; pip install record3d (and the Record3D iOS app)"
            )
        devices = Record3DStream.get_connected_devices()
        if not devices:
            raise RuntimeError(
                "no Record3D device found; connect the phone and enable USB Streaming"
            )
        self._stream = Record3DStream()
        self._stream.on_new_frame = self._on_new_frame
        self._stream.connect(devices[self.device_index])

    def _on_new_frame(self) -> None:
        self._new = True

    def read(self) -> Optional[PhoneFrame]:
        """Return the latest PhoneFrame, or None if no new frame is ready.

        Non-blocking: relies on the on_new_frame callback flag so the orchestrator loop
        never stalls waiting on the phone.
        """
        if self._stream is None:
            raise RuntimeError("call connect() first")
        if not self._new:
            return None
        self._new = False
        # Pull the synchronized RGBD + pose for this frame.
        depth = self._stream.get_depth_frame()  # HxW meters
        intrinsics = self._stream.get_intrinsic_mat()  # 3x3
        cam = self._stream.get_camera_pose()  # quaternion + translation
        ts = time.monotonic()
        # TODO: verify these getter names/shapes against your installed record3d version.
        return PhoneFrame(
            pose=_pose_from_camera(cam, ts),
            depth=np.asarray(depth, dtype=np.float32),
            intrinsics=np.asarray(intrinsics, dtype=np.float32),
            timestamp=ts,
        )
