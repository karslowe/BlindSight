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
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # navigation, for perception.*
from schemas.schemas import Pose  # noqa: E402
from perception.pointcloud import pose_from_extrinsic  # noqa: E402

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
    extrinsic: np.ndarray  # 4x4 camera->world (ARKit world, Y up); full 6-DoF for the 3D viz
    timestamp: float
    rgb: Optional[np.ndarray] = None  # HcxWcx3 uint8 camera image (higher res than depth)
    confidence: Optional[np.ndarray] = None  # HxW ARKit confidence 0/1/2 (low/med/high) per depth px


def _intrinsics_to_mat(coeffs) -> np.ndarray:
    """Build the 3x3 camera matrix our back-projection expects from Record3D's intrinsics.

    Record3D returns an IntrinsicMatrixCoeffs object (fx, fy, tx, ty), NOT a matrix - and its
    (tx, ty) is the principal point (cx, cy). Assemble [[fx,0,cx],[0,fy,cy],[0,0,1]].
    """
    fx, fy = float(coeffs.fx), float(coeffs.fy)
    cx, cy = float(coeffs.tx), float(coeffs.ty)
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)


def _field(cam, name):
    """Read a Record3D pose field, tolerant of attribute (cam.qx) or key (cam['qx']) access."""
    return cam[name] if hasattr(cam, "__getitem__") else getattr(cam, name)


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """3x3 rotation matrix from a unit quaternion (normalized defensively)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float32)


def _extrinsic_from_camera(cam) -> np.ndarray:
    """Full 4x4 camera->world transform from Record3D's quaternion + translation (ARKit world,
    Y up). This keeps pitch and roll (a yaw-only pose would collapse them), so the 3D viz can
    place tilted views correctly instead of ramping vertical surfaces. pose_from_extrinsic()
    derives the 2D nav pose from this same matrix, so all frames stay consistent."""
    R = _quat_to_rot(
        float(_field(cam, "qx")), float(_field(cam, "qy")),
        float(_field(cam, "qz")), float(_field(cam, "qw")),
    )
    M = np.eye(4, dtype=np.float32)
    M[:3, :3] = R
    M[0, 3] = float(_field(cam, "tx"))
    M[1, 3] = float(_field(cam, "ty"))
    M[2, 3] = float(_field(cam, "tz"))
    return M


class PhoneLink:
    """Owns the Record3D stream and yields PhoneFrames.

    SELF-HEALING: the iPhone stream drops whenever the phone sleeps / the app backgrounds /
    USB hiccups. read() watches for that (a stream-stopped callback AND a no-frames watchdog)
    and transparently reconnects when the device comes back, so the orchestrator loop never
    has to be restarted - it just resumes once the phone wakes.
    """

    def __init__(self, device_index: int = 0, stale_timeout_s: float = 3.0,
                 reconnect_interval_s: float = 2.0) -> None:
        self.device_index = device_index
        self.stale_timeout_s = stale_timeout_s  # no frames for this long -> assume dropped
        self.reconnect_interval_s = reconnect_interval_s  # how often to retry while down
        self._stream = None
        self._new = False
        self._stopped = False  # set by the on_stream_stopped callback
        self._lost = False  # currently in a dropped state (for one-shot logging)
        self._last_frame_t = 0.0
        self._last_reconnect_t = 0.0
        # Bumped on every (re)connect. A RECONNECT means ARKit restarted with a NEW world
        # origin, so anything accumulated in the old frame is stale - consumers compare this
        # to detect that and reset their map/cloud instead of mixing two coordinate frames.
        self.epoch = 0

    def _open_stream(self) -> bool:
        """(Re)create and connect the Record3D stream to the device. Returns True on success,
        False if no device is currently visible (phone asleep / unplugged) - caller retries."""
        if not _RECORD3D:
            raise RuntimeError(
                "record3d not installed; pip install record3d (and the Record3D iOS app)"
            )
        devices = Record3DStream.get_connected_devices()
        if not devices or self.device_index >= len(devices):
            return False
        if self._stream is not None:
            try:
                self._stream.disconnect()  # clean up the old (dead) session
            except Exception:
                pass
        self._stream = Record3DStream()
        self._stream.on_new_frame = self._on_new_frame
        try:
            self._stream.on_stream_stopped = self._on_stream_stopped  # older bindings may lack it
        except Exception:
            pass
        self._stopped = False
        self._new = False
        self._stream.connect(devices[self.device_index])
        self._last_frame_t = time.monotonic()  # grace period before the watchdog can fire
        self.epoch += 1  # new (or restarted) ARKit session -> a new world origin
        return True

    def connect(self) -> None:
        """Connect to the first available Record3D device. Raises if none / no dep. After this,
        read() keeps the link alive on its own (auto-reconnect)."""
        if not _RECORD3D:
            raise RuntimeError(
                "record3d not installed; pip install record3d (and the Record3D iOS app)"
            )
        if not self._open_stream():
            raise RuntimeError(
                "no Record3D device found; connect the phone and enable USB Streaming"
            )

    def _on_new_frame(self) -> None:
        self._new = True

    def _on_stream_stopped(self) -> None:
        self._stopped = True

    def _build_frame(self, ts: float) -> PhoneFrame:
        """Pull the synchronized RGBD + pose for the current frame into a PhoneFrame."""
        depth = self._stream.get_depth_frame()  # HxW, meters
        coeffs = self._stream.get_intrinsic_mat()  # IntrinsicMatrixCoeffs(fx, fy, tx, ty)
        cam = self._stream.get_camera_pose()  # CameraPose(qx/qy/qz/qw, tx/ty/tz)
        rgb = np.asarray(self._stream.get_rgb_frame())  # HcxWcx3 uint8 (higher res than depth)
        if rgb.ndim == 3 and rgb.shape[2] > 3:
            rgb = rgb[..., :3]  # drop alpha if present
        # ARKit confidence per depth pixel (0=low/1=med/2=high). Low = no real LiDAR return.
        try:
            conf = np.asarray(self._stream.get_confidence_frame())
        except Exception:
            conf = None
        # Full 6-DoF transform; the 2D pose is derived from it (same map convention as the
        # cloud/mesh) so the rover marker and the 3D geometry stay aligned.
        extrinsic = _extrinsic_from_camera(cam)
        px, py, th = pose_from_extrinsic(extrinsic)
        return PhoneFrame(
            pose=Pose(x=px, y=py, theta=th, timestamp=ts),
            depth=np.asarray(depth, dtype=np.float32),
            intrinsics=_intrinsics_to_mat(coeffs),
            extrinsic=extrinsic,
            timestamp=ts,
            rgb=rgb,
            confidence=conf,
        )

    def read(self) -> Optional[PhoneFrame]:
        """Return the latest PhoneFrame, or None if no new frame is ready.

        Non-blocking: relies on the on_new_frame flag so the loop never stalls. If the stream
        has dropped (callback or no frames for stale_timeout_s), it retries the connection
        every reconnect_interval_s and resumes automatically when the phone comes back.
        """
        if self._stream is None:
            raise RuntimeError("call connect() first")
        now = time.monotonic()

        if self._new:
            self._new = False
            try:
                frame = self._build_frame(now)
            except Exception:
                self._stopped = True  # the stream died mid-read; fall through to reconnect
                return None
            self._last_frame_t = now
            if self._lost:
                print("[phone] stream recovered")
                self._lost = False
            return frame

        # No new frame: is the stream down? If so, retry the connection on a throttle.
        down = self._stopped or (now - self._last_frame_t) > self.stale_timeout_s
        if down:
            if not self._lost:
                print(f"[phone] stream lost (no frames for {self.stale_timeout_s:.0f}s); "
                      "reconnecting when the phone is back...")
                self._lost = True
            if now - self._last_reconnect_t >= self.reconnect_interval_s:
                self._last_reconnect_t = now
                try:
                    self._open_stream()  # no-op (returns False) while no device is visible
                except Exception:
                    pass
        return None
