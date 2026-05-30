"""SLAM frontend: produce a Pose (and depth) for the rest of the pipeline.

DEFAULT BACKEND: the phone (Record3D). An iPhone/iPad Pro fuses camera + LiDAR + IMU into
AR-grade, metric, drift-resistant SLAM on-device, and streams pose + depth to us. So this
frontend does not implement SLAM itself - it adapts the phone's output into our Pose. That
offloads the hardest part of the project onto consumer hardware. See bridge/phone_link.py.

ALTERNATIVE BACKEND: if you are not using the phone, swap in a self-contained visual-
inertial SLAM (candidates: RTAB-Map, ORB-SLAM3, OpenVINS, Kimera) behind this same
interface, taking the mono frame + ImuSample instead.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import ImuSample, Pose  # noqa: E402

from bridge.phone_link import PhoneFrame, PhoneLink  # noqa: E402


class SlamFrontend:
    """Pose source. With the phone backend, it reshapes Record3D frames into Pose."""

    def __init__(self, phone: Optional[PhoneLink] = None) -> None:
        self.phone = phone or PhoneLink()
        self._last: Optional[PhoneFrame] = None

    def connect(self) -> None:
        """Open the phone stream. Call once at setup."""
        self.phone.connect()

    def process(self, frame=None, imu: Optional[ImuSample] = None) -> Optional[Pose]:
        """Return the latest pose estimate.

        With the phone backend, `frame` and `imu` are ignored: the phone provides the
        camera, depth, and IMU fusion itself. Returns None while no new frame is ready or
        the tracker is still initializing.

        (For an alternative backend, this is where you would feed `frame` + `imu` to the
        chosen SLAM library and return its pose.)
        """
        pf = self.phone.read()
        if pf is None:
            return None
        self._last = pf
        return pf.pose

    def last_depth(self) -> Optional[Tuple]:
        """Depth + intrinsics from the most recent frame, for mapping.

        Returns (depth, intrinsics) or None if no frame yet. This is the richer alternative
        to a single ultrasonic range: a full depth image to project into the occupancy grid.
        """
        if self._last is None:
            return None
        return self._last.depth, self._last.intrinsics

    def is_keyframe(self) -> bool:
        """Kept for interface compatibility; the phone handles keyframing internally."""
        return False
