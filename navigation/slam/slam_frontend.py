"""SLAM frontend: mono frame + IMU -> Pose (and keyframes).

Interface stub only. The rover has no wheel encoders, so odometry is visual-inertial:
one mono USB webcam plus the Modulino IMU.

Candidate backends to swap in later (do not lock one in here): RTAB-Map, ORB-SLAM3,
OpenVINS, Kimera. This class is the seam they plug into.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import ImuSample, Pose  # noqa: E402


class SlamFrontend:
    """Estimates the rover pose by fusing camera frames and IMU samples."""

    def __init__(self) -> None:
        # TODO: instantiate the chosen SLAM backend and any IMU pre-integrator state.
        pass

    def process(self, frame, imu: Optional[ImuSample]) -> Optional[Pose]:
        """Ingest one frame and one IMU sample, return the current pose estimate.

        Inputs:
            frame: a mono camera image (e.g. a numpy array from cv2), or None if dropped.
            imu: the latest ImuSample (accel[3], gyro[3], timestamp), or None.
        Output:
            a Pose (x, y, theta, covariance, timestamp) in the map frame, or None while
            the tracker is still initializing or has lost tracking.
        TODO: feed the frame and IMU to the backend; return its pose as a Pose. On a
              keyframe, also update the internal map structure (see keyframe()).
        """
        raise NotImplementedError("SLAM frontend not implemented yet")

    def is_keyframe(self) -> bool:
        """Whether the last processed frame was selected as a keyframe.

        Output: True on a keyframe, False otherwise.
        TODO: surface the backend's keyframe decision.
        """
        return False
