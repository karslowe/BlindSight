"""Optional object detection on the mono camera feed.

Interface stub only, and optional. Not on the mission-critical path: the return mission
works without it. If enabled, it annotates the map with detected people, doors, or
obstacles.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List


@dataclass
class Detection:
    """One detected object.

    Fields:
        label: class name, e.g. "person", "door", "obstacle".
        confidence: 0..1 score.
        bbox: pixel box (x, y, w, h) in the source frame.
    """

    label: str
    confidence: float
    bbox: tuple  # (x, y, w, h)


class Detector:
    """Runs optional object detection over camera frames."""

    def __init__(self, enabled: bool = False) -> None:
        """Inputs: enabled, whether detection runs at all (off by default).

        TODO: if enabled, load a lightweight model that fits the edge device
              (e.g. a small YOLO or MobileNet-SSD variant).
        """
        self.enabled = enabled

    def detect(self, frame) -> List[Detection]:
        """Detect objects in one frame.

        Inputs: frame, a camera image array.
        Output: a list of Detection (empty if disabled or nothing found).
        TODO: run inference and map raw outputs into Detection records.
        """
        if not self.enabled:
            return []
        raise NotImplementedError("object detection not implemented yet")
