"""Object detection on the camera feed (YOLO), for the "find a target" mission.

The rover explores until it detects a target object, then (the base case) marks where it is
and returns home. This module is the seam where YOLO runs on the real camera frames; the
demo (server/autonomy_demo.py) simulates the same detections geometrically so the whole
search-find-return flow is testable with no model or camera.

ultralytics is imported lazily so this stays importable before deps are installed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

try:
    from ultralytics import YOLO

    _ULTRALYTICS = True
except ImportError:  # pragma: no cover - exercised only without the dep
    _ULTRALYTICS = False


@dataclass
class Detection:
    """One detected object in an image.

    Fields:
        label: class name, e.g. "person", "door".
        confidence: 0..1 score.
        bbox: pixel box (x, y, w, h) in the source frame.
    """

    label: str
    confidence: float
    bbox: tuple  # (x, y, w, h)


class Detector:
    """Runs YOLO over camera frames, looking for a target class."""

    def __init__(
        self,
        target_label: str = "person",
        min_confidence: float = 0.5,
        model: str = "yolov8n.pt",
        enabled: bool = False,
    ) -> None:
        """Inputs: target_label, the class to search for; min_confidence threshold; model,
        the (small) YOLO weights to load; enabled, whether detection runs at all.

        A tiny model (yolov8n) keeps inference fast enough for the edge device. Run
        detection at a reduced rate (a few Hz), not every control tick.
        """
        self.target_label = target_label
        self.min_confidence = min_confidence
        self.enabled = enabled
        self._model = None
        if enabled and _ULTRALYTICS:
            self._model = YOLO(model)

    def detect(self, frame) -> List[Detection]:
        """Detect target-class objects in one frame.

        Inputs: frame, a camera image array (e.g. the phone's RGB frame).
        Output: a list of Detection for the target class above the confidence threshold
                (empty if disabled, no model, or nothing found).
        """
        if not self.enabled or self._model is None or frame is None:
            return []
        results = self._model(frame, verbose=False)
        out: List[Detection] = []
        for r in results:
            for box in r.boxes:
                label = self._model.names[int(box.cls)]
                conf = float(box.conf)
                if label == self.target_label and conf >= self.min_confidence:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    out.append(Detection(label, conf, (x1, y1, x2 - x1, y2 - y1)))
        return out

    @staticmethod
    def detection_to_world(det: Detection, depth, intrinsics, pose) -> Optional[tuple]:
        """Project a detection's box center to a map-frame (x, y).

        Output: (world_x, world_y) or None if no valid depth.
        TODO: read depth at the bbox center, back-project through intrinsics to a 3D camera
              point, transform into the map frame by `pose` (mirrors occupancy_grid's depth
              projection). The demo provides the world position directly instead.
        """
        raise NotImplementedError("detection-to-world projection not implemented yet")
