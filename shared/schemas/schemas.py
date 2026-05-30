"""Typed Python mirror of the Recon Rover message contract.

Human source of truth: docs/message-schemas.md
Language-neutral mirror: the *.schema.json files next to this module.

Every message that crosses a component boundary is defined here once and imported by the
navigation brain. Field names match the firmware (C++) and the viewer (JS) exactly.

This module is plain stdlib (dataclasses only) so it imports cheaply on the edge device.
Each dataclass offers:
  - to_dict() / from_dict(): JSON-friendly conversion for the websocket and logs.
  - DriveCommand and CarTelemetry add to_serial_line() / from_serial_line() for the
    line-based ASCII protocol over USB serial.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


# --------------------------------------------------------------------------------------
# 1. DriveCommand  (brain -> car)   Wire: "DRV <linear_velocity> <angular_velocity> <stop>"
# --------------------------------------------------------------------------------------
@dataclass
class DriveCommand:
    """Velocity command from the brain to the car.

    Fields:
        linear_velocity: forward velocity, m/s, forward positive.
        angular_velocity: yaw rate, rad/s, counter-clockwise positive.
        stop: optional hard stop. 1 overrides the velocities and brakes.
    """

    linear_velocity: float
    angular_velocity: float
    stop: int = 0

    def to_serial_line(self) -> str:
        """Serialize to the ASCII line the firmware parses (no trailing newline)."""
        return f"DRV {self.linear_velocity:.4f} {self.angular_velocity:.4f} {self.stop}"

    @classmethod
    def from_serial_line(cls, line: str) -> "DriveCommand":
        """Parse a 'DRV ...' line. Raises ValueError on a malformed line."""
        parts = line.strip().split()
        if not parts or parts[0] != "DRV":
            raise ValueError(f"not a DRV line: {line!r}")
        lin = float(parts[1])
        ang = float(parts[2])
        stop = int(parts[3]) if len(parts) > 3 else 0
        return cls(linear_velocity=lin, angular_velocity=ang, stop=stop)

    def to_dict(self) -> dict:
        return {
            "type": "DriveCommand",
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "stop": self.stop,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "DriveCommand":
        return cls(
            linear_velocity=float(d["linear_velocity"]),
            angular_velocity=float(d["angular_velocity"]),
            stop=int(d.get("stop", 0)),
        )


# --------------------------------------------------------------------------------------
# 2. CarTelemetry  (car -> brain)
#    Wire: "TEL <ultrasonic_distance> <bumper> <line_left> <line_center> <line_right> <timestamp>"
# --------------------------------------------------------------------------------------
@dataclass
class CarTelemetry:
    """Sensor record from the car to the brain.

    Fields:
        ultrasonic_distance: forward range, m. -1 if no echo.
        bumper: bumper or stall flag, 0 or 1.
        line_left / line_center / line_right: line sensor flags, 0 or 1.
        timestamp: car-side time of the reading, seconds.
    """

    ultrasonic_distance: float
    bumper: int
    line_left: int
    line_center: int
    line_right: int
    timestamp: float

    def to_serial_line(self) -> str:
        return (
            f"TEL {self.ultrasonic_distance:.4f} {self.bumper} "
            f"{self.line_left} {self.line_center} {self.line_right} {self.timestamp:.4f}"
        )

    @classmethod
    def from_serial_line(cls, line: str) -> "CarTelemetry":
        """Parse a 'TEL ...' line. Raises ValueError on a malformed line."""
        parts = line.strip().split()
        if not parts or parts[0] != "TEL":
            raise ValueError(f"not a TEL line: {line!r}")
        return cls(
            ultrasonic_distance=float(parts[1]),
            bumper=int(parts[2]),
            line_left=int(parts[3]),
            line_center=int(parts[4]),
            line_right=int(parts[5]),
            timestamp=float(parts[6]),
        )

    def to_dict(self) -> dict:
        return {
            "type": "CarTelemetry",
            "ultrasonic_distance": self.ultrasonic_distance,
            "bumper": self.bumper,
            "line_left": self.line_left,
            "line_center": self.line_center,
            "line_right": self.line_right,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CarTelemetry":
        return cls(
            ultrasonic_distance=float(d["ultrasonic_distance"]),
            bumper=int(d["bumper"]),
            line_left=int(d["line_left"]),
            line_center=int(d["line_center"]),
            line_right=int(d["line_right"]),
            timestamp=float(d["timestamp"]),
        )


# --------------------------------------------------------------------------------------
# 3. Pose  (slam -> mapping, planning)
# --------------------------------------------------------------------------------------
@dataclass
class Pose:
    """Estimated 2D pose of the rover in the map frame.

    Fields:
        x, y: position in the map frame, meters.
        theta: heading, radians, CCW from the map x-axis.
        covariance: row-major 3x3 covariance over (x, y, theta), length 9.
        timestamp: time of the estimate, seconds.
    """

    x: float
    y: float
    theta: float
    timestamp: float
    covariance: List[float] = field(default_factory=lambda: [0.0] * 9)

    def to_dict(self) -> dict:
        return {
            "type": "Pose",
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "covariance": list(self.covariance),
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Pose":
        return cls(
            x=float(d["x"]),
            y=float(d["y"]),
            theta=float(d["theta"]),
            timestamp=float(d["timestamp"]),
            covariance=list(d.get("covariance", [0.0] * 9)),
        )


# --------------------------------------------------------------------------------------
# 4. ImuSample  (modulino -> slam)
# --------------------------------------------------------------------------------------
@dataclass
class ImuSample:
    """One visual-inertial IMU reading from the Modulino Movement sensor.

    Fields:
        accel: linear acceleration (x, y, z), m/s^2.
        gyro: angular velocity (x, y, z), rad/s.
        timestamp: time of the sample, seconds.
    """

    accel: List[float]
    gyro: List[float]
    timestamp: float

    def to_dict(self) -> dict:
        return {
            "type": "ImuSample",
            "accel": list(self.accel),
            "gyro": list(self.gyro),
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "ImuSample":
        return cls(
            accel=list(d["accel"]),
            gyro=list(d["gyro"]),
            timestamp=float(d["timestamp"]),
        )


# --------------------------------------------------------------------------------------
# 5. MapUpdate  (mapping -> server -> viz)
# --------------------------------------------------------------------------------------
@dataclass
class Waypoint:
    """A single point on the planned return path, map-frame meters."""

    x: float
    y: float

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y}

    @classmethod
    def from_dict(cls, d: dict) -> "Waypoint":
        return cls(x=float(d["x"]), y=float(d["y"]))


@dataclass
class Target:
    """A detected object of interest (e.g. from YOLO), located in the map frame.

    Fields:
        x, y: map-frame position of the detection, meters.
        label: class name, e.g. "person", "door".
        confidence: detection score, 0..1.
    """

    x: float
    y: float
    label: str
    confidence: float = 1.0

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "label": self.label, "confidence": self.confidence}

    @classmethod
    def from_dict(cls, d: dict) -> "Target":
        return cls(
            x=float(d["x"]),
            y=float(d["y"]),
            label=str(d["label"]),
            confidence=float(d.get("confidence", 1.0)),
        )


@dataclass
class MapUpdate:
    """Full map payload pushed to the phone over the websocket.

    Fields:
        width, height: grid size in cells.
        resolution_m: cell edge length, meters per cell.
        origin: map-frame coordinate {x, y} of cell (0, 0), the grid lower-left corner.
        cells: row-major int list of length width*height. -1 unknown, 0 free, 100 occupied.
        pose: the robot's current Pose.
        return_path: ordered list of Waypoint, empty until a return is requested.
        targets: detected objects of interest (e.g. from YOLO), empty until something is
                 found. Each accumulates once detected.
    """

    width: int
    height: int
    resolution_m: float
    origin: dict  # {"x": float, "y": float}
    cells: List[int]
    pose: Pose
    return_path: List[Waypoint] = field(default_factory=list)
    targets: List[Target] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "type": "MapUpdate",
            "width": self.width,
            "height": self.height,
            "resolution_m": self.resolution_m,
            "origin": {"x": self.origin["x"], "y": self.origin["y"]},
            "cells": list(self.cells),
            "pose": self.pose.to_dict(),
            "return_path": [w.to_dict() for w in self.return_path],
            "targets": [t.to_dict() for t in self.targets],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "MapUpdate":
        return cls(
            width=int(d["width"]),
            height=int(d["height"]),
            resolution_m=float(d["resolution_m"]),
            origin={"x": float(d["origin"]["x"]), "y": float(d["origin"]["y"])},
            cells=list(d["cells"]),
            pose=Pose.from_dict(d["pose"]),
            return_path=[Waypoint.from_dict(w) for w in d.get("return_path", [])],
            targets=[Target.from_dict(t) for t in d.get("targets", [])],
        )
