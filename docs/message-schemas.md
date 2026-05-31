# Message schemas: the contract

This document is the single source of truth for every message that crosses a boundary
between the three components. The same definitions are mirrored as machine-readable JSON
Schema in [../shared/schemas/](../shared/schemas/) and as typed Python dataclasses in
[../shared/schemas/schemas.py](../shared/schemas/schemas.py).

Rule: field names are identical across the firmware (C++), the brain (Python), and the
viewer (JS). Do not rename a field in one place only. Change it here first, then mirror it
everywhere.

Units are SI throughout: meters, meters per second, radians, radians per second, seconds.
All timestamps are `float` seconds since an epoch each component agrees on (monotonic
clock on the brain is fine for a hackathon; document it if you change it).

---

## 1. DriveCommand  (brain -> car)

Transport: USB serial, line-based ASCII so it is debuggable with a serial monitor. One
command per line, terminated by `\n`.

Wire format:

```
DRV <linear_velocity> <angular_velocity> <stop>
```

Example:

```
DRV 0.25 -0.40 0
DRV 0.00 0.00 1
```

| Field | Type | Unit | Notes |
| --- | --- | --- | --- |
| `linear_velocity` | float | m/s | Forward positive. |
| `angular_velocity` | float | rad/s | Counter-clockwise positive. |
| `stop` | int (0 or 1) | - | Optional hard stop. `1` overrides the velocities and brakes. |

JSON form (used internally on the brain and over the websocket where convenient):

```json
{ "type": "DriveCommand", "linear_velocity": 0.25, "angular_velocity": -0.40, "stop": 0 }
```

---

## 2. CarTelemetry  (car -> brain)

Transport: USB serial, line-based ASCII, one record per line.

Wire format:

```
TEL <ultrasonic_distance> <bumper> <line_left> <line_center> <line_right> <timestamp>
```

Example:

```
TEL 0.83 0 0 1 0 1234.567
```

| Field | Type | Unit | Notes |
| --- | --- | --- | --- |
| `ultrasonic_distance` | float | m | Forward range from the servo-mounted ultrasonic. `-1` if no echo. |
| `bumper` | int (0 or 1) | - | Bumper or stall flag. |
| `line_left` | int (0 or 1) | - | Line sensor, left. |
| `line_center` | int (0 or 1) | - | Line sensor, center. |
| `line_right` | int (0 or 1) | - | Line sensor, right. |
| `timestamp` | float | s | Car-side time of the reading. |

JSON form:

```json
{ "type": "CarTelemetry", "ultrasonic_distance": 0.83, "bumper": 0,
  "line_left": 0, "line_center": 1, "line_right": 0, "timestamp": 1234.567 }
```

---

## 3. Pose  (slam -> mapping, planning)

The estimated 2D pose of the rover in the map frame.

| Field | Type | Unit | Notes |
| --- | --- | --- | --- |
| `x` | float | m | Position in the map frame. |
| `y` | float | m | Position in the map frame. |
| `theta` | float | rad | Heading, counter-clockwise from the map x-axis. |
| `covariance` | float[9] | - | Row-major 3x3 covariance over (x, y, theta). |
| `timestamp` | float | s | Time of the estimate. |

JSON form:

```json
{ "type": "Pose", "x": 1.20, "y": -0.35, "theta": 0.78,
  "covariance": [0.01,0,0, 0,0.01,0, 0,0,0.02], "timestamp": 1234.570 }
```

---

## 4. ImuSample  (modulino -> slam)

A single visual-inertial IMU reading from the Modulino Movement sensor.

| Field | Type | Unit | Notes |
| --- | --- | --- | --- |
| `accel` | float[3] | m/s^2 | Linear acceleration (x, y, z). |
| `gyro` | float[3] | rad/s | Angular velocity (x, y, z). |
| `timestamp` | float | s | Time of the sample. |

JSON form:

```json
{ "type": "ImuSample", "accel": [0.02, -0.01, 9.79],
  "gyro": [0.001, 0.000, -0.002], "timestamp": 1234.560 }
```

---

## 5. MapUpdate  (mapping -> server -> viz)

The full payload pushed to the phone over the websocket. Carries the occupancy grid, the
robot's current pose, and the planned return path.

| Field | Type | Unit | Notes |
| --- | --- | --- | --- |
| `width` | int | cells | Grid width in cells. |
| `height` | int | cells | Grid height in cells. |
| `resolution_m` | float | m/cell | Edge length of one cell. |
| `origin` | object `{x, y}` | m | Map-frame coordinate of cell (0, 0), the grid's lower-left corner. |
| `cells` | int[] | - | Row-major, length `width * height`. Each cell: `-1` unknown, `0` free, `100` occupied. |
| `pose` | Pose | - | The robot's current pose (see schema 3). |
| `return_path` | Waypoint[] | - | Planned return route, ordered start of travel to goal. Empty until a return is requested. |
| `targets` | Target[] | - | Detected objects of interest (e.g. from YOLO), in the map frame. Empty until something is found. |
| `start` | object `{x, y}` or null | m | Map-frame position of the mission start (home), for the viewer's START marker. null until set. |
| `point_cloud` | float[] | m | Optional 3D point cloud for the 3D viz only (navigation never uses it). Flat `[x0,y0,z0, x1,y1,z1, ...]` in the map frame. Empty unless a depth source produces it. |

Where a `Waypoint` is:

| Field | Type | Unit | Notes |
| --- | --- | --- | --- |
| `x` | float | m | Map-frame position. |
| `y` | float | m | Map-frame position. |

And a `Target` is:

| Field | Type | Unit | Notes |
| --- | --- | --- | --- |
| `x` | float | m | Map-frame position of the detection. |
| `y` | float | m | Map-frame position of the detection. |
| `label` | string | - | Class name, e.g. "person", "door". |
| `confidence` | float | - | Detection score, 0..1. Optional, defaults to 1.0. |

JSON form:

```json
{
  "type": "MapUpdate",
  "width": 4, "height": 3, "resolution_m": 0.05,
  "origin": { "x": -1.0, "y": -1.0 },
  "cells": [-1,-1,0,0, 0,0,100,0, 0,0,0,-1],
  "pose": { "type": "Pose", "x": 0.1, "y": 0.0, "theta": 0.0,
            "covariance": [0,0,0,0,0,0,0,0,0], "timestamp": 1234.57 },
  "return_path": [ { "x": 0.1, "y": 0.0 }, { "x": 0.0, "y": 0.0 } ]
}
```

The cell encoding (`-1 / 0 / 100`) matches the common ROS occupancy-grid convention so it
is familiar and easy to swap in real tooling later.
