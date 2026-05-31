# Recon Rover — Handoff

A complete snapshot of project scope, structure, the frozen message contract, and what is
implemented vs stubbed. `CLAUDE.md` (repo root) is the short version loaded into Claude Code
each session; this is the full reference.

## 1. Scope

A fully autonomous exploratory ground rover. It drives itself into an unknown indoor space
(frontier-based exploration, no human driving), builds a 2D metric occupancy map on the edge
with no network, and when exploration is complete (or on command, or a battery-time
failsafe) it computes a direct route home over the map it built and drives back to the
start. A phone connects to the rover's own Wi-Fi to watch the map fill in live. Optional
YOLO tier: detect a target object during exploration, mark it on the map, optionally
return-on-find.

Hardware: single **Arduino UNO Q** (Dragonwing/Linux = brain; onboard STM32 = motor
controller). **iPhone/iPad Pro** (ARKit + Record3D) = perception (pose + depth). **Elegoo
Smart Car V4** = motors + servo-mounted ultrasonic + 3 line sensors. **3x Modulino ToF** =
side coverage. No wheel encoders. UNO Q GPIO is 3.3V; the car's 5V sensor lines need
level-shifting.

## 2. Frozen message contract (exact fields)

SI units throughout (m, m/s, rad, rad/s, s). Source of truth: `docs/message-schemas.md` +
`shared/schemas/schemas.py`. Field names identical across C++/Python/JS.

- **DriveCommand** (brain->car; serial `DRV <linear_velocity> <angular_velocity> <stop>\n`):
  `linear_velocity` float m/s, `angular_velocity` float rad/s, `stop` int 0/1.
- **CarTelemetry** (car->brain; `TEL <ultrasonic_distance> <bumper> <line_left> <line_center>
  <line_right> <timestamp>\n`): `ultrasonic_distance` float m (-1 = no echo), `bumper` 0/1,
  `line_left`/`line_center`/`line_right` 0/1, `timestamp` float s.
- **Pose**: `x`,`y` float m, `theta` float rad (CCW from +x), `covariance` float[9] row-major
  3x3, `timestamp` float s.
- **ImuSample**: `accel` float[3] m/s^2, `gyro` float[3] rad/s, `timestamp` float s.
- **MapUpdate** (mapping->server->viz, JSON over websocket):
  - `width`,`height` int (cells), `resolution_m` float (m/cell),
  - `origin` {x,y} (map coord of cell (0,0), lower-left),
  - `cells` int[] row-major, length width*height, **row 0 = bottom**, values **-1 unknown /
    0 free / 100 occupied**,
  - `pose` Pose, `return_path` Waypoint[] (empty unless returning),
  - `targets` Target[] (empty until found), `start` {x,y} or null (true home).
  - Waypoint = {x,y}. Target = {x,y,label,confidence(default 1.0)}.

## 3. On-disk structure

```
docs/                architecture.md, message-schemas.md, HANDOFF.md
shared/schemas/       schemas.py (Python), *.schema.json, README.md
car-firmware/         car-firmware.ino, src/{motor_control,serial_protocol,sensors}.{h,cpp}
navigation/
  main.py             orchestrator (explore->map->return loop)
  slam/slam_frontend.py        phone (Record3D) -> Pose + depth
  mapping/occupancy_grid.py    scan->grid, frontiers, A* helpers, MapUpdate
  planning/pathfind.py         shared A* (inflation, wall-cost, tiers)
  planning/explorer.py         frontier exploration -> next_path
  planning/return_planner.py   direct A* home + follower + breadcrumb fallback
  perception/detector.py       YOLO detect() + world-projection stub
  bridge/car_link.py           pyserial DRV/TEL (implemented)
  bridge/phone_link.py         Record3D wrapper (needs device verify)
  bridge/modulino_io.py        I2C IMU/ToF (STUB)
  bridge/fake_car.py           virtual serial car (PTY)
  bridge/drive_test.py         scripted DRV sender
  bridge/AGENT.MD              teammate build-plan doc
  server/app.py                FastAPI MapServer (broadcast + static viz + ReturnHome)
  server/autonomy_demo.py      FULL autonomy demo (sim)
  server/grid_demo.py          synthetic scans -> grid -> viz
  server/demo_broadcast.py     server + synthetic MapUpdates
visualization/        index.html, src/{viewer,ws_client}.js, dev/fake_map.js
```

## 4. Implemented vs stubbed

**Implemented + tested (in software):** occupancy mapping (scan contract, frontiers,
inflation, wall-cost), autonomous exploration, A* pathfinding (tiered, centered), direct
return planning + follower, the orchestrator wiring, the pyserial car link (verified vs the
fake car over a PTY), the FastAPI server (verified live), the 2D visualization (cells/trail/
route/start/targets, pan/zoom), YOLO `detect()`, and all the dev tools/demos. Firmware logic
is complete with the **verified Elegoo V4 pins** (motors 5/7/6/8/3, ultrasonic 13/12, line
A2/A1/A0).

**Stubbed / not built:**
- `bridge/modulino_io.py` (`connect`/`read_imu`/`read_distance`) - `NotImplementedError`.
- `perception/detector.py::detection_to_world` - `NotImplementedError`.
- `bridge/phone_link.py` + `slam/slam_frontend.py` - written but unverified on a real device
  (Record3D getter names + camera-transform->2D-Pose calibration). Needs the phone + the
  Record3D USB-stream bring-up (the load-bearing unknown).
- Live phone->scan integration in `main.py` (ToF + depth-wedge lines are commented TODOs;
  only an ultrasonic fallback ray is active).
- Firmware not flashed/calibrated: confirm pins in the official `DeviceDriverSet_xxx0.h`,
  then tune invert/swap, line threshold/polarity, `TRACK_WIDTH_M`, `MAX_WHEEL_SPEED_MPS`,
  and level-shift the 5V sensor lines on the UNO Q.

## 5. Demos / entry points

```
python server/autonomy_demo.py      # full autonomy, open http://localhost:8000/?live
python server/grid_demo.py          # synthetic scans -> real grid -> viz
python server/demo_broadcast.py     # real server + synthetic MapUpdates
python bridge/fake_car.py           # virtual serial car; then drive_test.py against it
```

## 6. Behaviors worth knowing

- Explores ALL reachable free space; occluded regions stay unknown, so coverage plateaus
  below 100%, then it returns. Returns DIRECTLY (not retracing). The dotted trail (where it
  drove) and the green route home differ on purpose.
- The viewer's START marker comes from `MapUpdate.start`, not a guess.
- `return_path`/the green line only appear while returning.
