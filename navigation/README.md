# navigation

Role: the autonomous brain. Runs SLAM, mapping, return planning, the bridge to the car and
sensors, and the web server that streams the map to the phone.

Runs on: the Arduino UNO Q, Qualcomm Dragonwing side, under Debian Linux. Pure Python.

Tech stack: Python 3. OpenCV and NumPy for vision and grids, pyserial for the car link,
FastAPI plus uvicorn and websockets for the server. The SLAM and IMU libraries are left as
a deliberate choice for later (see below).

## Build and run

```bash
cd navigation
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
python main.py
```

`main.py` is the orchestrator. Out of the box it wires the stubbed modules into the run
loop so the shape of the system is runnable and inspectable before the algorithms exist.

## Layout

```
navigation/
├── main.py                  orchestrator: wires modules into the run loop
├── slam/slam_frontend.py    frame + IMU -> Pose + keyframe   (interface stub)
├── mapping/occupancy_grid.py  Pose + range -> occupancy grid (stub)
├── planning/return_planner.py A* to start, reverse-path fallback (stub)
├── perception/detector.py   optional object detection (stub)
├── bridge/car_link.py       pyserial link to the Elegoo UNO (stub)
├── bridge/modulino_io.py    read IMU + ToF over Qwiic / I2C (stub)
└── server/app.py            FastAPI + websocket, serves map to the phone (stub)
```

## Developing without hardware

Two dev simulators let you build and test the brain with no rover, no car, and no camera:

- `python server/demo_broadcast.py` runs the REAL server with synthetic map frames. Open
  http://localhost:8000/?live to watch the live map flow through the genuine websocket
  pipeline into the visualization.
- `python bridge/fake_car.py` opens a virtual serial port that behaves like the Elegoo
  car: it accepts `DRV` lines and streams back `TEL` lines from a simulated rover in a
  virtual room (ultrasonic shrinks as it nears a wall, bumper trips on contact, line
  sensors fire near an edge). Point `CarLink` at the printed device path to develop and
  test `bridge/car_link.py` against it.
- `python bridge/drive_test.py --port <path>` streams a scripted DRV sequence and prints
  the TEL coming back. The same tool drives the fake car (pass its printed PTY path) or
  the real Elegoo (pass `/dev/ttyACM0` once the firmware parses DRV). This is the bring-up
  test for the drive path and needs no SLAM, mapping, or brain logic.

The fake car and demo broadcast are dev-only; delete them once the real hardware is in
the loop.

## Message schemas

The contract lives in [../docs/message-schemas.md](../docs/message-schemas.md) and is
imported as dataclasses from [../shared/schemas/schemas.py](../shared/schemas/schemas.py).

- Consumes: `CarTelemetry` (from the car via `car_link`), `ImuSample` (from `modulino_io`),
  raw camera frames (from the webcam).
- Produces internally: `Pose` (slam -> mapping, planning).
- Produces outward: `DriveCommand` (to the car via `car_link`), `MapUpdate` (to the phone
  via `server`).

## The SLAM choice is deliberately open

[slam/slam_frontend.py](slam/slam_frontend.py) is a clean interface, not an
implementation. Candidate libraries to swap in later: RTAB-Map, ORB-SLAM3, OpenVINS, or
Kimera. Do not let the scaffold lock in a heavy framework.
