# Recon Rover

An autonomous-return ground rover. A human drives it into an unknown space while it
builds a metric map on the edge. Then, with one command, it computes a route over the
map it just built and drives itself back to the start, with no network required.

A phone connects to the rover's own Wi-Fi hotspot to watch the map fill in live. A dense
3D reconstruction is an optional bonus tier, built later, only when a network link exists.

## System at a glance

```
                       +-------------------------------------------+
                       |              Arduino UNO Q                 |
                       |        (Qualcomm Dragonwing, Debian)       |
   USB webcam (mono) --+--> [ slam ] --pose--> [ mapping ]          |
                       |        ^                   |               |
   Modulino IMU  ------+--I2C-->|                   v               |
   Modulino ToF  ------+--I2C-->                [ planning ]        |
                       |                            |               |
                       |   [ server: FastAPI + websocket ]  <-------+--- map updates
                       |                            |               |
                       +----------------------------|---------------+
                          USB serial (ASCII)        |   Wi-Fi hotspot (AP)
                                |                    |        |
                                v                    |        v
                  +-----------------------------+    |   +---------------------+
                  |   Elegoo Smart Car V4       |    |   |  Phone browser      |
                  |   Arduino UNO R3 + TB6612   |    |   |  (visualization)    |
                  |   motors, ultrasonic, line  |    |   |  Three.js / canvas  |
                  +-----------------------------+    |   +---------------------+
                                                     |
                                          (visualization is served
                                           statically from the rover)
```

## The three workstreams

The repo is split into three parallel folders, each owned by a teammate:

| Folder | Role | Runs on | Stack |
| --- | --- | --- | --- |
| [car-firmware/](car-firmware/) | Drive motors, read ultrasonic and line sensors, speak serial | Elegoo UNO R3 | Arduino C++ |
| [navigation/](navigation/) | SLAM, mapping, planning, bridge, web server | UNO Q Dragonwing (Debian) | Python |
| [visualization/](visualization/) | Live map viewer in the phone browser | Phone browser, served from rover | JS + Three.js |

Supporting folders: [docs/](docs/) (architecture + the message contract) and
[shared/schemas/](shared/schemas/) (the message contract as JSON Schema, the single
source of truth reused by all three components).

## Data flow walkthrough

1. The human teleoperates the rover. The brain (UNO Q) streams `DriveCommand` messages
   over USB serial to the car (UNO R3), which turns them into motor outputs.
2. The car streams `CarTelemetry` back (ultrasonic distance, line and bumper flags).
3. The mono webcam frames plus `ImuSample` data from the Modulino IMU feed the SLAM
   frontend, which emits a `Pose` per frame.
4. `mapping` fuses each `Pose` with range data into an occupancy grid.
5. On the return command, `planning` runs A* over that grid to the logged start, with a
   reverse-of-the-driven-path fallback, and produces a list of waypoints.
6. The `server` packs the grid, current pose, and planned path into a `MapUpdate` and
   pushes it over a websocket to any phone connected to the rover's Wi-Fi AP.
7. `visualization` renders the grid and path live in the browser.

The message contract is defined once in [docs/message-schemas.md](docs/message-schemas.md)
and [shared/schemas/](shared/schemas/). Field names are identical across the firmware,
Python, and JS so the contract is literally one definition reused everywhere.

## Quickstart

```bash
# 1. Navigation brain (on the UNO Q Dragonwing, or a dev laptop for stubs)
cd navigation
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
python main.py            # runs the orchestrator against the stubs

# 2. Car firmware (flash to the Elegoo UNO R3)
#    Open car-firmware/car-firmware.ino in the Arduino IDE and upload.

# 3. Visualization (served from the rover, or open locally for dev)
cd visualization
# Plain static: open index.html in a browser. See its README for the CDN vs bundler note.
```

See each folder's README for the full build and run details.

## Scope

This repository is scaffolding. The hard algorithms (SLAM, path-planning math, dense 3D
reconstruction) are intentionally left as documented interface stubs with `TODO` markers
so the team can fill them in. Prefer clean, documented interfaces over working logic.

## License

MIT. See [LICENSE](LICENSE).
