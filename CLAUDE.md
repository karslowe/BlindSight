# Recon Rover — context for Claude Code

A **fully autonomous exploratory ground rover**. It drives itself into an unknown indoor
space (frontier exploration, no human driving), builds a 2D occupancy map on the edge with
no network, then computes a **direct route home** and drives back to the start. A phone on
the rover's Wi-Fi watches the map fill in live. Optional **YOLO** tier: detect a target
object during exploration and mark it. See `docs/HANDOFF.md` for the full state, and
`docs/architecture.md` for the why.

## Architecture (hardware)

- **Single Arduino UNO Q** = the whole rover computer. Dragonwing (Debian/Python) = brain
  (mapping, planning, web server, Wi-Fi AP). Onboard STM32 = motor controller driving the
  Elegoo shield. (Original Elegoo UNO R3 was lost; STM32 or a replacement R3 fills that role.)
- **iPhone/iPad Pro** = perception: camera + LiDAR + IMU -> ARKit -> streams pose + depth to
  the UNO Q via Record3D. This is the pose authority (SLAM is offloaded to the phone).
- **Elegoo Smart Car V4** = motors + ultrasonic + line sensors. **3x Modulino ToF** = side
  coverage. UNO Q GPIO is 3.3V; the car's 5V sensor lines need level-shifting.

## Three workstreams (folders)

- `navigation/` — Python brain (UNO Q). Mapping, exploration, planning, serial link, server.
- `car-firmware/` — Arduino C++ on the motor MCU. Parses `DRV`, drives motors, sends `TEL`.
- `visualization/` — static web viewer, served from the rover.
- `shared/schemas/` + `docs/message-schemas.md` — the message contract.

## The contract rule (do not break)

All cross-component messages are defined ONCE in `docs/message-schemas.md` and mirrored in
`shared/schemas/schemas.py` (Python), `shared/schemas/*.schema.json`, and used verbatim in
the firmware (C++) and `visualization/` (JS). **Field names are identical across all three
languages.** Change `docs/message-schemas.md` first, then mirror everywhere. Messages:
`DriveCommand`, `CarTelemetry`, `Pose`, `ImuSample`, `MapUpdate` (embeds `Pose`, `Waypoint`,
`Target`, and a `start`).

## Run the demos (Mac, no hardware)

```bash
cd navigation && python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt
python server/autonomy_demo.py     # FULL autonomy (explore->map->find target->return) -> http://localhost:8000/?live
python bridge/fake_car.py          # virtual serial car (DRV in / TEL out); then in another shell:
python bridge/drive_test.py --port <PTY path it printed>   # stream drive commands
```
Demo tunables live at the top of `server/autonomy_demo.py`: `TARGET`, `DETECT_RANGE_M`,
`RETURN_ON_TARGET`, `EXPLORE_BUDGET_TICKS`.

## Verifying changes

The hard logic (grid, explorer, planner) is pure Python + numpy and is tested headlessly:
spin up a throwaway venv with numpy, import `server.autonomy_demo.step` + `_State`, run the
loop, assert on the result. Search the git log for the test snippets used per feature.

## Gotchas that have bitten us

- **Restart the server after Python changes** (`Ctrl-C`, `lsof -ti:8000 | xargs kill -9`,
  re-run). A browser refresh does NOT reload the Python server.
- **Hard-refresh the browser** (`Cmd-Shift-R`) for JS changes; `index.html` has a `?v=N`
  cache-bust on the script tags - bump it when editing `viewer.js`/`ws_client.js`.
- **Sim vs real:** `autonomy_demo.py` moves a kinematic rover with a synthetic sensor model -
  it tests the BRAIN's decisions, not motor code. The real motor path is `car_link.py` +
  `car-firmware/` (shares the same `DriveCommand`). `fake_car.py` is a different stand-in
  (the serial car), not the autonomy sim.
- **Return is a DIRECT A\* route home, by design** (not retracing the breadcrumb). The dotted
  trail (where it drove) and the green route home are intentionally different.
- **The START marker comes from `MapUpdate.start`** (authoritative), not a guess.
- **Firmware** has the real Elegoo V4 pins filled in and verified; it still needs flashing +
  bench calibration (invert/swap, line threshold, track width, m/s->PWM) and 5V level-shifting
  on the UNO Q. See `car-firmware/README.md`.

## What's stubbed (the real remaining work)

- `bridge/phone_link.py` + `slam/slam_frontend.py`: Record3D wrapper written but UNVERIFIED on
  a device (getter names + camera-transform->2D-Pose need calibration). Needs the phone +
  Record3D streaming bring-up (the load-bearing unknown).
- Live phone->scan wiring in `main.py` (ToF + depth-wedge lines are commented TODOs).
- `perception/detector.py::detection_to_world` (project a YOLO box to map coords).
- `bridge/modulino_io.py` (I2C IMU/ToF) - mostly redundant now that the phone provides IMU.

## Git

Everyone has been committing to `main`, which already caused a divergence (a teammate's
"removed nav" commit collided with active work). Prefer a branch + small PR per change.
