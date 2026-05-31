# App Lab deployment (Branch B)

The App Lab deployment of the BlindSight navigation brain on the Arduino UNO Q. record3d
(iPhone depth + ARKit pose) can't run in the App Lab container — no C toolchain, and the
container doesn't get the iPhone's USB node — so capture runs natively on the host and the
App Lab app is the brain. See `PROGRESS.md` for the full status and evidence.

```
iPhone ──USB──> HOST: lidar_feed.py              APP LAB CONTAINER: edge/
                record3d capture                  edge/python/main.py — brain
                depth → PNG feed   :8008          pulls /data over the docker gateway
                pose + scan        /data   ─────> OccupancyGrid + explore/return autonomy
                (lidar-feed.service)               drives the car over the App Lab Bridge
                                                   live map + Return-home UI    :8000
```

## Contents
- `lidar_feed.py` — host capture: record3d → depth PNG feed (`:8008`) + brain-facing
  `/data` (2D pose + horizontal scan). Runs as the `lidar-feed` systemd **user** service.
- `lidar-feed.service` — that systemd unit (`~/.config/systemd/user/`). Cold-boot needs
  `sudo loginctl enable-linger arduino` once.
- `edge/` — the App Lab project:
  - `python/main.py` — brain: pulls `/data`, builds the map, runs autonomy, renders `:8000`,
    sends drive over the Bridge (`Bridge.call("drive", …)`), receives `car_telemetry`.
  - `python/navigator.py` — autonomy core (FrontierExplorer + ReturnPlanner + A*).
  - `python/requirements.txt` — `numpy` only (unpinned → aarch64 wheel; no record3d).
  - `sketch/sketch.ino` — UNO Q MCU: App Lab Bridge ↔ Elegoo V4 motors/sensors.

> The deployed `edge/python/` also vendors `navigation/`, `shared/`, and `visualization/`
> copied from the repo root — those are NOT duplicated here; copy them in at deploy time:
> `cp -r navigation shared visualization <applab>/edge/python/`.

## Run
1. Host: `~/r3d-venv/bin/python lidar_feed.py` (or the `lidar-feed` service) + stream from
   the Record3D iOS app (USB Streaming → live).
2. App Lab: deploy `edge/` (GUI Run or `arduino-app-cli app start user:edge`), open
   `http://<board>:8000`. ⚠️ Deploying flashes the MCU and lets the brain command motors —
   put the car on blocks until geometry (TASK B) and the sketch pins (TASK C) are verified.
