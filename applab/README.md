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
> from the repo root — copy them in at deploy time:
> `cp -r navigation shared visualization <applab>/edge/python/`.
> THEN overlay the two viewer files committed here — `edge/python/visualization/index.html`
> and `src/ws_client.js` — which swap the viewer's transport from the websocket (used by the
> old FastAPI server) to polling `/mapupdate` (the App Lab brain). Rendering stays in
> `viewer.js`, client-side.
>
> Note: no sketch→brain telemetry. The ultrasonic was removed (no time to level-shift its
> 5 V echo), so `sketch.ino` is drive-only and the brain maps purely from phone depth.

## Run
1. Host: `~/r3d-venv/bin/python lidar_feed.py` (or the `lidar-feed` service) + stream from
   the Record3D iOS app (USB Streaming → live).
2. App Lab: deploy `edge/` (GUI Run or `arduino-app-cli app start user:edge`), open
   `http://<board>:8000`. The brain starts **DISARMED** — it maps but does not drive.

## Control + calibration endpoints (on the brain, :8000)
- `POST /calibrate` — motor-driven pose calibration: drives a forward leg + a left turn,
  measures the raw ARKit-pose deltas, computes the phone→rover axis mapping, and POSTs it to
  the feed's `/calib`. Re-runnable any time (e.g. after a remount). Drives the rover.
- `POST /start` — ARM autonomy (rover begins exploring). `POST /stop` — disarm + brake (kill
  switch; app keeps mapping). `POST /return` — plan + drive home.
- `POST /drive?v=&w=&secs=` — phone-independent bench wiring test (spin the wheels directly).

Pose calibration lives in `~/blindsight_calib.json` on the host via the feed's `GET/POST
/calib` (keys: fwd, fwd_sign, lat, lat_sign, yaw_sign, scan_sign) — applied at runtime, no
code edit/restart. Manual fallback: `calibrate.py` (push the rover by hand; same math).

⚠️ Once armed (or calibrating), the rover drives — and it auto-explores an UNCALIBRATED map
straight into walls. Calibrate first, verify the map orientation, then `/start`.
