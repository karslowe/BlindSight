# Task brief — App Lab DEPLOY & VERIFY (SSH Claude instance)

You are in an **SSH VS Code session on the Arduino UNO Q** (Dragonwing, Debian Linux). Your
job is to **deploy and verify the EXISTING navigation pipeline on real hardware** — NOT to
build it. Most of it is already implemented on the `with_nav` branch. Read this whole file.

## ⚠️ FIRST: work off the `with_nav` branch, not `main`

The real implementation lives on **`with_nav`** (22 commits ahead of `main`; `main` is
stale). Before anything:

```bash
git fetch --all
git checkout with_nav      # or: git checkout -b work with_nav
```

Do NOT build features against `main` — you will duplicate code that already exists and
create merge pain. Everything below assumes you are on `with_nav`.

## What is ALREADY BUILT on `with_nav` (do NOT rebuild)

- **`mapping/occupancy_grid.py`** — generalized to the batched **scan contract**
  `update(pose, scan)` where `scan = [(angle_offset_rad, range_m), ...]`. Log-odds, growable,
  sensor-agnostic. (This was "Step 1" — done.)
- **`bridge/phone_link.py`** — the Record3D wrapper. Connects, reads depth + intrinsics +
  **camera pose (quaternion + translation)**, reshapes to our 2D `Pose`. Confirms pose IS
  available from Record3D. Non-blocking (stale-frame-safe).
- **`slam/slam_frontend.py`** — thin wrapper emitting `Pose` (no SLAM computed; ARKit owns it).
- **`planning/pathfind.py`** (A*), **`planning/explorer.py`** (frontier exploration),
  **`planning/return_planner.py`** — autonomous explore-then-return, verified against a
  synthetic room (`server/autonomy_demo.py`).
- **`perception/detector.py`** — YOLO "find a target" mission (ultralytics).
- **Schemas** — `MapUpdate` already carries a `targets` list of `Target{x,y,label,confidence}`
  (the semantic/object-pin layer — already in the contract).
- **`main.py`** — orchestrator wiring all of the above, with `TOF_ANGLES = [-1.22, 1.22,
  1.57]` (−70/+70/+90°) and the battery failsafe.
- **Firmware** — real Elegoo V4 pins filled in; viz updated.
- **Docs** — `navigation/bridge/AGENT.MD` is the canonical plan doc; `CLAUDE.md`,
  `docs/HANDOFF.md` exist.

So the pipeline EXISTS and is verified in simulation. The remaining work is **making it run
on the real Dragonwing + phone + sensors + car**, and the App Lab deployment question.

## TASK A — App Lab deployment: where does the brain run?

We are **defaulting to App Lab** (reuse the `web_ui` brick for the viz + the Bridge to the
MCU + the official deploy path). The one open risk is whether the App Lab **container** can
reach the iPhone over usbmuxd.

**What we know (from our CarbinWatcher App Lab project):** the container CAN open a USB
**webcam** via `cv2.VideoCapture` (App Lab brokers `/dev/video*`), and deps persist by
self-`pip install -r requirements.txt` at the top of `main.py`. BUT the iPhone via record3d
is a raw USB device via **usbmuxd + `/dev/bus/usb`** — a different class — so `/dev/video`
working does NOT prove the iPhone works in-container.

1. Confirm at the **SSH host shell**: `lsusb | grep -i 05ac` and `idevice_id -l` (UDID).
   (Should be true — see Confirmed state.)
2. From **inside an App Lab `App.run()` context**, call
   `Record3DStream.get_connected_devices()`. Does it see the iPhone?
   - **YES → BRANCH A:** run the whole brain in App Lab. Cleanest.
   - **NO → BRANCH B:** keep brain logic, web_ui, Bridge in App Lab, but split out a tiny
     **native-Linux** process that owns the iPhone record3d stream (full /dev access) and
     forwards pose+depth to the App Lab app over a local socket. Carve out ONLY the iPhone
     capture; do NOT move the whole brain native. Also check if this firmware's `app.yaml`
     supports a `/dev/bus/usb` passthrough (would restore Branch A).

**Deliverable:** Branch A or B, and if B the IPC mechanism chosen.

## TASK B — resolve the `phone_link.py` hardware TODOs (needs the real phone)

`bridge/phone_link.py` is written but has TODOs only the physical phone can close:
1. **Verify the record3d getter names/shapes** against the INSTALLED `record3d` version
   (`get_depth_frame` / `get_intrinsic_mat` / `get_camera_pose` and the `cam.qx..tz` fields).
   Run a probe; fix any name mismatches.
2. **Calibrate the axis mapping** in `_pose_from_camera()`. It currently assumes
   `x_map = tz, y_map = tx, theta = yaw about Y` for an upright forward-facing phone. Drive a
   known path on the bench and confirm the emitted `Pose` matches reality for how the phone
   ACTUALLY mounts on the rover. This is the one calibration that, if wrong, smears the map.

## TASK C — wire the real sensors + car, then integrate

1. **3 ToF** via `bridge/modulino_io.py` (3× Modulino Distance, Qwiic-chained). Confirm
   `TOF_ANGLES` order matches the physical chain order (mirrored map = swapped order).
2. **Car** over the **Bridge** (App Lab), not USB serial — `Bridge.call` drive commands,
   `Bridge.provide` telemetry on the sketch side. Keep the watchdog (brake on command loss).
3. Run `main.py` end-to-end on the Dragonwing: phone pose+depth + ToF → grid → autonomous
   explore → return home → live map in the browser via the web_ui brick.

## Confirmed state (don't re-investigate — this was hard-won)

- iPhone enumerates over USB at the SSH shell: `lsusb` shows `05ac:12a8 Apple`;
  `idevice_id -l` returns the UDID; usbmuxd running; Trust accepted.
- **BOOT-LOOP GOTCHA:** the UNO Q boot-loops if the iPhone is attached at power-on (tries to
  boot off it). **ALWAYS boot with the iPhone UNPLUGGED, wait for Linux, THEN hot-plug.** Bake
  this into the demo runbook: power on → wait for boot → connect iPhone → start Record3D.
- usbmuxd is `static` (not auto-enabled): `sudo systemctl start usbmuxd`. If it logs
  `LIBUSB_ERROR_ACCESS`/errno=13 → unplug/replug the iPhone so udev re-applies permissions.
- Debian Trixie package names: `usbmuxd libimobiledevice-1.0-6 libimobiledevice-utils`.
- record3d on ARM may need: `build-essential cmake libusb-1.0-0-dev`.
- Record3D stream metadata seen live: intrinsics fx=fy≈430, principal point ≈(238,317),
  frame 480×640 portrait. (WiFi depth caps at 3 m; USB gives native range — USB won.)
- iPhone app: Record3D → Settings → enable **USB Streaming mode** → tap record.

## What NOT to do

- Do NOT build against `main` — use `with_nav`.
- Do NOT rebuild the grid, planner, phone_link, detector, or schemas — they exist on `with_nav`.
- Do NOT move the whole brain native — App Lab default; native only the iPhone capture, only
  if Task A shows the container can't see usbmuxd.
- Do NOT power on the board with the iPhone attached (boot loop).
- Do NOT fabricate/mock sensor data to "make it run" — if blocked, surface the blocker.

## Reference: our working App Lab template

CarbinWatcher (`…/Final/carbinwatch-final/Edge/carbinwatcher/`) shows the proven App Lab
patterns: requirements self-install at top of `main.py`; `App.run(user_loop=...)`; `WebUI` +
`expose_api` + `send_message` + MJPEG `/stream`; `Bridge.call(...)`; an `_ui_lock` serializing
web_ui sends (two threads calling send_message at once crashes the websocket pipeline).
`app.yaml`: `bricks: [arduino:web_ui: {}]`, `ports: [8888]`.

## Sources

- App Lab docs: https://docs.arduino.cc/software/app-lab/
- Bridge/structure: https://dronebotworkshop.com/arduino-app-lab/
- USB-in-container report: https://forum.arduino.cc/t/arduino-uno-q-app-lab-as-a-usb-host-to-an-uno-r3-python-environment-sees-0-devices/1433482
