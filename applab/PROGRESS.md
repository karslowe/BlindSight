# BlindSight — App Lab deployment progress

_Last updated: 2026-05-31. Board: Arduino UNO Q (Qualcomm Dragonwing, Debian Trixie)._

## TL;DR

The architecture question is resolved (**Branch B**) and the full data path is built and
validated on **live phone data**: iPhone → native host capture → occupancy map served inside
App Lab. The only blocked item is the **geometry calibration**, which needs the phone
physically mounted on the rover. Everything else below can proceed in the meantime.

## Architecture (decided, evidence-backed)

record3d (iPhone depth+pose) **cannot run in the App Lab container** — no C toolchain to build
it, and the container doesn't get the iPhone's USB node. So we split (Branch B):

```
iPhone ──USB──> HOST: ~/lidar_feed.py            APP LAB CONTAINER: edge/python/main.py
                 record3d capture                  pulls /data over docker gateway
                 depth → PNG feed   :8008          OccupancyGrid mapping
                 pose + scan        /data   ─────> renders top-down map        :8000
                 (systemd user service)            (pure stdlib + numpy wheel)
```

## Done & validated

| Item | Status | Evidence |
|---|---|---|
| Clone BlindSight, `edge` branch | ✅ | `~/BlindSight` (edge = with_nav + docs) |
| Branch A vs B determination | ✅ | container: no gcc/cmake, no iPhone USB node |
| record3d builds on host | ✅ | needs `cmake`+`libusb-1.0-0-dev` + `CXXFLAGS="-include cstdint -include string"` (GCC-14 fix) |
| Native capture + depth feed | ✅ | `~/lidar_feed.py` :8008, ~30 fps, 256×192 depth |
| Latency fix | ✅ | decoupled encode (newest-frame-only, on-demand); ~40 ms→~11 ms full-res |
| Auto-start | ✅* | systemd **user** service `lidar-feed`; *needs `sudo loginctl enable-linger arduino` once |
| App Lab relay (step 2) | ✅ | pure-stdlib proxy, reachable via bridge gateway |
| Brain: pose + scan at `/data` | ✅ | live values sane (90 rays, 0.33–4 m, stable pose) |
| Brain: OccupancyGrid map on :8000 | ✅ | 30 folds/5 s on live data → grid grew to 75×102 |
| Autonomy: explore + return-home | ✅ | closed-loop sim: explore → no frontiers → return → home @0.05 m |
| Car Bridge (Python side) | ✅ | `Bridge.call("drive",…)` + `Bridge.provide("car_telemetry",…)`; API confirmed vs Arduino examples |
| Car Bridge (sketch.ino) | ⚠️ | written (Bridge + diff-drive + watchdog); **motor/sensor pins TODO, not compiled/bench-tested** |

## Pending

### Needs the phone mount (do on mount day)
- **Geometry calibration (TASK B).** Mount the phone, deploy, walk it straight forward ~1 m,
  observe which way the rover marker moves vs. reality. Then fix the axis mapping in
  `lidar_feed.py`: `_pose_from_camera` (currently `x=tz, y=tx, θ=yaw`) and the angle sign in
  `_depth_to_scan`. Until done, the map may be rotated/mirrored/scaled.

### Can do now (mount-independent)
1. **Deploy the brain in-container & confirm it runs.** App is registered (`user:edge`).
   Deploy via the App Lab GUI or `arduino-app-cli app start user:edge`. ⚠️ This flashes the
   sketch and lets the brain command motors over the Bridge — **put the car on blocks** until
   geometry + pins are verified. The map will build (possibly mis-oriented pre-calibration).
2. **Car Bridge — bench verification (sketch side).** ✅ DONE in code (Python both directions;
   `sketch/sketch.ino` has the Bridge wiring + differential-drive + watchdog). REMAINING: fill
   the real motor/sensor pins (UNO Q → Elegoo TB6612 + HC-SR04), compile, and confirm a "drive"
   command actually moves the wheels and telemetry arrives. Diff-drive math from `~/BlindSight/
   car-firmware`.
3. **Planning / autonomy.** ✅ DONE + validated (closed-loop sim). Lives in `python/navigator.py`
   (explore + return-home + A*); map renders the planned path + pose trail; "Return home" button
   on the :8000 page + `POST /return`.
4. **Full interactive viewer.** Swap the rendered PNG for `navigation/visualization` (real map
   UI). Needs a polling or websocket adapter in the container.
5. **Calibration prep.** Add a record/replay of `/data` to a file so mount-day calibration is
   instant.
6. **Side ToF sensors.** Implement `navigation/bridge/modulino_io.py` `connect()`/`read` (3×
   Modulino Distance over Qwiic/I2C) — currently a `NotImplementedError` stub. Needs hardware.

## Ops notes
- Host feed: `systemctl --user {status,restart} lidar-feed` · logs `journalctl --user -u lidar-feed -f`.
- For cold-boot-without-login, run once: `sudo loginctl enable-linger arduino`.
- Host feed binds `0.0.0.0:8008` (reachable on the LAN directly, not only via App Lab). Can be
  locked to the docker bridge only if desired.
- App Lab brain: open `http://192.168.137.252:8000` after deploy. Override host URL with env
  `BLINDSIGHT_FEED_URL` if the bridge gateway differs.
