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
                 record3d capture                  pulls /data over docker bridge
                 depth → PNG feed   :8008          OccupancyGrid + explore/return autonomy
                 pose + scan        /data   ─────> MapUpdate JSON + real viewer   :8000
                 (user service, bound to            (browser renders the map; drive → car
                  docker bridges only)               over the App Lab Bridge)
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
| Brain: OccupancyGrid mapping | ✅ | 30 folds/5 s on live data → grid grew to 75×102 |
| Autonomy: explore + return-home | ✅ | closed-loop sim: explore → no frontiers → return → home @0.05 m |
| Map rendered CLIENT-SIDE | ✅ | brain serves MapUpdate JSON at `/mapupdate`; real `visualization/` viewer renders it (no server raster) |
| Car Bridge (Python) | ✅ | `Bridge.call("drive",…)`; API confirmed vs Arduino examples |
| Car Bridge (sketch.ino) | ⚠️ | drive-only, real V4 pins (single dir-pin/motor) + watchdog; **needs compile + bench pass** |
| Ultrasonic | ❌ removed | no time to level-shift 5 V→3.3 V; sketch→brain telemetry dropped with it |
| Host-feed LAN lockdown | ✅ | `lidar_feed.py` binds docker bridges only (172.x); LAN IP refuses (verified) |

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
2. **Car Bridge — bench pass (sketch).** ✅ Code done (Python `Bridge.call("drive",…)`;
   `sketch/sketch.ino` is drive-only with the real Elegoo V4 pins from `car-firmware` —
   single direction pin per motor — + watchdog). REMAINING: compile/flash and confirm a
   `drive` command spins the wheels the right way (flip `*_INVERT`/`SWAP`). TB6612 logic is
   fine at 3.3 V.
3. **Planning / autonomy.** ✅ DONE + validated (closed-loop sim). `python/navigator.py`
   (explore + return-home + A*); "Return home" button on :8000 + `POST /return`.
4. **Full interactive viewer.** ✅ DONE — serves the real `navigation/visualization` viewer;
   transport swapped from websocket to polling `/mapupdate` (`src/ws_client.js`), rendering
   is client-side.
5. **Calibration prep.** Add a record/replay of `/data` so mount-day calibration is instant.
6. **Side ToF sensors.** Implement `navigation/bridge/modulino_io.py` `connect()`/`read` (3×
   Modulino Distance over Qwiic/I2C) — currently a `NotImplementedError` stub. Needs hardware.

## Ops notes
- Host feed: `systemctl --user {status,restart} lidar-feed` · logs `journalctl --user -u lidar-feed -f`.
- **Run once for cold-boot survival:** `sudo loginctl enable-linger arduino` (can't be done
  from a no-password-sudo shell — the single most likely demo-day failure if skipped).
- Host feed `:8008` binds the docker bridges only (locked off the LAN). The brain `:8000` binds
  `0.0.0.0` *inside the container* (container-internal; App Lab maps it to the host for the demo
  UI — intended). Override the feed host with env `BLINDSIGHT_FEED_URL` if needed.
- Deploy: App Lab GUI Run or `arduino-app-cli app start user:edge`. ⚠️ flashes the MCU and lets
  the brain command motors — **put the car on blocks** until geometry + sketch pins are verified.
- View: `http://10.161.73.225:8000`.
