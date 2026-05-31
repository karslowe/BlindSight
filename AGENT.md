# AGENT.md (root) — POINTER + iPhone USB bring-up runbook

> **The canonical navigation plan is `navigation/bridge/AGENT.MD` on the `with_nav` branch.**
> This root file is NOT a second source of truth — it only carries the one thing not captured
> there: the hard-won iPhone-USB bring-up runbook below. For the architecture, scan contract,
> TOF angles, and build steps, read `navigation/bridge/AGENT.MD`.

## Status (read this first)

- The navigation pipeline is **already implemented and verified in simulation on the
  `with_nav` branch** (grid scan-contract, `phone_link.py` Record3D wrapper, A* + frontier
  exploration, YOLO target detection, `MapUpdate.targets`). `main` is stale — **work off
  `with_nav`.**
- Remaining work is **hardware deploy + verification on the Dragonwing**, and the **App Lab
  deployment** question. That work is briefed in **`APPLAB_TASK.md`** (for the SSH instance on
  the board).
- `phone_link.py` confirms **Record3D exposes camera pose (quaternion + translation)** → the
  "ARKit owns pose" architecture holds.

## iPhone USB bring-up runbook (DONE — recorded so nobody re-suffers it)

The transport took a while to crack. The whole problem was **boot order**: the UNO Q
boot-loops if the iPhone is attached at power-on (it tries to boot off the phone). The fix and
the full bring-up, in order:

1. **Boot the UNO Q with the iPhone UNPLUGGED.** Wait for the Linux prompt. ← THE KEY STEP.
   Demo runbook: power on → wait for boot → THEN connect the iPhone → start Record3D.
2. **Hot-plug the iPhone** after boot. Confirm: `lsusb | grep -i 05ac` shows the Apple device.
3. **usbmuxd + libimobiledevice** (Debian Trixie package names):
   `sudo apt install -y usbmuxd libimobiledevice-1.0-6 libimobiledevice-utils`
4. **Start usbmuxd** (it's `static`, not auto-enabled): `sudo systemctl start usbmuxd`.
   If it logs `LIBUSB_ERROR_ACCESS` / errno=13 on the device node → **unplug/replug the
   iPhone** so udev re-applies permissions, then it works.
5. **Unlock the iPhone, accept "Trust This Computer"** (enter passcode) on replug.
6. **Verify:** `idevice_id -l` prints the UDID = data path fully open.
7. **Library (in a venv):** `pip install record3d numpy`. If the C++ build fails on ARM:
   `sudo apt install -y build-essential cmake libusb-1.0-0-dev` then retry.
8. **iPhone app:** Record3D → Settings → enable **USB Streaming mode** → tap record.

Confirmed stream metadata (from the live device): intrinsics `K` with fx=fy≈430, principal
point ≈ (238, 317), frame **480×640** portrait. USB beat WiFi (WiFi caps depth at 3 m and
needs the paid WebRTC path; USB is free via the `record3d` Python lib and lower-latency).

## Open hardware items to close

- **+90° side ToF beam:** which side (left/right)? Put it on the *open* side of the demo space.
- **Measured TOF angles** replace the nominal `[-1.22, 1.22, 1.57]` once mounted (match the
  Qwiic chain order, or the map mirrors).
- **`phone_link.py` axis calibration:** confirm `_pose_from_camera()` mapping for how the
  phone physically mounts (drive a known path, check the emitted Pose).
