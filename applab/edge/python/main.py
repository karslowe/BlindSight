"""App Lab app: the navigation BRAIN (Branch B, steps 3-4).

Capture (record3d) runs natively on the host (~/lidar_feed.py) and exposes brain-facing data
at /data: 2D camera pose + a horizontal depth scan. THIS app is the brain:

  * pulls /data over the docker gateway,
  * folds (pose, scan) into the real OccupancyGrid (navigation/mapping),
  * runs autonomous frontier exploration + return-home planning (navigation/planning),
  * computes DriveCommands (sent to the car over the App Lab Bridge by the sketch side),
  * serves the live occupancy map on :8000 — RENDERED CLIENT-SIDE by the real browser viewer
    (navigation/visualization). The brain only ships MapUpdate JSON at /mapupdate; the
    Dragonwing does no map rasterization (that was a deliberate fix — keep render off the brain).

Deps: numpy only (prebuilt aarch64 wheel; never PIN it — see requirements.txt). The mapping,
planning, schema, and visualization assets are vendored under python/ already.

Drive output: the latest DriveCommand is published to the MCU via Bridge.call("drive", ...)
when the App Lab Bridge is available. (No telemetry comes back: the ultrasonic was removed —
no time to level-shift its 5 V echo — and nothing else on the car feeds the brain.)
"""

from __future__ import annotations

import json
import math
import mimetypes
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from arduino.app_utils import App

PORT = 8000
FEED_PORT = 8008
FETCH_TIMEOUT = 2.0

HERE = Path(__file__).resolve().parent
VIZ = HERE / "visualization"
for _p in (HERE / "navigation", HERE / "shared"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def _ensure_numpy() -> None:
    try:
        import numpy  # noqa: F401
        return
    except ImportError:
        pass
    print("[brain] installing numpy (wheel) ...", flush=True)
    subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy"])


_ensure_numpy()
from schemas.schemas import Pose  # noqa: E402
from navigator import Navigator  # noqa: E402  (autonomy core; sets up navigation/shared paths)

# Optional: the App Lab Bridge to the MCU. Imported lazily so the brain runs without it.
try:
    from arduino.app_utils import Bridge  # type: ignore
    _BRIDGE = True
except Exception:  # noqa: BLE001
    _BRIDGE = False

_lock = threading.Lock()        # guards _state (status fields)
_nav_lock = threading.Lock()    # guards the grid/planner (brain writes, /mapupdate reads)
_state = {"brain": "starting", "upstream": "unknown", "folds": 0,
          "grid_wh": None, "mode": "explore", "cmd": None, "base": None, "drive": "—",
          "calib": "idle"}

_nav = Navigator()
_last_pose = None  # most recent Pose (for building MapUpdate); guarded by _nav_lock


# ============================================================ host-feed discovery ====
def _default_gateway() -> "str | None":
    try:
        with open("/proc/net/route") as f:
            for line in f.readlines()[1:]:
                p = line.split()
                if len(p) > 2 and p[1] == "00000000":
                    return socket.inet_ntoa(struct.pack("<L", int(p[2], 16)))
    except Exception:  # noqa: BLE001
        pass
    return None


def _candidate_bases() -> "list[str]":
    bases = []
    env = os.environ.get("BLINDSIGHT_FEED_URL")
    if env:
        bases.append(env.rstrip("/"))
    gw = _default_gateway()
    if gw:
        bases.append(f"http://{gw}:{FEED_PORT}")
    for ip in ("172.18.0.1", "172.17.0.1"):
        u = f"http://{ip}:{FEED_PORT}"
        if u not in bases:
            bases.append(u)
    return bases


def _get(url: str) -> bytes:
    import urllib.request
    with urllib.request.urlopen(url, timeout=FETCH_TIMEOUT) as r:
        return r.read()


def _find_base() -> "str | None":
    for base in _candidate_bases():
        try:
            _get(f"{base}/status")
            return base
        except Exception:  # noqa: BLE001
            continue
    return None


# ============================================================ car bridge (drive only) =
def _send_drive(cmd) -> None:
    """Publish the DriveCommand to the MCU over the App Lab Bridge, if available.

    Records the outcome in _state["drive"] so /status shows whether Bridge.call is actually
    reaching the MCU (vs silently failing) — the key signal when the wheels don't move."""
    if _calibrating or _manual:  # calibration / manual drive-test owns the motors
        return
    if not _BRIDGE:
        with _lock:
            _state["drive"] = "no bridge"
        return
    if cmd is None:
        return
    try:
        Bridge.call("drive", float(cmd.linear_velocity), float(cmd.angular_velocity), int(cmd.stop))
        with _lock:
            _state["drive"] = "ok"
    except Exception as e:  # noqa: BLE001 - report it instead of swallowing
        with _lock:
            _state["drive"] = f"ERR {type(e).__name__}: {e}"


def _send_stop() -> None:
    """Brake the motors (used every tick while disarmed, so the rover holds still)."""
    if not _BRIDGE:
        return
    try:
        Bridge.call("drive", 0.0, 0.0, 1)
    except Exception:  # noqa: BLE001
        pass


# ===================================================== motor-driven calibration (TASK B)
# Drives two known legs (forward, then turn-left), reads the raw ARKit pose deltas off the
# feed's /data, computes the pose→rover axis mapping, and POSTs it to the feed's /calib (which
# applies + persists it). Re-runnable any time via POST /calibrate. REQUIRES the wheels to
# actually move — until the drive path is fixed this will report "no motion detected".
_calibrating = False
_manual = False                       # a manual /drive test owns the motors while True
_armed = False                        # autonomy drives ONLY when armed (default: idle/safe)
CAL_FWD_V, CAL_FWD_S = 0.15, 3.0      # forward leg: ~0.15 m/s for 3 s
CAL_TURN_W, CAL_TURN_S = 0.6, 3.0     # turn leg: ~0.6 rad/s CCW (left) for 3 s
CAL_MIN_FWD_M = 0.05                  # need at least this much translation to trust the axis


def _raw_yaw(r) -> float:
    qx, qy, qz, qw = r["qx"], r["qy"], r["qz"], r["qw"]
    return math.atan2(2 * (qw * qy + qz * qx), 1 - 2 * (qy * qy + qx * qx))


def _capture_raw(base: str, n: int = 10):
    """Average n raw poses off /data (uncalibrated tx/ty/tz + yaw). None if no frames."""
    import statistics
    acc = {"tx": [], "ty": [], "tz": [], "yaw": []}
    for _ in range(n * 3):
        if len(acc["tx"]) >= n:
            break
        try:
            p = json.loads(_get(f"{base}/data").decode()).get("pose")
            if p and p.get("raw"):
                r = p["raw"]
                acc["tx"].append(r["tx"]); acc["ty"].append(r["ty"]); acc["tz"].append(r["tz"])
                acc["yaw"].append(_raw_yaw(r))
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.05)
    if len(acc["tx"]) < 3:
        return None
    return {k: statistics.fmean(v) for k, v in acc.items()}


def _drive_for(v: float, w: float, secs: float) -> None:
    """Hold a velocity for `secs` (re-sent each 100 ms to satisfy the MCU watchdog), then stop."""
    end = time.time() + secs
    while time.time() < end:
        try:
            Bridge.call("drive", float(v), float(w), 0)
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.1)
    try:
        Bridge.call("drive", 0.0, 0.0, 1)
    except Exception:  # noqa: BLE001
        pass


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _run_calibration() -> None:
    """Drive forward + turn, measure raw-pose deltas, POST the computed mapping to /calib."""
    global _calibrating
    _calibrating = True

    def setc(s):
        with _lock:
            _state["calib"] = s

    try:
        if not _BRIDGE:
            setc("no bridge — can't drive"); return
        base = _state.get("base") or _find_base()
        if not base:
            setc("feed not reachable"); return
        setc("capturing start pose"); a = _capture_raw(base)
        if not a:
            setc("no pose (is the phone streaming?)"); return
        setc("driving forward..."); _drive_for(CAL_FWD_V, 0.0, CAL_FWD_S); time.sleep(0.4)
        b = _capture_raw(base)
        setc("turning left..."); _drive_for(0.0, CAL_TURN_W, CAL_TURN_S); time.sleep(0.4)
        c = _capture_raw(base)
        if not b or not c:
            setc("lost pose mid-calibration"); return
        # ARKit world is gravity-aligned (Y up), so forward is the horizontal axis (tx/tz)
        # that moved most during the forward leg; the other horizontal axis is lateral.
        dtx, dtz = b["tx"] - a["tx"], b["tz"] - a["tz"]
        if max(abs(dtx), abs(dtz)) < CAL_MIN_FWD_M:
            setc(f"no motion detected (Δtx={dtx:+.3f} Δtz={dtz:+.3f}) — wheels not moving?"); return
        fwd = "tx" if abs(dtx) > abs(dtz) else "tz"
        lat = "tz" if fwd == "tx" else "tx"
        fwd_sign = 1 if (b[fwd] - a[fwd]) >= 0 else -1
        dyaw = _wrap(c["yaw"] - a["yaw"])  # commanded CCW(left); want derived theta to rise
        yaw_sign = 1 if dyaw >= 0 else -1
        calib = {"fwd": fwd, "fwd_sign": fwd_sign, "lat": lat, "lat_sign": 1, "yaw_sign": yaw_sign}
        req = urllib.request.Request(f"{base}/calib", data=json.dumps(calib).encode(),
                                     method="POST", headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=3).read()
        setc(f"done: {calib} | Δfwd={b[fwd]-a[fwd]:+.2f} m, Δyaw={dyaw:+.2f} rad "
             f"(if walls land mirrored, POST scan_sign:-1)")
    except Exception as e:  # noqa: BLE001
        setc(f"error: {e}")
    finally:
        _calibrating = False


def _manual_drive(v: float, w: float, secs: float) -> None:
    """Bench wiring test: drive (v, w) for `secs` then brake — independent of the phone/brain.
    Lets you confirm a `drive` command actually spins the wheels with nothing else running."""
    global _manual
    _manual = True
    try:
        with _lock:
            _state["calib"] = f"manual drive v={v} w={w} for {secs}s"
        _drive_for(v, w, secs)
        with _lock:
            _state["calib"] = "manual drive done"
    finally:
        _manual = False


# ============================================================ map payload =============
def _map_update_json() -> "bytes | None":
    """Serialize the current grid as a MapUpdate (the real browser viewer renders it).

    return_path is populated only while returning (matches the viewer's convention); start
    is the mission home. Reads the grid under _nav_lock so it never races the brain thread.
    """
    with _nav_lock:
        if _last_pose is None or _nav.grid.width == 0:
            return None
        path = _nav.planner.current_path() if _nav.returning else []
        home = None if _nav.start_pose is None else {"x": _nav.start_pose.x, "y": _nav.start_pose.y}
        mu = _nav.grid.to_map_update(_last_pose, return_path=path, start=home)
    return json.dumps(mu.to_dict()).encode()


# ============================================================ brain loop ==============
def _brain_loop() -> None:
    global _last_pose
    base = None
    last_fid = -1
    while True:
        if base is None:
            base = _find_base()
            if base is None:
                with _lock:
                    _state["brain"] = "host feed not reachable — is ~/lidar_feed.py running?"
                time.sleep(2)
                continue
            with _lock:
                _state["brain"], _state["base"] = f"connected to {base}", base
        try:
            data = json.loads(_get(f"{base}/data").decode())
            fid, pose_d, scan = data.get("frame_id"), data.get("pose"), data.get("scan")
            if pose_d is None or not scan:
                with _lock:
                    _state["upstream"] = "no pose/scan yet (phone streaming?)"
                time.sleep(0.2)
                continue
            if fid != last_fid:
                last_fid = fid
                pose = Pose(x=pose_d["x"], y=pose_d["y"], theta=pose_d["theta"],
                            timestamp=pose_d.get("timestamp", time.time()))
                with _nav_lock:
                    cmd = _nav.step(pose, [(a, r) for a, r in scan])
                    _last_pose = pose
                # SAFETY: only drive when explicitly armed. Disarmed = keep mapping but hold
                # the brakes — so the rover never auto-drives (e.g. into a wall) on startup or
                # before calibration. Calibration owns the motors separately while it runs.
                if _calibrating or _manual:
                    pass
                elif _armed:
                    _send_drive(cmd)
                else:
                    _send_stop()
                with _lock:
                    _state["folds"] += 1
                    _state["grid_wh"] = [_nav.grid.width, _nav.grid.height]
                    _state["mode"] = "return" if _nav.returning else "explore"
                    _state["upstream"] = "mapping"
                    _state["cmd"] = (None if cmd is None else
                                     {"v": round(cmd.linear_velocity, 3),
                                      "w": round(cmd.angular_velocity, 3), "stop": cmd.stop})
            time.sleep(0.05)
        except Exception as e:  # noqa: BLE001
            with _lock:
                _state["brain"] = f"reconnecting ({e.__class__.__name__})"
            base = None
            time.sleep(1)


# ============================================================ web (serves real viewer) =
def _static(path: str) -> "tuple[bytes, str] | None":
    """Read a file from the vendored visualization/ dir (path-traversal safe)."""
    rel = path.lstrip("/") or "index.html"
    target = (VIZ / rel).resolve()
    if target != VIZ.resolve() and VIZ.resolve() not in target.parents:
        return None
    if not target.is_file():
        return None
    ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
    if target.suffix == ".js":
        ctype = "application/javascript"  # ES modules need a JS MIME type
    return target.read_bytes(), ctype


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/return":
            with _nav_lock:
                _nav.request_return()
            self._send(200, "application/json", b'{"ok":true}')
        elif path == "/calibrate":
            # Kick off the motor-driven pose calibration (drives forward + turn). Re-runnable.
            if _calibrating:
                self._send(409, "application/json", b'{"error":"already calibrating"}')
            else:
                threading.Thread(target=_run_calibration, name="calibrate", daemon=True).start()
                self._send(200, "application/json", b'{"ok":true,"msg":"calibration started"}')
        elif path == "/drive":
            # Bench wiring test (phone-independent): POST /drive?v=0.15&w=0&secs=2
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            try:
                v = float(q.get("v", ["0.15"])[0]); w = float(q.get("w", ["0"])[0])
                secs = min(float(q.get("secs", ["2"])[0]), 10.0)  # cap at 10 s for safety
            except ValueError:
                self._send(400, "text/plain", b"bad v/w/secs"); return
            if _calibrating or _manual:
                self._send(409, "application/json", b'{"error":"motors busy"}')
            else:
                threading.Thread(target=_manual_drive, args=(v, w, secs), daemon=True).start()
                self._send(200, "application/json",
                           json.dumps({"ok": True, "v": v, "w": w, "secs": secs}).encode())
        elif path == "/start":   # ARM autonomy — the rover begins driving (explore)
            global _armed
            _armed = True
            self._send(200, "application/json", b'{"ok":true,"armed":true}')
        elif path == "/stop":    # DISARM — brake + hold still (safe). App keeps running/mapping.
            _armed = False
            _send_stop()
            self._send(200, "application/json", b'{"ok":true,"armed":false}')
        else:
            self._send(404, "text/plain", b"not found")

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/mapupdate":
            body = _map_update_json()
            self._send(200, "application/json", body) if body else self._send(503, "text/plain", b"no map yet")
        elif path == "/status":
            with _lock:
                s = {k: _state[k] for k in ("brain", "upstream", "folds", "grid_wh", "mode", "cmd", "base", "drive", "calib")}
                s["bridge"] = _BRIDGE
                s["calibrating"] = _calibrating
                s["armed"] = _armed
            # Navigator internals — so we can see WHY it's doing what it's doing (survey vs
            # recovery vs exploring, what it's blocked by) instead of guessing.
            with _nav_lock:
                try:
                    s["nav"] = {
                        "surveying": _nav._surveying,
                        "survey_deg": round(_nav._survey_accum * 57.3, 0),
                        "recovery": _nav._recovery,
                        "recovery_count": _nav._recovery_count,
                        "turn_swept_deg": round(_nav._turn_swept * 57.3, 0),
                        "turn_sign": _nav._turn_sign,
                        "fwd_clear_m": round(_nav._forward_clear(), 2),
                        "blocked_ahead": _nav._blocked_ahead(),
                        "boxed": _nav._boxed(),
                        "dist_since_survey": round(_nav._dist_since_survey, 2),
                        "returning": _nav.returning,
                    }
                except Exception as e:  # noqa: BLE001
                    s["nav"] = {"error": str(e)}
            self._send(200, "application/json", json.dumps(s).encode())
        else:
            asset = _static(path)
            self._send(200, asset[1], asset[0]) if asset else self._send(404, "text/plain", b"not found")


def _serve():
    # 0.0.0.0 is the container's internal interface; App Lab maps :8000 to the host for the
    # demo UI (the container is not on the LAN itself). The host-side LAN exposure that
    # mattered — the raw depth feed — is locked to the docker bridges in lidar_feed.py.
    ThreadingHTTPServer(("0.0.0.0", PORT), _Handler).serve_forever()


threading.Thread(target=_serve, name="brain-web", daemon=True).start()
threading.Thread(target=_brain_loop, name="brain", daemon=True).start()
print(f"[brain] serving viewer + /mapupdate on :{PORT}; pulling /data from gateway:{FEED_PORT}; bridge={_BRIDGE}", flush=True)


def loop() -> None:
    with _lock:
        b, m, n = _state["brain"], _state["mode"], _state["folds"]
    print(f"[brain] {b} | mode={m} | folds={n}", flush=True)
    time.sleep(5)


# See: https://docs.arduino.cc/software/app-lab/tutorials/getting-started/#app-run
App.run(user_loop=loop)
