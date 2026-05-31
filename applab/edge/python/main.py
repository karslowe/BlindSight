"""App Lab app: the navigation BRAIN (Branch B, steps 3-4).

Capture (record3d) runs natively on the host (~/lidar_feed.py) and exposes brain-facing data
at /data: 2D camera pose + a horizontal depth scan. THIS app is the brain:

  * pulls /data over the docker gateway,
  * folds (pose, scan) into the real OccupancyGrid (navigation/mapping),
  * runs autonomous frontier exploration + return-home planning (navigation/planning),
  * computes DriveCommands (sent to the car over the App Lab Bridge by the sketch side),
  * serves a live top-down map with the planned path + pose trail on :8000.

Deps: numpy only (prebuilt aarch64 wheel; never PIN it — see requirements.txt). The mapping,
planning, and schema modules are vendored under python/ already; we just put them on sys.path.

Drive output: the latest DriveCommand is published to the MCU via Bridge.call("drive", ...)
when the App Lab Bridge is available. Without the Bridge (e.g. running the logic standalone)
the command is still computed and shown in /status, so the planner is testable with no car.
"""

from __future__ import annotations

import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from arduino.app_utils import App

PORT = 8000
FEED_PORT = 8008
FETCH_TIMEOUT = 2.0
CELL_PX = 4                       # display upscale: each grid cell -> CELL_PX px

HERE = Path(__file__).resolve().parent
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
import numpy as np  # noqa: E402
from schemas.schemas import Pose  # noqa: E402
from navigator import Navigator  # noqa: E402  (autonomy core; sets up navigation/shared paths)

# Optional: the App Lab Bridge to the MCU. Imported lazily so the brain runs without it.
try:
    from arduino.app_utils import Bridge  # type: ignore
    _BRIDGE = True
except Exception:  # noqa: BLE001
    _BRIDGE = False

_lock = threading.Lock()
_state = {"brain": "starting", "upstream": "unknown", "folds": 0, "map_png": None,
          "grid_wh": None, "mode": "explore", "cmd": None, "base": None}

_nav = Navigator()


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


# ============================================================ rendering ==============
def _png_gray(img) -> bytes:
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape
    filtered = np.empty((h, w + 1), dtype=np.uint8)
    filtered[:, 0] = 0
    filtered[:, 1:] = img

    def chunk(typ, payload):
        body = typ + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(filtered.tobytes(), 1)) + chunk(b"IEND", b""))


def _line(img, c0, r0, c1, r1, val):
    """Rasterize a cell-space line into img (Bresenham), bounds-checked."""
    h, w = img.shape
    dc, dr = abs(c1 - c0), abs(r1 - r0)
    sc, sr = (1 if c0 < c1 else -1), (1 if r0 < r1 else -1)
    err = dc - dr
    c, r = c0, r0
    while True:
        if 0 <= r < h and 0 <= c < w:
            img[r, c] = val
        if c == c1 and r == r1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr; c += sc
        if e2 < dc:
            err += dc; r += sr


def _render() -> bytes:
    """Top-down map: unknown=60 free=170 obstacle=10 trail=95 path=230 home=120 rover=255."""
    g = _nav.grid
    mu = g.to_map_update(_nav.driven_path[-1] if _nav.driven_path else _nav.start_pose
                         or Pose(0, 0, 0, 0))
    h, w = mu.height, mu.width
    if h == 0 or w == 0:
        return _png_gray(np.full((1, 1), 60, dtype=np.uint8))
    cells = np.asarray(mu.cells, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), 60, dtype=np.uint8)
    img[cells == 0] = 170
    img[cells == 100] = 10

    def cell(px, py):
        return g.world_to_cell(px, py)

    # pose trail (faint)
    trail = _nav.driven_path
    for i in range(1, len(trail)):
        c0, r0 = cell(trail[i - 1].x, trail[i - 1].y)
        c1, r1 = cell(trail[i].x, trail[i].y)
        _line(img, c0, r0, c1, r1, 95)
    # active planned path (frontier route while exploring, route home while returning)
    path = _nav.planner.current_path()
    for i in range(1, len(path)):
        c0, r0 = cell(path[i - 1].x, path[i - 1].y)
        c1, r1 = cell(path[i].x, path[i].y)
        _line(img, c0, r0, c1, r1, 230)

    def mark(px, py, val):
        c, r = cell(px, py)
        if 0 <= r < h and 0 <= c < w:
            img[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = val

    if _nav.start_pose is not None:
        mark(_nav.start_pose.x, _nav.start_pose.y, 120)
    mark(mu.pose.x, mu.pose.y, 255)
    img = np.flipud(img)  # row 0 is the bottom row
    img = np.repeat(np.repeat(img, CELL_PX, axis=0), CELL_PX, axis=1)
    return _png_gray(img)


# ============================================================ car bridge =============
# Latest telemetry from the sketch (forward ultrasonic etc.), folded into the scan.
_telemetry = {"ultrasonic": None, "bumper": 0, "ts": 0.0}


def _on_car_telemetry(ultrasonic, bumper, line_left, line_center, line_right, ts):
    """Bridge handler: the sketch pushes CarTelemetry here via Bridge.notify('car_telemetry',...)."""
    with _lock:
        _telemetry.update(ultrasonic=float(ultrasonic), bumper=int(bumper), ts=float(ts))


if _BRIDGE:
    try:
        Bridge.provide("car_telemetry", _on_car_telemetry)
    except Exception:  # noqa: BLE001 - already registered / bridge not ready
        pass


def _send_drive(cmd) -> None:
    """Publish the DriveCommand to the MCU over the App Lab Bridge, if available."""
    if cmd is None or not _BRIDGE:
        return
    try:
        Bridge.call("drive", cmd.linear_velocity, cmd.angular_velocity, cmd.stop)
    except Exception:  # noqa: BLE001 - sketch may not be up yet; don't kill the loop
        pass


def _augment_scan(scan):
    """Add the car's forward ultrasonic as a 0-rad ray (mirrors the orchestrator fallback)."""
    with _lock:
        u = _telemetry["ultrasonic"]
    if u is not None and u >= 0:
        return list(scan) + [(0.0, u)]
    return scan


# ============================================================ brain loop ==============
def _brain_loop() -> None:
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
                cmd = _nav.step(pose, _augment_scan([(a, r) for a, r in scan]))
                _send_drive(cmd)
                png = _render()
                with _lock:
                    _state["map_png"] = png
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


# ============================================================ web =====================
_PAGE = """<!doctype html><meta charset=utf-8><title>BlindSight map (App Lab)</title>
<body style="margin:0;background:#11151c;color:#ecf0f1;font-family:system-ui;text-align:center">
<h1 style="margin:12px">Live occupancy map + autonomy — App Lab</h1>
<img id=f style="max-width:96vw;max-height:62vh;image-rendering:pixelated;background:#000;border:1px solid #333">
<div style="margin:8px"><button id=ret style="min-height:44px;padding:0 22px;font-weight:700;border:0;border-radius:10px;background:#2d6cdf;color:#fff">Return home</button></div>
<p id=cap style="color:#95a5a6"></p>
<p style="color:#7f8c8d;font-size:12px">white=rover · yellow-line=plan · faint=trail · light=free · dark=obstacle · gray=unknown · mid=home</p>
<script>
const img=document.getElementById('f'), cap=document.getElementById('cap');
function next(){ img.src='/map.png?t='+Date.now(); }
img.onload=img.onerror=()=>requestAnimationFrame(next);
document.getElementById('ret').onclick=()=>fetch('/return',{method:'POST'});
async function stat(){ try{const s=await (await fetch('/status?t='+Date.now())).json();
  const c=s.cmd?('v='+s.cmd.v+' w='+s.cmd.w+(s.cmd.stop?' STOP':'')):'-';
  cap.textContent='brain: '+s.brain+' | mode: '+s.mode+' | drive: '+c+' | folds: '+s.folds+' | grid: '+(s.grid_wh||'-');}catch(e){} }
setInterval(stat,500); stat(); next();
</script></body>"""


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
        if self.path.split("?", 1)[0] == "/return":
            _nav.request_return()
            self._send(200, "application/json", b'{"ok":true}')
        else:
            self._send(404, "text/plain", b"not found")

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/map.png":
            with _lock:
                png = _state["map_png"]
            self._send(200, "image/png", png) if png else self._send(503, "text/plain", b"no map yet")
        elif path == "/status":
            with _lock:
                s = {k: _state[k] for k in ("brain", "upstream", "folds", "grid_wh", "mode", "cmd", "base")}
                s["bridge"] = _BRIDGE
                s["telemetry"] = dict(_telemetry)
            self._send(200, "application/json", json.dumps(s).encode())
        else:
            self._send(200, "text/html; charset=utf-8", _PAGE.encode())


def _serve():
    ThreadingHTTPServer(("0.0.0.0", PORT), _Handler).serve_forever()


threading.Thread(target=_serve, name="brain-web", daemon=True).start()
threading.Thread(target=_brain_loop, name="brain", daemon=True).start()
print(f"[brain] serving map on :{PORT}; pulling /data from gateway:{FEED_PORT}; bridge={_BRIDGE}", flush=True)


def loop() -> None:
    with _lock:
        b, m, n = _state["brain"], _state["mode"], _state["folds"]
    print(f"[brain] {b} | mode={m} | folds={n}", flush=True)
    time.sleep(5)


# See: https://docs.arduino.cc/software/app-lab/tutorials/getting-started/#app-run
App.run(user_loop=loop)
