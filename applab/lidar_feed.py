"""NATIVE host LiDAR feed (Branch B, step 1: prove record3d works on this phone+board).

Runs on the Dragonwing HOST (NOT in the App Lab container) — the host has gcc, /dev/bus/usb
access to the iPhone, and (after the apt+pip step below) record3d. It connects to the iPhone
over record3d, turns each depth frame into a grayscale PNG (near = bright), and serves a page
at http://<board-ip>:8008 that shows the live LiDAR depth feed.

LATENCY DESIGN (why it can run the phone at 60 fps without lag): capture and encoding are
DECOUPLED. The record3d callback does almost nothing — it just stores the newest depth frame
and drops the previous one (we never display old frames). PNG encoding happens on demand in
the HTTP handler, so we only ever encode at the rate the browser actually fetches (~display
rate), and always the freshest frame. Encoding every one of 60 fps in the callback was what
built a backlog = latency. zlib level is low (speed over size; this is a LAN). Optional
DOWNSCALE halves resolution for even less encode+transfer cost.

SETUP (host shell, one time):
    sudo apt-get update && sudo apt-get install -y cmake libusb-1.0-0-dev   # gcc already present
    python3 -m venv ~/r3d-venv
    CXXFLAGS="-include cstdint -include string" ~/r3d-venv/bin/pip install numpy record3d
    # ^ the CXXFLAGS force-includes dodge a record3d-vs-GCC-14 'incomplete type std::string' error

RUN:
    sudo systemctl start usbmuxd              # it's 'static'; start if not running
    # On the iPhone: Record3D app -> USB Streaming -> tap the live-stream button
    ~/r3d-venv/bin/python ~/lidar_feed.py
    # open http://<board-ip>:8008

Boot-loop gotcha: boot the board WITHOUT the iPhone, wait for Linux, THEN hot-plug. This
script retries connection every few seconds, so hot-plugging after it starts is fine.
"""

from __future__ import annotations

import os
import struct
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from record3d import Record3DStream

import math

PORT = 8008
DEPTH_MAX_M = 4.0   # depth (m) mapped to black; nearer = brighter. Also the scan no-return range.
ZLIB_LEVEL = 1      # 0=none(biggest,fastest) .. 9=smallest(slowest). 1 is the LAN sweet spot.
DOWNSCALE = 1       # 1 = full 480x640; 2 = quarter the pixels (faster encode + transfer).
SCAN_RAYS = 90      # columns sampled from the depth band into the horizontal scan.
SCAN_BAND = 0.10    # fraction of image height (around vertical center) used as the scan band.
FRAME_TIMEOUT_S = 3.0  # no new frame for this long => stream is dead; disconnect + reconnect.

_lock = threading.Lock()
# latest_depth holds ONLY the newest frame (overwritten each callback). frame_id bumps so the
# HTTP side can cache an encoded PNG and skip re-encoding while no new frame has arrived.
# pose/scan are the brain-facing data (the App Lab container pulls them from /data).
_state = {"capture": "starting", "frames": 0, "latest_depth": None, "frame_id": 0,
          "depth_shape": None, "pose": None, "scan": None}
_png_cache = {"id": -1, "png": None}
_png_cache_lock = threading.Lock()

# ---- runtime pose calibration (TASK B) --------------------------------------------------
# How the phone's ARKit pose maps to the rover ground frame. Set by calibration (manual
# calibrate.py or the motor-driven routine) via POST /calib — no code edit / restart needed.
# Persisted to CALIB_FILE so it survives a reboot. Defaults reproduce the original mapping
# (x = +tz forward, y = +tx left, theta = +yaw, scan angle unflipped).
import json as _json
from pathlib import Path as _Path
CALIB_FILE = _Path.home() / "blindsight_calib.json"
_calib_lock = threading.Lock()
_calib = {"fwd": "tz", "fwd_sign": 1, "lat": "tx", "lat_sign": 1, "yaw_sign": 1, "scan_sign": 1,
          # scan tuning (fixes "open corridor read as a wall"):
          "conf_min": 1,          # drop LiDAR returns below this confidence (0/1/2); 2 = strictest
          "ground_reject": 1,     # 1 = drop floor returns (phantom walls across open floor)
          "cam_height_m": 0.12,   # phone camera height above the floor (m) — tune to your mount
          "ground_margin_m": 0.06,  # keep returns this far above the floor as real obstacles
          "auto_pitch": 1,        # 1 = derive camera pitch from the pose each frame (recommended)
          "auto_pitch_sign": 1,   # flip to -1 if /depthprobe pitch_auto has the wrong sign
          "pitch_offset_deg": 0.0,  # added to the auto pitch (bias correction)
          "pitch_deg": 0.0}       # manual fallback tilt (deg, nose-down +) when auto_pitch=0
_scan_dbg = {}  # last scan diagnostics for /depthprobe


def _load_calib() -> None:
    try:
        with _calib_lock:
            _calib.update(_json.loads(CALIB_FILE.read_text()))
        print(f"[lidar] loaded calibration from {CALIB_FILE}: {_calib}", flush=True)
    except FileNotFoundError:
        print("[lidar] no calibration file yet — using identity defaults", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[lidar] bad calibration file ({e}); using defaults", flush=True)


def _save_calib() -> None:
    with _calib_lock:
        CALIB_FILE.write_text(_json.dumps(_calib, indent=2))


# ----------------------------------------------------------- stdlib grayscale PNG -----
def _encode_png_gray(img: "np.ndarray") -> bytes:
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape
    # PNG wants a filter byte (0 = none) prepended to each scanline. Build that with numpy
    # (a zero column + the image) instead of a per-row Python loop — ~5x faster to assemble.
    filtered = np.empty((h, w + 1), dtype=np.uint8)
    filtered[:, 0] = 0
    filtered[:, 1:] = img
    comp = zlib.compress(filtered.tobytes(), ZLIB_LEVEL)

    def chunk(typ: bytes, payload: bytes) -> bytes:
        body = typ + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)
    return sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", comp) + chunk(b"IEND", b"")


def _depth_to_png(depth) -> bytes:
    d = np.asarray(depth, dtype="float32")
    if DOWNSCALE > 1:
        d = d[::DOWNSCALE, ::DOWNSCALE]
    valid = np.isfinite(d) & (d > 0)
    g = np.zeros(d.shape, dtype="float32")
    dmax = DEPTH_MAX_M if DEPTH_MAX_M > 0 else (float(d[valid].max()) if valid.any() else 1.0)
    g[valid] = np.clip(1.0 - d[valid] / dmax, 0.0, 1.0)
    return _encode_png_gray((g * 255).astype("uint8"))


def _latest_png():
    """Encode the newest frame on demand, caching per frame_id so repeated fetches (and the
    common case of polling faster than 60 fps) don't re-encode the same frame."""
    with _lock:
        fid = _state["frame_id"]
        depth = _state["latest_depth"]
    if depth is None:
        return None
    with _png_cache_lock:
        if _png_cache["id"] == fid and _png_cache["png"] is not None:
            return _png_cache["png"]
        png = _depth_to_png(depth)
        _png_cache["id"], _png_cache["png"] = fid, png
        return png


# ----------------------------------------------------- pose + scan (brain-facing data) -
def _pose_from_camera(cam, ts: float) -> dict:
    """Record3D 6-DoF camera transform -> our ground-plane 2D pose dict {x,y,theta,timestamp}.

    Mirrors navigation/bridge/phone_link._pose_from_camera: assumes an upright forward-facing
    phone (x_map = tz forward, y_map = tx lateral, theta = yaw about the up/Y axis). This axis
    mapping is the TASK B calibration — verify on the bench; a wrong mapping smears the map.
    """
    qx, qy, qz, qw = cam.qx, cam.qy, cam.qz, cam.qw
    yaw = math.atan2(2.0 * (qw * qy + qz * qx), 1.0 - 2.0 * (qy * qy + qx * qx))
    raw = {"tx": float(cam.tx), "ty": float(cam.ty), "tz": float(cam.tz),
           "qx": float(qx), "qy": float(qy), "qz": float(qz), "qw": float(qw)}
    # Apply the runtime calibration (POST /calib sets it). raw is carried through so the
    # calibration routine can recompute the mapping from real moves.
    with _calib_lock:
        c = dict(_calib)
    return {"x": c["fwd_sign"] * raw[c["fwd"]], "y": c["lat_sign"] * raw[c["lat"]],
            "theta": c["yaw_sign"] * float(yaw), "timestamp": ts, "raw": raw}


def _pitch_from_cam(cam) -> float:
    """Camera down-tilt below horizontal, in DEGREES (nose-down positive), from the pose.

    ARKit's world frame is gravity-aligned (Y up), so the camera's forward axis (+Z) world
    Y-component gives how much the optical axis dips below horizontal: pitch = asin(-Zf.y),
    where Zf.y = 2(qy*qz - qx*qw). Same quaternion convention the yaw extraction above uses.
    Sign/offset are correctable at runtime (auto_pitch_sign, pitch_offset_deg) since the
    record3d/pinhole convention can flip — verify with /depthprobe (level -> ~0, tilt -> +).
    """
    qx, qy, qz, qw = cam.qx, cam.qy, cam.qz, cam.qw
    s = max(-1.0, min(1.0, 2.0 * (qx * qw - qy * qz)))   # -(forward+Z world Y component)
    return math.degrees(math.asin(s))


def _depth_to_scan(depth, intr, conf=None, pitch_auto_deg=0.0) -> list:
    """Horizontal ray fan from the depth frame: list of [angle_offset_rad, range_m].

    Takes a band of rows around the vertical center; per column, the nearest depth that is
    (a) valid, (b) high-enough LiDAR confidence, and (c) NOT the floor. Converts to (angle,
    range). The confidence + ground filters fix "open corridor read as a wall": low-confidence
    distant returns and the floor a couple metres ahead are both dropped instead of becoming
    phantom walls. Columns with no obstacle return the DEPTH_MAX_M no-return sentinel (free).

    Ground rejection: for a roughly level camera at height `cam_height_m`, a floor pixel sits
    that far below the optical axis — its reconstructed height Yc=(row-cy)/fy*depth ≈ cam_height.
    Pixels at/below floor level are dropped; ones standing >ground_margin above it are obstacles.
    """
    d = np.asarray(depth, dtype=np.float32)
    h, w = d.shape
    fx = float(intr[0, 0]) if intr is not None else float(w)
    fy = float(intr[1, 1]) if intr is not None else float(w)
    cx = float(intr[0, 2]) if intr is not None else (w / 2.0)
    cy = float(intr[1, 2]) if intr is not None else (h / 2.0)
    # intrinsics may be quoted at a different resolution than the depth frame; scale to it.
    if intr is not None and abs(cx - w / 2.0) > w * 0.25:
        sx = w / (2.0 * cx); fx *= sx; cx *= sx
        sy = h / (2.0 * cy); fy *= sy; cy *= sy
    with _calib_lock:
        c = dict(_calib)
    r0 = max(0, int(h * (0.5 - SCAN_BAND / 2)))
    r1 = min(h, int(h * (0.5 + SCAN_BAND / 2)) + 1)
    rows = np.arange(r0, r1)

    # Effective camera pitch: derived from the pose (auto) or the manual override.
    if c.get("auto_pitch"):
        pitch_deg = c.get("auto_pitch_sign", 1) * pitch_auto_deg + c.get("pitch_offset_deg", 0.0)
    else:
        pitch_deg = c.get("pitch_deg", 0.0)

    band = d[r0:r1, :].astype(np.float32)
    valid = np.isfinite(band) & (band > 0.05)
    if conf is not None and conf.shape == d.shape:           # drop low-confidence returns
        valid &= (conf[r0:r1, :] >= c["conf_min"])
    if c.get("ground_reject"):                               # drop floor returns
        # True drop below the camera, accounting for a nose-down pitch: a floor point sits at
        # ~cam_height regardless of distance. depth * [(row-cy)/fy*cos(pitch) + sin(pitch)].
        pitch = math.radians(pitch_deg)
        yc = band * ((rows[:, None].astype(np.float32) - cy) / fy * math.cos(pitch) + math.sin(pitch))
        floor = yc >= (c["cam_height_m"] - c["ground_margin_m"])
        valid &= ~floor

    band = np.where(valid, band, np.inf)
    col_min = band.min(axis=0)                               # nearest kept obstacle per column

    cols = np.linspace(0, w - 1, SCAN_RAYS).astype(int)
    scan, n_obst = [], 0
    for u in cols:
        ang = c["scan_sign"] * math.atan2(cx - u, fx)
        z = col_min[u]
        if np.isfinite(z):
            rng = min(float(z) / max(math.cos(ang), 1e-3), DEPTH_MAX_M)
            n_obst += rng < DEPTH_MAX_M - 1e-6
        else:
            rng = DEPTH_MAX_M
        scan.append([round(ang, 5), round(rng, 4)])
    with _calib_lock:
        _scan_dbg.update(rays=len(scan), obstacles=int(n_obst), conf_min=c["conf_min"],
                         ground_reject=int(bool(c.get("ground_reject"))),
                         pitch_auto_deg=round(pitch_auto_deg, 1), pitch_used_deg=round(pitch_deg, 1))
    return scan


# ---------------------------------------------------------------- record3d capture ----
def _capture_loop() -> None:
    while True:
        try:
            devices = Record3DStream.get_connected_devices()
        except Exception as e:  # noqa: BLE001
            with _lock:
                _state["capture"] = f"get_connected_devices() raised: {e!r}"
            time.sleep(3)
            continue
        if not devices:
            with _lock:
                _state["capture"] = "no Record3D device — enable USB Streaming + tap live on the phone"
            time.sleep(3)
            continue

        stopped = threading.Event()
        stream = Record3DStream()
        last_frame = [time.time()]   # wall-clock of the most recent frame (watchdog input)

        def on_new_frame() -> None:
            # CHEAP: copy the newest depth (record3d reuses its buffer) and drop the old one.
            # No encoding here — that's what kept the feed from keeping up at 60 fps.
            try:
                d = stream.get_depth_frame()
                # pose + intrinsics for the brain. Guard each: some builds/frames may lack them.
                pose = scan = None
                try:
                    intr = np.asarray(stream.get_intrinsic_mat(), dtype=np.float32)
                except Exception:  # noqa: BLE001
                    intr = None
                try:
                    conf = np.asarray(stream.get_confidence_frame())  # 0=low,1=med,2=high
                except Exception:  # noqa: BLE001
                    conf = None
                try:
                    cam = stream.get_camera_pose()
                    pose = _pose_from_camera(cam, time.time())
                    scan = _depth_to_scan(d, intr, conf, _pitch_from_cam(cam))
                except Exception:  # noqa: BLE001
                    pass
                with _lock:
                    _state["latest_depth"] = np.array(d, copy=True)
                    _state["frames"] += 1
                    _state["frame_id"] += 1
                    _state["depth_shape"] = list(getattr(d, "shape", []))
                    if pose is not None:
                        _state["pose"] = pose
                    if scan is not None:
                        _state["scan"] = scan
                    _state["capture"] = "STREAMING — LiDAR depth live"
                last_frame[0] = time.time()
            except Exception as e:  # noqa: BLE001
                with _lock:
                    _state["capture"] = f"frame grab error: {e!r}"

        stream.on_new_frame = on_new_frame
        stream.on_stream_stopped = lambda: stopped.set()
        try:
            stream.connect(devices[0])
            last_frame[0] = time.time()   # grace period before the watchdog can fire
            with _lock:
                _state["capture"] = "connected — waiting for frames (is the phone streaming?)"
        except Exception as e:  # noqa: BLE001
            with _lock:
                _state["capture"] = f"connect() raised: {e!r}"
            time.sleep(3)
            continue

        # Watchdog: reconnect on an explicit stop OR if frames go silent. record3d USB drops
        # frequently DON'T fire on_stream_stopped — leaving a dead stream while the phone sits
        # on "waiting for connection" forever. Forcing a disconnect+reconnect re-initiates the
        # handshake the phone is waiting for, so the feed self-heals without a manual restart.
        while not stopped.wait(0.5):
            if time.time() - last_frame[0] > FRAME_TIMEOUT_S:
                with _lock:
                    _state["capture"] = "frames stalled; reconnecting..."
                break
        try:
            stream.disconnect()
        except Exception:  # noqa: BLE001
            pass
        time.sleep(1.0)


# ---------------------------------------------------------------------------- web -----
_PAGE = """<!doctype html><meta charset=utf-8><title>iPhone LiDAR feed</title>
<body style="margin:0;background:#11151c;color:#ecf0f1;font-family:system-ui;text-align:center">
<h1 style="margin:12px">iPhone LiDAR depth feed (native)</h1>
<img id=f style="max-width:96vw;max-height:72vh;image-rendering:pixelated;background:#000;border:1px solid #333">
<p id=cap style="color:#95a5a6"></p>
<script>
// Chain fetches (no fixed timer): request the next frame as soon as the last <img> loads,
// so the display runs as fast as encode+network allow and always shows the freshest frame.
const img=document.getElementById('f'), cap=document.getElementById('cap');
function next(){ img.src='/frame.png?t='+Date.now(); }
img.onload=img.onerror=()=>requestAnimationFrame(next);
async function stat(){ try{const s=await (await fetch('/status?t='+Date.now())).json();
  cap.textContent='capture: '+s.capture+'  |  frames: '+s.frames+'  |  shape: '+(s.depth_shape||'-');}catch(e){} }
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

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/frame.png":
            png = _latest_png()
            self._send(200, "image/png", png) if png else self._send(503, "text/plain", b"no frame yet")
        elif path == "/status":
            import json
            with _lock:
                s = {k: _state[k] for k in ("capture", "frames", "depth_shape")}
            self._send(200, "application/json", json.dumps(s).encode())
        elif path == "/data":
            # Brain-facing data: 2D pose + horizontal scan. The App Lab container pulls this.
            import json
            with _lock:
                s = {"frames": _state["frames"], "frame_id": _state["frame_id"],
                     "pose": _state["pose"], "scan": _state["scan"]}
            self._send(200, "application/json", json.dumps(s).encode())
        elif path == "/calib":
            with _calib_lock:
                self._send(200, "application/json", _json.dumps(_calib).encode())
        elif path == "/depthprobe":
            # Diagnostic: scan summary + the center column's depth down the rows. A floor reads
            # as depth that RAMPS DOWN toward the bottom rows; low-confidence noise is erratic.
            with _lock:
                d = _state["latest_depth"]
            with _calib_lock:
                dbg = dict(_scan_dbg); cal = {k: _calib[k] for k in
                          ("conf_min", "ground_reject", "cam_height_m", "ground_margin_m")}
            out = {"scan": dbg, "calib": cal}
            if d is not None:
                col = d[:, d.shape[1] // 2]
                rows = np.linspace(0, len(col) - 1, 12).astype(int)
                out["center_col_depth"] = [round(float(col[r]), 2) for r in rows]
            self._send(200, "application/json", _json.dumps(out).encode())
        else:
            self._send(200, "text/html; charset=utf-8", _PAGE.encode())

    def do_POST(self):
        # POST /calib {fwd,fwd_sign,lat,lat_sign,yaw_sign,scan_sign} — set + persist the pose
        # calibration at runtime (manual calibrate.py or the motor-driven routine post here).
        if self.path.split("?", 1)[0] == "/calib":
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = _json.loads(self.rfile.read(n).decode()) if n else {}
                allowed = {"fwd", "fwd_sign", "lat", "lat_sign", "yaw_sign", "scan_sign",
                           "conf_min", "ground_reject", "cam_height_m", "ground_margin_m",
                           "pitch_deg", "auto_pitch", "auto_pitch_sign", "pitch_offset_deg"}
                with _calib_lock:
                    _calib.update({k: body[k] for k in body if k in allowed})
                    snap = dict(_calib)
                _save_calib()
                print(f"[lidar] calibration updated: {snap}", flush=True)
                self._send(200, "application/json", _json.dumps(snap).encode())
            except Exception as e:  # noqa: BLE001
                self._send(400, "text/plain", f"bad calib: {e}".encode())
        else:
            self._send(404, "text/plain", b"not found")


def _bind_addrs() -> "list[str]":
    """Where to serve. This feed is consumed ONLY by the App Lab container (over the docker
    bridge), so bind to the docker bridge gateway IPs (172.x) and NOT the LAN (wlan0) — that
    keeps the raw depth/pose off the venue network. Override with LIDAR_BIND (e.g. 0.0.0.0).
    Falls back to 0.0.0.0 only if no bridge is found, so it never silently goes dark."""
    forced = os.environ.get("LIDAR_BIND")
    if forced:
        return [forced]
    ips = []
    try:
        import re
        out = subprocess.check_output(["ip", "-o", "-4", "addr", "show"], text=True)
        for line in out.splitlines():
            m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", line)
            if m and m.group(1).startswith("172."):  # docker/bridge range, not the LAN
                ips.append(m.group(1))
    except Exception:  # noqa: BLE001
        pass
    return ips or ["0.0.0.0"]


def main() -> None:
    _load_calib()
    threading.Thread(target=_capture_loop, name="lidar-capture", daemon=True).start()
    addrs = _bind_addrs()
    # Serve on every chosen address (one ThreadingHTTPServer per bind) so the container
    # reaches us via its bridge gateway regardless of which docker bridge App Lab uses.
    servers = []
    for addr in addrs:
        try:
            servers.append(ThreadingHTTPServer((addr, PORT), _Handler))
            print(f"[lidar] serving feed on http://{addr}:{PORT}", flush=True)
        except OSError as e:
            print(f"[lidar] could not bind {addr}:{PORT} — {e}", flush=True)
    if not servers:
        return
    for s in servers[1:]:
        threading.Thread(target=s.serve_forever, daemon=True).start()
    try:
        servers[0].serve_forever()
    except KeyboardInterrupt:
        print("\n[lidar] stopped")


if __name__ == "__main__":
    main()
