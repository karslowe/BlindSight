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

import struct
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

_lock = threading.Lock()
# latest_depth holds ONLY the newest frame (overwritten each callback). frame_id bumps so the
# HTTP side can cache an encoded PNG and skip re-encoding while no new frame has arrived.
# pose/scan are the brain-facing data (the App Lab container pulls them from /data).
_state = {"capture": "starting", "frames": 0, "latest_depth": None, "frame_id": 0,
          "depth_shape": None, "pose": None, "scan": None}
_png_cache = {"id": -1, "png": None}
_png_cache_lock = threading.Lock()


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
    return {"x": float(cam.tz), "y": float(cam.tx), "theta": float(yaw), "timestamp": ts}


def _depth_to_scan(depth, intr) -> list:
    """Horizontal ray fan from the depth frame: list of [angle_offset_rad, range_m].

    Takes a band of rows around the vertical center, the nearest valid depth per column, and
    converts each sampled column to (angle relative to the optical axis, ray range). Columns
    left of center are CCW-positive to match the grid's angle convention. z-depth is converted
    to ray length by /cos(angle). Invalid/empty columns emit the DEPTH_MAX_M no-return sentinel.
    """
    d = np.asarray(depth, dtype=np.float32)
    h, w = d.shape
    fx = float(intr[0, 0]) if intr is not None else float(w)  # fallback: ~53 deg HFOV
    cx = float(intr[0, 2]) if intr is not None else (w / 2.0)
    # intrinsics may be quoted at a different resolution than the depth frame; scale to width.
    if intr is not None and abs(cx - w / 2.0) > w * 0.25:
        s = w / (2.0 * cx)
        fx *= s; cx *= s
    r0 = max(0, int(h * (0.5 - SCAN_BAND / 2))); r1 = min(h, int(h * (0.5 + SCAN_BAND / 2)) + 1)
    band = d[r0:r1, :]
    band = np.where(np.isfinite(band) & (band > 0.05), band, np.inf)
    col_min = band.min(axis=0)  # nearest obstacle per column
    cols = np.linspace(0, w - 1, SCAN_RAYS).astype(int)
    scan = []
    for u in cols:
        ang = math.atan2(cx - u, fx)  # left of center -> positive (CCW)
        z = col_min[u]
        rng = DEPTH_MAX_M if not np.isfinite(z) else min(float(z) / max(math.cos(ang), 1e-3), DEPTH_MAX_M)
        scan.append([round(ang, 5), round(rng, 4)])
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
                    pose = _pose_from_camera(stream.get_camera_pose(), time.time())
                    scan = _depth_to_scan(d, intr)
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
            except Exception as e:  # noqa: BLE001
                with _lock:
                    _state["capture"] = f"frame grab error: {e!r}"

        stream.on_new_frame = on_new_frame
        stream.on_stream_stopped = lambda: stopped.set()
        try:
            stream.connect(devices[0])
            with _lock:
                _state["capture"] = "connected — waiting for frames (is the phone streaming?)"
        except Exception as e:  # noqa: BLE001
            with _lock:
                _state["capture"] = f"connect() raised: {e!r}"
            time.sleep(3)
            continue
        stopped.wait()
        with _lock:
            _state["capture"] = "stream stopped; reconnecting..."
        time.sleep(2)


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
        else:
            self._send(200, "text/html; charset=utf-8", _PAGE.encode())


def main() -> None:
    threading.Thread(target=_capture_loop, name="lidar-capture", daemon=True).start()
    print(f"[lidar] serving feed on http://0.0.0.0:{PORT}  (open http://<board-ip>:{PORT})", flush=True)
    try:
        ThreadingHTTPServer(("0.0.0.0", PORT), _Handler).serve_forever()
    except KeyboardInterrupt:
        print("\n[lidar] stopped")


if __name__ == "__main__":
    main()
