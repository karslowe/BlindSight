"""BlindSight pose calibration (TASK B) — MANUAL fallback (push the rover by hand).

Use this when the motors aren't driving yet. (Once they are, prefer the motor-driven routine:
`curl -X POST http://<board>:8000/calibrate`, which does the same thing automatically.)

You do two known moves by hand; this reads the raw ARKit pose off the live feed, computes the
phone→rover axis mapping, and POSTs it to the feed's /calib (applied + persisted immediately —
no code edit / restart). Run:  ~/r3d-venv/bin/python ~/calibrate.py
"""

import json
import math
import statistics as st
import sys
import time
import urllib.request

BASES = ["http://172.18.0.1:8008", "http://172.17.0.1:8008", "http://127.0.0.1:8008"]


def _base():
    for b in BASES:
        try:
            urllib.request.urlopen(b + "/status", timeout=2).read()
            return b
        except Exception:
            continue
    sys.exit("feed not reachable on any bridge IP — is lidar-feed running?")


def _raw_yaw(r):
    qx, qy, qz, qw = r["qx"], r["qy"], r["qz"], r["qw"]
    return math.atan2(2 * (qw * qy + qz * qx), 1 - 2 * (qy * qy + qx * qx))


def capture(base, n=12):
    acc = {"tx": [], "ty": [], "tz": [], "yaw": []}
    for _ in range(n * 3):
        if len(acc["tx"]) >= n:
            break
        try:
            p = json.load(urllib.request.urlopen(base + "/data", timeout=2)).get("pose")
            if p and p.get("raw"):
                r = p["raw"]
                acc["tx"].append(r["tx"]); acc["ty"].append(r["ty"]); acc["tz"].append(r["tz"])
                acc["yaw"].append(_raw_yaw(r))
        except Exception:
            pass
        time.sleep(0.05)
    if len(acc["tx"]) < 3:
        sys.exit("no pose frames — is the phone streaming (Record3D USB live)?")
    return {k: st.fmean(v) for k, v in acc.items()}


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    base = _base()
    print(f"feed: {base}\n")
    input("STEP 1/3 — rover at start, hold still, press Enter...")
    a = capture(base)
    input("STEP 2/3 — push the rover STRAIGHT FORWARD ~1 m (no turning), press Enter...")
    b = capture(base)
    input("STEP 3/3 — back at start, ROTATE ~90° to its LEFT (CCW) in place, press Enter...")
    c = capture(base)

    dtx, dtz = b["tx"] - a["tx"], b["tz"] - a["tz"]
    if max(abs(dtx), abs(dtz)) < 0.05:
        sys.exit(f"barely moved (Δtx={dtx:+.3f} Δtz={dtz:+.3f}) — push a full metre and retry.")
    fwd = "tx" if abs(dtx) > abs(dtz) else "tz"
    lat = "tz" if fwd == "tx" else "tx"
    fwd_sign = 1 if (b[fwd] - a[fwd]) >= 0 else -1
    dyaw = _wrap(c["yaw"] - a["yaw"])
    yaw_sign = 1 if dyaw >= 0 else -1
    calib = {"fwd": fwd, "fwd_sign": fwd_sign, "lat": lat, "lat_sign": 1, "yaw_sign": yaw_sign}

    print(f"\nforward Δ: tx={dtx:+.3f} tz={dtz:+.3f}  ->  forward = {'+' if fwd_sign>0 else '-'}{fwd}")
    print(f"turn Δyaw: {dyaw:+.3f} rad ({dyaw*57.3:+.0f}°)  ->  yaw_sign = {yaw_sign}")
    print(f"\nPOSTing calibration: {calib}")
    req = urllib.request.Request(base + "/calib", data=json.dumps(calib).encode(),
                                 method="POST", headers={"Content-Type": "application/json"})
    print("applied:", urllib.request.urlopen(req, timeout=3).read().decode())
    print("\nNow open the map and check: forward→up, left-turn→CCW, walls on the correct side.")
    print("If walls land MIRRORED left/right, run:")
    print(f"  curl -X POST {base}/calib -d '{{\"scan_sign\":-1}}'")


if __name__ == "__main__":
    main()
