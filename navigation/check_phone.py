"""Quick phone-link sanity check: is the iPhone reaching THIS computer and streaming?

The Record3D USB link is the #1 thing that breaks (phone locks / app backgrounds / a stale
process holds the device). Run this BEFORE the demos to confirm the link is up. It prints how
many devices are visible and how many frames actually arrive in a few seconds.

Usage:
    cd navigation
    venv/bin/python check_phone.py

NOTE: this script connects to the phone, so let it FINISH (it exits after ~5s and releases the
device) before starting phone_cloud_demo.py / main.py. Only one consumer can hold the phone.
"""

from __future__ import annotations

import time

try:
    from record3d import Record3DStream
except ImportError:
    print("record3d not installed. In the venv:  pip install record3d")
    raise SystemExit(1)

devs = Record3DStream.get_connected_devices()
print(f"Record3D devices visible: {len(devs)}")
if not devs:
    print("  -> 0 devices = PHONE SIDE. Fix on the phone, then re-run:")
    print("     1) unlock the phone")
    print("     2) open Record3D, keep it in the FOREGROUND, enable USB Streaming")
    print("     3) Settings > Display & Brightness > Auto-Lock > Never")
    print("     4) reseat the USB-C cable")
    print("  (If it was 1 a moment ago and is now 0, the app backgrounded or the phone locked.)")
    raise SystemExit(1)

frames = {"n": 0}
stream = Record3DStream()
stream.on_new_frame = lambda: frames.__setitem__("n", frames["n"] + 1)
stream.connect(devs[0])
print("connected; counting frames for 5s (wave the phone around a little)...")
time.sleep(5.0)
fps = frames["n"] / 5.0
print(f"frames received in 5s: {frames['n']}  (~{fps:.1f} fps)")
if frames["n"] == 0:
    print("  -> device visible but NO frames: the app isn't actively streaming.")
    print("     Re-toggle USB Streaming in Record3D, keep it foregrounded + phone unlocked, retry.")
else:
    print("  -> OK, the phone is streaming. Run:  python server/phone_cloud_demo.py")
