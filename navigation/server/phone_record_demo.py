"""Record a walk-around from the phone into a Nerfstudio/DN-Splatter dataset (offline splat).

The CAPTURE step of the photorealistic reconstruction path. The rover (or a handheld phone)
streams RGBD + ARKit pose over Record3D; this records posed RGB + LiDAR-depth keyframes to
disk. NOTHING is processed in real time here - you later train DN-Splatter on the GPU box
against the folder this writes. See docs/reconstruction.md for the offline training steps.

Unlike server/phone_cloud_demo.py (which projects + publishes a live cloud), this has no
viewer and no projection: it just selects keyframes and writes files. That is the whole job.

Prereqs (same as the cloud demo):
  - Record3D iOS app on an iPhone/iPad Pro with LiDAR, USB Streaming purchased + enabled.
  - phone connected by USB-C (recommended for the heavy RGBD stream).
  - In the venv:  pip install record3d   (opencv-python + numpy are already required)

Usage:
    cd navigation
    python server/phone_record_demo.py --out ../captures/livingroom
    # walk the space slowly + steadily, overlapping views; Ctrl-C when done.
    # then on the GPU box:  ns-train dn-splatter --data captures/livingroom/session_01
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.phone_link import PhoneLink  # noqa: E402
from perception.record_dataset import DatasetRecorder  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True, help="output dataset root (sessions written inside)")
    ap.add_argument("--trans", type=float, default=0.05, help="keyframe translation threshold (m)")
    ap.add_argument("--rot", type=float, default=5.0, help="keyframe rotation threshold (deg)")
    ap.add_argument("--conf", type=int, default=2, choices=(0, 1, 2),
                    help="min ARKit depth confidence to keep (2=high-only, the phantom-dot fix)")
    args = ap.parse_args()

    recorder = DatasetRecorder(
        out_root=Path(args.out).expanduser().resolve(),
        trans_thresh_m=args.trans, rot_thresh_deg=args.rot, depth_conf_min=args.conf,
    )

    link = PhoneLink()
    print("connecting to phone (Record3D)... make sure USB Streaming is on")
    link.connect()
    print(f"connected. recording keyframes to {recorder.out_root}")
    print("Walk the space slowly with overlapping views. Ctrl-C to finish + write transforms.json.")

    frames = 0
    last_log = 0.0
    try:
        while True:
            try:
                frame = link.read()
                if frame is None:
                    time.sleep(0.005)  # no new phone frame yet; don't busy-spin
                    continue
                recorder.on_epoch(link.epoch)  # rolls to a new session on a reconnect/new origin
                if recorder.add(frame):
                    pass  # kept a keyframe
                frames += 1
                now = time.monotonic()
                if now - last_log >= 2.0:
                    last_log = now
                    print(f"  {frames} frames seen, {recorder.total_kept} keyframes kept "
                          f"(cam xyz {frame.extrinsic[0,3]:+.2f},"
                          f"{frame.extrinsic[1,3]:+.2f},{frame.extrinsic[2,3]:+.2f})")
            except Exception:
                # One bad frame must not kill a long capture; log and keep going.
                print("[record] frame error (continuing):")
                traceback.print_exc()
                time.sleep(0.1)
    except KeyboardInterrupt:
        recorder.close()
        print(f"\nstopped: {recorder.total_kept} keyframes across "
              f"{recorder._session_idx} session(s) in {recorder.out_root}")


if __name__ == "__main__":
    main()
