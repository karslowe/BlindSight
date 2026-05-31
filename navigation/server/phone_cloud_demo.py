"""Walk-around 3D test: stream the phone's LiDAR depth as a live 3D point cloud.

Hold the phone and walk around a room; the 3D viewer fills in with a real point cloud built
from the phone's depth. This is the REAL phone path (unlike server/autonomy_demo.py, which
uses a synthetic cloud): it connects over Record3D, back-projects each depth frame to
map-frame 3D points (perception.pointcloud.depth_to_points), accumulates them (voxel-deduped),
and publishes them to the same /3d.html viewer over the same MapUpdate contract.

This is ALSO the first place phone_link touches a real device, so the first run is the
calibration moment ("Step 0"). Expect to confirm/adjust two things - see CALIBRATION below.

Prereqs (the part this script can't do for you):
  - Record3D iOS app on an iPhone/iPad Pro with LiDAR, with the USB Streaming purchase.
  - Enable "USB Streaming" in the Record3D app settings, set it streaming.
  - Connect the phone to this machine by USB-C (recommended for the heavy RGBD stream).
  - In the venv:  pip install record3d

Usage:
    cd navigation
    python server/phone_cloud_demo.py        # then open http://localhost:8000/3d.html?live

CALIBRATION (first run, on a real device):
  1. DEPTH UNITS: depth_to_points assumes meters. If the cloud looks 1000x too big/small,
     the phone is sending millimeters - scale the depth by 0.001 in read() / here.
  2. POSE AXES: bridge/phone_link.py::_pose_from_camera maps the phone's AR pose onto our
     ground plane with an assumed mounting. If walking forward moves the cloud sideways, or
     turning spins it the wrong way, fix the axis/sign mapping there. The cloud is rendered
     from the pose, so this is where "the map drifts as I walk" gets fixed.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.phone_link import PhoneLink  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from perception.pointcloud import depth_to_points  # noqa: E402

# Sub-sample + range band for back-projection (fewer points = less bandwidth, smoother viz).
STRIDE = 8
MIN_RANGE_M = 0.2
MAX_RANGE_M = 4.0
# Accumulation: dedup points onto a voxel grid so revisiting a spot does not pile up points.
VOXEL_M = 0.04
MAX_POINTS = 120000  # cap the accumulated cloud (the viewer also caps what it draws)


def _accumulate(cloud: dict, pts: list) -> None:
    """Fold this frame's flat [x,y,z,...] points into the voxel-deduped accumulated cloud."""
    for i in range(0, len(pts) - 2, 3):
        x, y, z = pts[i], pts[i + 1], pts[i + 2]
        key = (round(x / VOXEL_M), round(y / VOXEL_M), round(z / VOXEL_M))
        if key not in cloud:
            if len(cloud) >= MAX_POINTS:
                return
            cloud[key] = (x, y, z)


def _flat(cloud: dict) -> list:
    out: list = []
    for x, y, z in cloud.values():
        out.append(x)
        out.append(y)
        out.append(z)
    return out


def main() -> None:
    try:
        from app import MapServer  # when run as: python server/phone_cloud_demo.py
    except ImportError:  # pragma: no cover
        from server.app import MapServer

    link = PhoneLink()
    print("connecting to phone (Record3D)... make sure USB Streaming is on")
    link.connect()  # raises a clear error if the dep / device / app is missing
    print("connected.")

    grid = OccupancyGrid(resolution_m=0.05, max_range_m=6.0)
    server = MapServer()
    server.run_in_thread(port=8000)
    print("Serving on http://localhost:8000  (open /3d.html?live)")
    print("Walk around holding the phone; the 3D cloud should fill in. Ctrl-C to stop.")

    cloud: dict = {}
    start = None
    frames = 0
    try:
        while True:
            frame = link.read()
            if frame is None:
                time.sleep(0.01)  # no new phone frame yet; do not busy-spin
                continue
            if start is None:
                start = frame.pose
                grid._init_grid(start)  # seed floor bounds so the viewer has a ground plane
                print(f"first frame: depth {frame.depth.shape}, "
                      f"range {float(frame.depth.min()):.2f}..{float(frame.depth.max()):.2f} "
                      f"(if that range looks like mm, see CALIBRATION in this file)")
            pts = depth_to_points(
                frame.depth, frame.intrinsics, frame.pose,
                stride=STRIDE, min_range_m=MIN_RANGE_M, max_range_m=MAX_RANGE_M,
            )
            _accumulate(cloud, pts)
            home = {"x": start.x, "y": start.y}
            # 2D map stays empty here (this test is the 3D cloud only); pose + cloud carry it.
            server.publish(grid.to_map_update(frame.pose, [], [], home, _flat(cloud)))
            frames += 1
            if frames % 30 == 0:
                print(f"  {frames} frames, {len(cloud)} points")
    except KeyboardInterrupt:
        print(f"\nstopped ({frames} frames, {len(cloud)} points)")


if __name__ == "__main__":
    main()
