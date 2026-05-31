"""Walk-around 3D SCAN test: stream the phone's depth as a live triangle mesh (solid surfaces).

Same idea as phone_cloud_demo.py, but instead of loose points it reconstructs a connected
mesh (perception.mesh_reconstruct.DepthMesher) and streams it to /mesh.html, so you see solid
surfaces ("a 3D scan") build up as you walk. Reuses the same Record3D path, the same pose +
projection, and the same MapUpdate transport (the mesh rides in MapUpdate.mesh).

This is a LAPTOP-DEMO tool, not something the rover runs - the reconstruction + mesh streaming
are far heavier than the rover's lightweight 2D map, and the rover does not need surfaces to
navigate. See the note at the bottom of this file.

Prereqs (same as the cloud demo):
  - Record3D iOS app with USB Streaming enabled and streaming.
  - phone connected by USB-C.
  - pip install record3d   (already done if the cloud demo ran)

Usage:
    cd navigation
    python server/phone_mesh_demo.py        # then open http://localhost:8000/mesh.html?live
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.phone_link import PhoneLink  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from perception.mesh_reconstruct import DepthMesher  # noqa: E402

# Meshing is heavier than a point cloud, so publish the (large) mesh only every Nth frame;
# the viewer keeps showing the last one in between. The mesh changes slowly, so this is fine.
PUBLISH_EVERY = 6


def main() -> None:
    try:
        from app import MapServer  # when run as: python server/phone_mesh_demo.py
    except ImportError:  # pragma: no cover
        from server.app import MapServer

    link = PhoneLink()
    print("connecting to phone (Record3D)... make sure USB Streaming is on")
    link.connect()
    print("connected.")

    mesher = DepthMesher(stride=4, voxel_m=0.035)
    grid = OccupancyGrid(resolution_m=0.05, max_range_m=6.0)
    server = MapServer()
    server.run_in_thread(port=8000)
    print("Serving on http://localhost:8000  (open /mesh.html?live)")
    print("Walk around holding the phone; solid surfaces should build up. Ctrl-C to stop.")

    start = None
    frames = 0
    try:
        while True:
            frame = link.read()
            if frame is None:
                time.sleep(0.01)
                continue
            if start is None:
                start = frame.pose
                grid._init_grid(start)  # seed floor bounds so the viewer has a ground plane
                print(f"first frame: depth {frame.depth.shape}, "
                      f"range {float(frame.depth.min()):.2f}..{float(frame.depth.max()):.2f}")
            mesher.add_frame(frame.depth, frame.intrinsics, frame.extrinsic)
            frames += 1
            if frames % PUBLISH_EVERY == 0:
                home = {"x": start.x, "y": start.y}
                server.publish(grid.to_map_update(frame.pose, [], [], home, [], mesher.mesh_dict()))
            if frames % 30 == 0:
                v, f = mesher.stats()
                print(f"  {frames} frames, mesh: {v} verts / {f} tris"
                      + ("  [FULL]" if mesher.full else ""))
    except KeyboardInterrupt:
        v, f = mesher.stats()
        print(f"\nstopped ({frames} frames, {v} verts / {f} tris)")


if __name__ == "__main__":
    main()
