"""Validate a recorded capture session BEFORE training DN-Splatter on the GPU box.

This is the calibration tool for server/phone_record_demo.py. It reads a recorded session
folder (the exact files DN-Splatter will read: transforms.json + images/ + depths/) and fuses
every keyframe back into ONE colored 3D point cloud using the recorded poses + depths +
intrinsics. That single fusion exercises all five capture assumptions at once:

  depth units, intrinsics resolution, RGB channel order, depth<->RGB alignment, pose convention

If the fused cloud is crisp and at the right scale, the dataset is good and the splat will
train cleanly. If it smears / doubles / is the wrong size, the printed diagnostics point at
which assumption is off. No GPU and no phone needed - it runs on the recorded files.

Usage:
    cd navigation
    venv/bin/python perception/validate_dataset.py ../captures/livingroom/session_01
    # then open fused.ply (MeshLab / CloudCompare / preview) and sanity-check it:
    #   * does it look like the room, at real scale (measure a known 1 m object)?
    #   * are walls vertical and the floor flat (pose pitch/roll correct)?
    #   * are surfaces sharp, not smeared into a double image (poses consistent)?
    #   * are colors right (not red/blue swapped)?
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from perception.pointcloud import depth_to_points_rgb_6dof  # noqa: E402


def _write_ply(path: Path, pts: np.ndarray, cols: np.ndarray) -> None:
    """Write a binary little-endian colored point cloud (x,y,z float + r,g,b uchar)."""
    n = pts.shape[0]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                             ("r", "u1"), ("g", "u1"), ("b", "u1")])
    rec["x"], rec["y"], rec["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    rec["r"], rec["g"], rec["b"] = cols[:, 0], cols[:, 1], cols[:, 2]
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(rec.tobytes())


def _check_assumptions(doc: dict, depth_med_m: float, M0: np.ndarray) -> None:
    """Print the deductions that catch each capture assumption going wrong."""
    w, h = doc["w"], doc["h"]
    cx, cy = doc["cx"], doc["cy"]
    print("\n=== calibration checks ===")
    # 1. Depth units: a roomy indoor median should be ~0.5..6 m. Way off => unit error.
    flag = "" if 0.2 <= depth_med_m <= 8.0 else "  <-- SUSPECT (not a metric room range)"
    print(f"depth: median {depth_med_m:.2f} m{flag}")
    if depth_med_m > 50:
        print("       looks like millimeters survived as meters -> fix depth_scale_mm in record_dataset.py")
    # 2. Intrinsics resolution: principal point should sit near the image center.
    rx, ry = cx / w, cy / h
    flag = "" if (0.35 < rx < 0.65 and 0.35 < ry < 0.65) else "  <-- SUSPECT (cx/cy not near center)"
    print(f"intrinsics: image {w}x{h}, principal point ({cx:.0f},{cy:.0f}) = "
          f"({rx:.2f},{ry:.2f}) of frame{flag}")
    if not (0.35 < rx < 0.65):
        print("       intrinsics may be at the wrong resolution -> check the depth->RGB scaling")
    # 3. Pose convention: rotation must be a proper, right-handed rotation.
    R = M0[:3, :3]
    det = float(np.linalg.det(R))
    ortho = float(np.linalg.norm(R @ R.T - np.eye(3)))
    flag = "" if abs(det - 1.0) < 1e-2 and ortho < 1e-2 else "  <-- SUSPECT (not a clean rotation)"
    print(f"pose[0]: det(R)={det:+.3f} (want +1), orthonormality err={ortho:.1e}{flag}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", help="a session_NN folder containing transforms.json")
    ap.add_argument("--stride", type=int, default=4, help="depth-pixel subsample (smaller=denser)")
    ap.add_argument("--voxel", type=float, default=0.02, help="dedup voxel size (m)")
    ap.add_argument("--max-range", type=float, default=6.0, help="drop depth beyond this (m)")
    ap.add_argument("--out", default=None, help="output PLY (default: <session>/fused.ply)")
    args = ap.parse_args()

    session = Path(args.session).expanduser().resolve()
    doc = json.loads((session / "transforms.json").read_text())
    K = np.array([[doc["fl_x"], 0, doc["cx"]],
                  [0, doc["fl_y"], doc["cy"]],
                  [0, 0, 1.0]], dtype=np.float32)
    depth_scale = float(doc.get("depth_unit_scale_factor", 0.001))  # png unit -> meters
    frames = doc["frames"]
    print(f"session {session.name}: {len(frames)} frames, image {doc['w']}x{doc['h']}")

    voxel = {}  # voxel key -> (x,y,z,r,g,b); first point per cell, dedup across frames
    depth_meds = []
    M0 = None
    for i, fr in enumerate(frames):
        rgb = cv2.imread(str(session / fr["file_path"]))[..., ::-1]  # BGR->RGB
        d = cv2.imread(str(session / fr["depth_file_path"]), cv2.IMREAD_UNCHANGED)
        depth_m = d.astype(np.float32) * depth_scale  # png(mm) -> meters
        M = np.array(fr["transform_matrix"], dtype=np.float32)
        if M0 is None:
            M0 = M
        nz = depth_m[depth_m > 0]
        if nz.size:
            depth_meds.append(float(np.median(nz)))
        pts, cols = depth_to_points_rgb_6dof(
            depth_m, K, M, rgb, stride=args.stride, max_range_m=args.max_range,
            confidence=None,  # depth was already confidence-masked at record time (zeros)
        )
        for j in range(0, len(pts) - 2, 3):
            x, y, z = pts[j], pts[j + 1], pts[j + 2]
            key = (round(x / args.voxel), round(y / args.voxel), round(z / args.voxel))
            if key not in voxel:
                voxel[key] = (x, y, z, cols[j], cols[j + 1], cols[j + 2])

    if not voxel:
        print("no points produced - check that depths are nonzero and poses are valid")
        return
    arr = np.array(list(voxel.values()), dtype=np.float32)
    pts, cols = arr[:, :3], arr[:, 3:6].astype(np.uint8)
    out = Path(args.out) if args.out else session / "fused.ply"
    _write_ply(out, pts, cols)

    lo, hi = pts.min(axis=0), pts.max(axis=0)
    _check_assumptions(doc, float(np.median(depth_meds)) if depth_meds else 0.0, M0)
    print("\n=== fused cloud ===")
    print(f"points: {len(pts)}  (voxel {args.voxel} m, stride {args.stride})")
    print(f"extent: x {hi[0]-lo[0]:.2f} m, y {hi[1]-lo[1]:.2f} m, z {hi[2]-lo[2]:.2f} m  "
          "(does this match the real room size?)")
    print(f"wrote {out}")
    print("Open it and eyeball: real scale? walls vertical? surfaces sharp (not doubled)? colors right?")


if __name__ == "__main__":
    main()
