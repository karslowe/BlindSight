# Photorealistic 3D reconstruction (offline)

How we turn the phone's LiDAR + camera into a photorealistic 3D model. This is an **offline**
pipeline that runs on a workstation GPU (a 5070 Ti, here) — it is **not** the rover's live 2D
map and is unrelated to navigation. The rover/phone just *captures*; reconstruction happens
later.

```
   phone (Record3D: RGB + LiDAR depth + ARKit pose)
        │  live USB/WiFi stream
        ▼
   laptop  ──  server/phone_record_demo.py   (records keyframes, no real-time compute)
        │
        ▼
   <out>/session_NN/   (Nerfstudio dataset: images/ + depths/ + transforms.json)
        │  copy to the GPU box
        ▼
   DN-Splatter  (ns-train dn-splatter)  →  photorealistic, metric 3D Gaussian splat (+ optional mesh)
```

## Why this method

The reconstruction is a **3D Gaussian Splat** trained with **DN-Splatter**, a Nerfstudio
method built for iPhone/Record3D RGBD captures. It adds **depth + normal supervision** (from
the LiDAR) on top of photometric splatting, so the result is both photorealistic **and**
geometrically accurate / metric — the LiDAR pins down the geometry that a camera-only splat
guesses at, killing floaters and scale drift.

Two facts make our capture drop in with no pose-solving and no axis surgery:

- **ARKit's camera frame is Nerfstudio's convention.** ARKit camera axes are +X right, +Y up,
  −Z forward, which is exactly the OpenGL/Blender convention Nerfstudio's `transform_matrix`
  uses. So `PhoneFrame.extrinsic` (the *raw* ARKit camera→world, **before** `_world_to_map`)
  is written verbatim as each frame's pose. No COLMAP, no `ns-process-data`.
- **RGB and depth share the camera FOV.** Record3D presents them as one aligned RGBD frame
  (only the resolution differs), so the depth intrinsics scale to RGB resolution by the
  resolution ratio alone. (Same assumption as `perception.pointcloud.sample_rgb_grid`.)

## Step 1 — Capture (on the laptop the rover streams to)

Same Record3D prerequisites as the other phone demos (USB Streaming on, phone unlocked,
`pip install record3d`). Run `check_phone.py` first if unsure the link is up.

```bash
cd navigation
python server/phone_record_demo.py --out ../captures/livingroom
# walk the space slowly, steady, with heavily overlapping views; Ctrl-C when done.
```

Flags: `--trans` / `--rot` set the keyframe spacing (default 5 cm / 5°), `--conf` the minimum
ARKit depth confidence to trust (default 2 = high-only, the phantom-dot fix).

What it writes (`perception/record_dataset.py`), one folder **per ARKit tracking session**:

```
captures/livingroom/session_01/
    transforms.json        # intrinsics (at RGB res) + per-frame camera->world + depth_unit_scale_factor
    images/frame_00001.jpg # RGB at full camera resolution
    depths/frame_00001.png # uint16 millimeters, upsampled to RGB res; low-confidence pixels = 0
```

- **Keyframes, not every frame.** A frame is written only after ~5 cm of translation or ~5° of
  rotation, so views are well distributed and motion-blur duplicates are dropped.
- **Sessions.** A reconnect (phone sleeps, USB hiccups) restarts ARKit with a *new* world
  origin; `PhoneLink.epoch` bumps and the recorder rolls to `session_02`. Poses across that
  boundary are incompatible — **train on a single session folder** (the longest complete walk).

### Capture tips (these decide the quality, more than any flag)
- Move slowly and keep the camera steady; splatting hates motion blur.
- Orbit objects and get many viewpoints of each surface; overlap heavily.
- Even, diffuse lighting. Avoid changing exposure mid-capture.
- Don't trigger a reconnect mid-walk — one continuous session is worth more than three short ones.

## Step 2 — Train (on the GPU box)

The 5070 Ti is **Blackwell (sm_120)** — it needs **CUDA 12.8+** and a matching PyTorch
(cu128) build; DN-Splatter and Nerfstudio also pin specific versions. This install is the
riskiest part — **validate the env on a tiny capture before a long walk-around.** Use a clean
Python 3.11 conda env (also sidesteps the Open3D / Python-3.13 wheel gap).

```bash
# sketch — follow the current DN-Splatter + Nerfstudio install docs for exact pins
conda create -n splat python=3.11 && conda activate splat
pip install torch --index-url https://download.pytorch.org/whl/cu128   # Blackwell needs cu128+
pip install nerfstudio
pip install dn-splatter            # adds the dn-splatter method to ns-train

ns-train dn-splatter --data captures/livingroom/session_01
# our dataset is the Nerfstudio layout with sensor depth already aligned and posed.
```

Then `ns-viewer` to fly through it. To get a textured **mesh** out of the splat (the metric
artifact), use DN-Splatter's mesh export (`gs-mesh` / TSDF or Poisson over the rendered
depths). For a pure-geometry mesh independent of splatting, the same posed RGBD also feeds an
Open3D GPU TSDF — but appearance is the goal here, so DN-Splatter is the primary path.

## Step 1.5 — Validate the capture (no GPU, no phone)

Before hauling the dataset to the GPU box, fuse it back into one point cloud and look at it.
This single check exercises all the assumptions below at once — units, intrinsics, color
order, depth/RGB alignment, and pose convention:

```bash
cd navigation
venv/bin/python perception/validate_dataset.py ../captures/livingroom/session_01
# prints depth/intrinsics/pose checks, writes session_01/fused.ply
```

Open `fused.ply` (MeshLab / CloudCompare / macOS preview) and ask:
- **Real scale?** Measure a known ~1 m object in the cloud. Off by ~1000× ⇒ depth-unit bug.
- **Walls vertical, floor flat?** If not, the pose pitch/roll (extrinsic) is wrong.
- **Surfaces sharp, not doubled?** A smeared/ghosted surface ⇒ inconsistent poses across frames.
- **Colors right (not red/blue swapped)?** ⇒ the RGB→BGR flip.

The printed `=== calibration checks ===` block flags a non-metric depth median, a principal
point that isn't near center (wrong intrinsics resolution), or a non-right-handed rotation.
A crisp, correctly-sized cloud here means DN-Splatter will train cleanly.

## Calibration / gotchas (first real capture)

- **Depth units.** We assume Record3D depth is **meters** and store `meters * 1000` as uint16
  mm (`depth_unit_scale_factor = 0.001`). If the first depth PNGs look ~1000× off, the device
  is already sending mm — adjust `depth_scale_mm` in `record_dataset.py`.
- **Intrinsics resolution.** We treat `frame.intrinsics` as matching the **depth** resolution
  (that is how `perception.pointcloud` uses it) and scale to RGB res. If geometry looks
  consistently sheared/offset, that base assumption is the place to check.
- **RGB color order.** Record3D gives RGB; we flip to BGR for `cv2.imwrite`. If the trained
  splat looks blue/red-swapped, that flip is the suspect.
- **Pose sanity.** `transform_matrix` is the raw ARKit cam→world. If the splat comes out
  mirrored or upside down, double-check that `phone_link._extrinsic_from_camera` is producing
  a right-handed cam→world (it should — `pose_from_extrinsic` derives the nav pose from the
  same matrix).

## Files

- `navigation/perception/record_dataset.py` — `DatasetRecorder`: keyframe selection, depth
  upsample + confidence masking, intrinsics scaling, `transforms.json` writing, session rollover.
- `navigation/server/phone_record_demo.py` — the capture runner (Record3D → dataset on disk).
- `navigation/bridge/phone_link.py` — the live Record3D stream (unchanged; reused as-is).
