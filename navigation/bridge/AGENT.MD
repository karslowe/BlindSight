# AGENT.md — Navigation pipeline build (Steps 1–3)

Hackathon POC. ~24h. You own the **software pipeline** on the Mac (Steps 1–3); your teammate
owns the **iPhone→Dragonwing USB bring-up** (Step 0) + the physical sensor mount, in
parallel. Your work is fully testable with **no hardware** — that's the point of the split.

> **Update (autonomy):** the mission is now fully autonomous — no human driving. Frontier
> exploration (`planning/explorer.py` + `planning/pathfind.py`) drives the rover into the
> unknown on its own; when the space is covered it hands off to `planning/return_planner.py`
> to drive home. Both are implemented and verified against a synthetic room
> (`server/autonomy_demo.py`). The mapping pipeline below (Steps 1–3) is unchanged and is
> still the foundation the autonomy sits on.

## The goal we agreed on

Navigate using: **iPhone (pose + forward depth) + 3 TOF sensors (side coverage)**.

- **Pose authority = ARKit (the iPhone).** We are NOT implementing visual-inertial
  odometry. ARKit gives us a world-class metric pose for free over the Record3D stream.
  `slam/slam_frontend.py` becomes a thin _wrapper_ that reads that stream and emits our
  existing `Pose` — it computes nothing.
- **3 TOF (Modulino Distance)** at fixed angles **−70° / +70° / +90°**
  (`[-1.22, 1.22, 1.57]` rad) give the side coverage the phone's forward depth wedge
  (~60–70° forward) is blind to. "Wide shoulders + one side" — chosen to complement the
  phone with **zero redundancy** and to map walls as the rover passes them (turns a corridor
  of forward readings into a real floor-plan). The angles are intentionally NOT symmetric or
  forward-facing — do not "fix" them to ±45°/0°.
- **The occupancy grid is the fusion layer.** It does not know or care which sensor a
  reading came from — it consumes rays in the rover frame. All readings are stamped against
  the ARKit pose.

```
iPhone (ARKit) --USB--> Dragonwing
   ├─ pose          → slam_frontend wraps it → Pose      (single pose authority)
   └─ depth wedge   → sampled to (angle,range) ─┐
                                                 ├─→ occupancy_grid.update(pose, scan)
3× Modulino TOF --Qwiic/I²C--> Dragonwing ───────┘   (−70° / +70° / +90°)
                                                       │
                                  server → MapUpdate → phone browser (Wi-Fi, separate link)
                                                       │
                                       planner → DRV → car (firmware watchdog brakes on silence)
```

## Confirmed hardware decisions

- **3× Modulino Distance modules** (confirmed). Qwiic chaining is handled by the Modulino
  library — **no XSHUT juggling, no TCA9548A mux.** Daisy-chain the Qwiic connectors.
- **TOF angles: −70° / +70° / +90°** = `[-1.22, 1.22, 1.57]` rad. These are _nominal_; the
  real mounted angles get **measured and typed into the code** (see Mounting below).
- **Pose authority: ARKit.** **Link: Record3D over USB** (lowest latency/jitter; keeps the
  rover Wi-Fi free for the viz phone).
- **+90° side beam is on the \_\_\_ side** (FILL IN — left or right). Put it on the _open_ side
  of the demo space so you map the room, not the wall you're already hugging.

## Read these first (don't skip — they ARE the contract)

- `docs/message-schemas.md` — the message contract. Source of truth. Do not rename fields.
- `shared/schemas/schemas.py` — the typed Python mirror you'll import (`Pose`, `MapUpdate`,
  `Waypoint`, `DriveCommand`, `CarTelemetry`). Field names match firmware + JS exactly.
- `navigation/main.py` — the orchestrator. The `tick()` loop is where everything wires
  together.
- `docs/architecture.md` — the why (edge / no-network; the failsafe; the watchdog).

## The scan contract (the key decision — already made)

`occupancy_grid.update()` currently takes a **single scalar** range and ray-casts straight
ahead. We are generalizing it to a **batched scan** so 3 TOF beams + a phone depth wedge
fuse in one atomic update per tick.

**A "scan" is a list of `(angle_offset, range_m)` tuples** where:

- `angle_offset` = **radians relative to the rover heading** (`pose.theta`), CCW positive.
  So the 3 TOF are `[-1.22, 1.22, 1.57]`; the phone depth wedge contributes a fan of samples
  across its FoV.
- `range_m` = meters to the hit. For "no return / max range", pass the max range and treat
  it as free-along-the-ray with no occupied cell (sentinel — pick a convention, document it
  in the docstring).

Calibration (where each sensor points) lives in the **caller** (the orchestrator), NOT in
the grid. The grid stays sensor-agnostic.

## Your steps, in dependency order

### Step 1 — Generalize the occupancy grid [START HERE — pure software, no hardware]

File: `navigation/mapping/occupancy_grid.py`

Change the signature:

```python
# from:
def update(self, pose: Pose, range_m: Optional[float]) -> None
# to:
def update(self, pose: Pose, scan: list[tuple[float, float]]) -> None
#   scan = [(angle_offset_rad, range_m), ...]
```

Implement a real **growable** grid (numpy `int8`, already a dep; init cells to `-1`):

- For each `(angle_offset, range_m)`: ray = `pose.theta + angle_offset`. Ray-cast from the
  pose: mark cells along the ray **free (0)**, the hit cell **occupied (100)**. A log-odds
  update is nicer but a simple last-write or small hit/miss counter is fine for the POC —
  don't gold-plate it.
- **Grow the grid** when a ray exits current bounds. Keep `origin` correct in the map frame
  as you grow — the viz relies on it.
- `to_map_update()` already has the right shape — just flatten your numpy backing store to a
  row-major `list[int]` for `cells` (see the existing TODO ~line 60).

**Test it NOW, no hardware:** drive synthetic scans through it and watch cells fill in the
real browser viz:

```bash
cd navigation
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
python server/demo_broadcast.py      # then open http://localhost:8000/?live
```

`demo_broadcast.py` is pure Python (runs on Mac). Adapt it (or a small scratch script) to
build a grid, call `grid.update(pose, synthetic_scan)` each tick, and
`server.publish(grid.to_map_update(pose, []))`. When the room fills in from a few beams,
Step 1 is done. **This is the first visible win and it gates everything else.**

> NOTE: `bridge/fake_car.py` is **Unix-only** (`pty`/`select`) — runs on your Mac, not
> Windows. It accepts `DRV` and streams `TEL` over a virtual serial port; use it for the
> car-link path. For Step 1, the `demo_broadcast.py` path is enough.

### Step 2 — Read the 3 TOF [needs the Modulino hardware; stub until then]

File: `navigation/bridge/modulino_io.py`

`read_distance()` currently returns one float for one ToF. We have three. Make it read all
three (e.g. `read_distances() -> list[float]`). Qwiic chaining handled by the Modulino lib.

Until the hardware is in hand, **return synthetic ranges** so the orchestrator runs. Then in
`main.py`'s `tick()`, build the TOF part of the scan:

```python
TOF_ANGLES = [-1.22, 1.22, 1.57]   # rad: -70° / +70° / +90° (wide shoulders + side)
tof_scan = [(a, r) for a, r in zip(TOF_ANGLES, self.imu.read_distances()) if r is not None]
```

**The list order must match the Qwiic chain order** — index 0 of `read_distances()` must be
the module physically at −70°, etc. If the map comes out mirrored, the order is swapped —
fix the list, not the hardware.

### Step 3 — Wrap Record3D as pose + depth [needs Step 0 green from teammate]

File: `navigation/slam/slam_frontend.py`

`process(frame, imu)` keeps its signature but the **body becomes a wrapper**: read the
latest ARKit pose + depth frame from the Record3D stream and return our `Pose`
(`shared/schemas/schemas.py` ~line 140). On the depth frame, sample the wedge into a handful
of `(angle_offset, range_m)` pairs and hand them to the orchestrator to merge into the scan.

Do NOT start the real wrapper until your teammate confirms Record3D's USB stream reaches the
Dragonwing (their Step 0). Until then, stub `process()` to return a synthetic moving `Pose`
so Steps 1–2 integrate.

### Step 4 — Integrate (both of you)

Orchestrator `tick()` assembles ONE scan per tick = `tof_scan + depth_scan`, all stamped to
the ARKit pose, and calls `grid.update(pose, scan)` once. Then car driving via
`bridge/drive_test.py`, then the full loop into the phone browser.

## Mounting the TOF (teammate's job, ~30 min — NO 3D PRINT)

Precision lives in software, not the mount. The bracket only has to hold each sensor
**rigidly at a repeatable angle**; the _value_ is whatever you measure and type in.

1. **Position** each Modulino: −70° and +70° at the two **front corners** angled outward;
   +90° on the chosen **side edge** pointing straight out. (Tack with putty first to fiddle.)
2. **Commit** with **hot glue** to the acrylic deck / corners once happy. Rigid in seconds.
   (Foamboard angle gussets or kit standoffs/brackets are fine alternatives. Don't demo on
   putty alone — it creeps and the angle silently drifts, corrupting the map.)
3. **Measure the real beam angle** of each sensor vs. the rover's forward axis (phone
   protractor app, or a printed protractor taped to the deck). Write all three down.
4. **Type the measured angles into `TOF_ANGLES`** in `main.py` — these REPLACE the nominal
   `[-1.22, 1.22, 1.57]`. A bracket 8° off is a non-issue if you measured 8° and entered it.
5. **Verify in the viz:** drive next to a wall on each side; confirm occupied cells appear in
   the direction the sensor actually points. Wrong side → chain-index↔angle mapping swapped
   (fix the list order, not the glue).

Only print a mount if a printer is **already free + idle teammate + models in <30 min**, and
even then run it in the background as an upgrade — the glued mount is the working one. Never
let a print become the critical path.

## Rules / gotchas (these will bite)

- **Timestamp discipline.** Every reading fuses against the pose at _its own_ timestamp, not
  arrival time. The schema mandates float-seconds timestamps everywhere. Mismatched timing =
  a smeared map.
- **One pose authority = ARKit.** Do NOT also run an IMU odometry and fuse two poses — two
  disagreeing estimates tear the grid apart. The Modulino IMU is redundant for pose now;
  keep it only as a sanity cross-check, or drop it from the critical path.
- **Stale-frame safety.** If the phone stream stalls (USB/Wi-Fi hiccup, ARKit relocalizing,
  thermal throttle), do NOT act on a frame you think is current. Timeout → stop/slow. The
  firmware already brakes on serial silence; mirror that on the depth stream.
- **Don't rename schema fields.** Change `docs/message-schemas.md` first, then mirror to
  `shared/schemas/schemas.py` and the JSON Schema. C++ and JS depend on exact names.
- **Calibration lives in the caller.** The grid takes rays in the rover frame. Where each
  sensor points is the orchestrator's job.
- **Don't gold-plate the grid.** Simple hit/miss is fine for a POC. The win is the system
  running front-to-back, not a perfect mapper.

## Priority order (be ruthless)

1. Step 1 working end-to-end in the browser. Nothing else matters if this doesn't run.
2. Steps 2–3 with synthetic data so the orchestrator ticks with the real shapes.
3. Real hardware swapped in as Step 0 goes green and the mount is done.
4. Leave buffer — hardware integration always overruns.

## Open items to close (ping the team when known)

- [ ] **Record3D USB stream reaches the Dragonwing (Linux)** — teammate's Step 0, the one
      load-bearing unknown. If it fails, fall back to Wi-Fi streaming.
- [ ] **Measured TOF angles** replace the nominal `[-1.22, 1.22, 1.57]` once mounted.
- [ ] **Which side the +90° beam is on** — fill into the "Confirmed hardware decisions"
      section and keep the `TOF_ANGLES` order matched to the Qwiic chain.
