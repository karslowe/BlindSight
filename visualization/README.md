# visualization

Role: the live map viewer. A static web app that connects to the rover and renders the
occupancy grid, the robot pose, and the planned return path as they update.

Runs on: the phone's browser. Served statically from the rover (the UNO Q web server). The
phone joins the rover's own Wi-Fi access point, so no internet is needed.

Tech stack: plain HTML and JavaScript. Three.js for rendering (point cloud later, a 2D
canvas grid to start). No build step is required; a bundler is optional.

## Build and run

Two options:

1. Plain static (recommended for the hackathon). No build. The rover server serves
   `index.html` and `src/`. For local dev, just open [index.html](index.html) in a
   browser, or run any static server:

   ```bash
   python3 -m http.server 8000   # then open http://localhost:8000
   ```

   In plain mode, load Three.js from a CDN in `index.html` (see the commented tag there)
   instead of installing it.

2. Bundled (optional). If you prefer a bundler, install deps and build:

   ```bash
   npm install
   npm run build      # outputs to dist/, which the rover then serves
   ```

   See [package.json](package.json).

## Files

- [index.html](index.html): the page shell and canvas.
- [src/viewer.js](src/viewer.js): renders the occupancy grid, pose, and path.
- [src/ws_client.js](src/ws_client.js): connects to the rover websocket and feeds updates.

## 3D view (optional, additive)

A 3D version lives at `/3d.html` ([3d.html](3d.html) + [src/viewer3d.js](src/viewer3d.js))
and renders the **same `MapUpdate` stream** with Three.js: a floor, occupied cells extruded
into walls, plus the rover, START, targets, and route home. It reuses `ws_client.js`
unchanged and touches no backend or contract, so the 2D view at `/` is unaffected.

```bash
cd ../navigation && python server/autonomy_demo.py
# 2D:  http://localhost:8000/?live
# 3D:  http://localhost:8000/3d.html?live   (drag to orbit, scroll to zoom, right-drag to pan)
```

Three.js loads from a CDN via the import map in `3d.html` (no build step; needs internet).

### Depth-based point cloud (the real 3D)

The 3D view shows a **point cloud** when `MapUpdate.point_cloud` is non-empty (a flat
`[x,y,z,...]` map-frame list), and falls back to extruded walls otherwise. The demo produces
a **synthetic** cloud today (`server/autonomy_demo.py`, accumulated as the rover explores).
When the phone (Record3D) is streaming real depth, swap the synthetic producer for the real
one - `navigation/perception/pointcloud.py::depth_to_points(depth, intrinsics, pose)` - and
the same contract and viewer render the real reconstruction unchanged. Navigation never uses
the point cloud; it is visualization-only.

Note: the demo sends the full cloud each frame for simplicity; in production, throttle it
(send every N frames or only new points) since point clouds are large.

## Message schemas

Defined in [../docs/message-schemas.md](../docs/message-schemas.md). Field names match the
Python and C++ sides exactly.

- Consumes: `MapUpdate` over the websocket (width, height, resolution_m, origin, cells,
  pose, return_path).
- Produces: a single "return home" command sent back over the same websocket.
