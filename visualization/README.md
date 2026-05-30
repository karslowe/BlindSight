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

## Message schemas

Defined in [../docs/message-schemas.md](../docs/message-schemas.md). Field names match the
Python and C++ sides exactly.

- Consumes: `MapUpdate` over the websocket (width, height, resolution_m, origin, cells,
  pose, return_path).
- Produces: a single "return home" command sent back over the same websocket.
