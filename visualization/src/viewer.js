/*
 * viewer.js - render the live map onto the 2D canvas.
 *
 * Draws one MapUpdate per frame: the occupancy grid, the robot pose, and the planned
 * return path. This is rung 1 of the visualization (2D top-down). The 3D point-cloud
 * upgrade later reuses the same window.__onMapUpdate hook.
 *
 * Message contract: ../../docs/message-schemas.md (MapUpdate). Field names match the
 * Python and C++ sides exactly: width, height, resolution_m, origin, cells, pose,
 * return_path. cells is row-major (index = row * width + col), row 0 at the bottom,
 * values -1 unknown / 0 free / 100 occupied.
 */

const canvas = document.getElementById("map");
const hud = document.getElementById("hud");
const ctx = canvas.getContext("2d");

// Colors for the three cell states, plus the pose and path overlays.
const COLOR = {
  unknown: "#222831",
  free: "#cfd8dc",
  occupied: "#e74c3c",
  path: "#2ecc71",
  pose: "#f1c40f",
  poseStroke: "#7a5c00",
};

// Keep the canvas backing store matched to its CSS size and the device pixel ratio,
// so the map stays crisp on a phone screen. Returns the CSS pixel size to draw within.
function fitCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || window.innerWidth;
  const cssH = canvas.clientHeight || window.innerHeight;
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); // draw in CSS pixels from here on
  return { w: cssW, h: cssH };
}

// Compute the placement that fits the whole grid into the canvas, centered, with a
// uniform scale (pixels per cell). Everything else derives from this.
function computeView(update, cssW, cssH) {
  const margin = 16;
  const scale = Math.min(
    (cssW - 2 * margin) / update.width,
    (cssH - 2 * margin) / update.height
  );
  const gridW = update.width * scale;
  const gridH = update.height * scale;
  return {
    scale,
    offsetX: (cssW - gridW) / 2,
    offsetY: (cssH - gridH) / 2,
    width: update.width,
    height: update.height,
    resolution_m: update.resolution_m,
    origin: update.origin,
  };
}

// World meters -> screen pixels. Screen y is flipped (world y up, screen y down).
function worldToScreen(wx, wy, view) {
  const col = (wx - view.origin.x) / view.resolution_m;
  const row = (wy - view.origin.y) / view.resolution_m;
  return {
    x: view.offsetX + col * view.scale,
    y: view.offsetY + (view.height - row) * view.scale,
  };
}

function drawCells(update, view) {
  const { scale, offsetX, offsetY, width, height } = view;
  for (let r = 0; r < height; r++) {
    for (let c = 0; c < width; c++) {
      const v = update.cells[r * width + c];
      if (v === -1) ctx.fillStyle = COLOR.unknown;
      else if (v >= 100) ctx.fillStyle = COLOR.occupied;
      else ctx.fillStyle = COLOR.free;
      // Row 0 is the bottom in the world, so flip the row onto screen rows.
      const sx = offsetX + c * scale;
      const sy = offsetY + (height - 1 - r) * scale;
      ctx.fillRect(sx, sy, scale + 0.5, scale + 0.5); // +0.5 avoids seam gaps
    }
  }
}

function drawPath(update, view) {
  const path = update.return_path || [];
  if (path.length < 2) return;
  ctx.strokeStyle = COLOR.path;
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.beginPath();
  path.forEach((wp, i) => {
    const p = worldToScreen(wp.x, wp.y, view);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.stroke();
}

function drawPose(update, view) {
  const pose = update.pose;
  if (!pose) return;
  const p = worldToScreen(pose.x, pose.y, view);
  const size = Math.max(8, view.scale * 3);

  ctx.save();
  ctx.translate(p.x, p.y);
  // World theta is CCW from +x; screen y is flipped, so negate to draw correctly.
  ctx.rotate(-pose.theta);
  ctx.beginPath();
  ctx.moveTo(size, 0); // nose, pointing along heading
  ctx.lineTo(-size * 0.6, size * 0.6);
  ctx.lineTo(-size * 0.6, -size * 0.6);
  ctx.closePath();
  ctx.fillStyle = COLOR.pose;
  ctx.fill();
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = COLOR.poseStroke;
  ctx.stroke();
  ctx.restore();
}

/*
 * Render one MapUpdate frame. This is the function ws_client (and the fake feed) call.
 *
 * Input: update - a MapUpdate object (see the contract referenced above).
 * Output: none. Draws to the #map canvas and updates the HUD text.
 */
export function renderMap(update) {
  const { w, h } = fitCanvas();
  ctx.clearRect(0, 0, w, h);

  const view = computeView(update, w, h);
  drawCells(update, view);
  drawPath(update, view);
  drawPose(update, view);

  const knownCells = update.cells.filter((v) => v !== -1).length;
  const pct = ((knownCells / update.cells.length) * 100).toFixed(0);
  hud.textContent =
    `Recon Rover - ${update.width}x${update.height} @ ${update.resolution_m} m/cell - ` +
    `mapped ${pct}% - path ${update.return_path.length} pts`;
}

// The single entry point ws_client.js / the fake feed call for every frame.
window.__onMapUpdate = renderMap;

// Redraw on resize using the most recent frame so the map stays fitted to the screen.
let lastUpdate = null;
const _orig = renderMap;
window.__onMapUpdate = (update) => {
  lastUpdate = update;
  _orig(update);
};
window.addEventListener("resize", () => {
  if (lastUpdate) _orig(lastUpdate);
});
