/*
 * viewer.js - render the live map onto the 2D canvas.
 *
 * Draws one MapUpdate per frame:
 *   - the occupancy grid (cells),
 *   - a faint breadcrumb TRAIL of everywhere the rover has driven (client-side history),
 *   - a START marker at the rover's first seen pose (where it returns to),
 *   - the planned RETURN PATH home (bright green, from return_path),
 *   - the rover pose (oriented marker).
 *
 * The trail and START marker are derived on the client from the stream of poses, so they
 * need no extra fields in the contract and work identically with real rover data.
 *
 * This is rung 1 of the visualization (2D top-down). The 3D point-cloud upgrade later
 * reuses the same window.__onMapUpdate hook.
 *
 * Message contract: ../../docs/message-schemas.md (MapUpdate). cells is row-major
 * (index = row * width + col), row 0 at the bottom, values -1 unknown / 0 free / 100 occupied.
 */

const canvas = document.getElementById("map");
const hud = document.getElementById("hud");
const ctx = canvas.getContext("2d");

const COLOR = {
  unknown: "#222831",
  free: "#cfd8dc",
  occupied: "#e74c3c",
  trail: "rgba(236, 240, 241, 0.35)", // faint: where the rover has been
  path: "#2ecc71", // bright: the planned route home
  pose: "#f1c40f",
  poseStroke: "#7a5c00",
  home: "#1abc9c",
  homeText: "#ecf0f1",
};

// ---- Client-side history derived from the pose stream ----
let home = null; // {x, y} of the first pose seen = the return target
const trail = []; // [{x, y}, ...] breadcrumb of where the rover has driven
const TRAIL_MIN_SPACING_M = 0.08; // only record a point after moving this far

// Let the data source reset history on (re)start of a feed.
window.__resetTrail = () => {
  home = null;
  trail.length = 0;
};

function recordHistory(pose) {
  if (!pose) return;
  if (!home) home = { x: pose.x, y: pose.y };
  const last = trail[trail.length - 1];
  if (!last || Math.hypot(pose.x - last.x, pose.y - last.y) >= TRAIL_MIN_SPACING_M) {
    trail.push({ x: pose.x, y: pose.y });
  }
}

// Keep the canvas backing store matched to its CSS size and the device pixel ratio.
function fitCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || window.innerWidth;
  const cssH = canvas.clientHeight || window.innerHeight;
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: cssW, h: cssH };
}

// Fit the whole grid into the canvas, centered, with a uniform pixels-per-cell scale.
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
      const sx = offsetX + c * scale;
      const sy = offsetY + (height - 1 - r) * scale; // row 0 is the bottom
      ctx.fillRect(sx, sy, scale + 0.5, scale + 0.5);
    }
  }
}

// Faint dashed line through everywhere the rover has driven.
function drawTrail(view) {
  if (trail.length < 2) return;
  ctx.strokeStyle = COLOR.trail;
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 5]);
  ctx.beginPath();
  trail.forEach((wp, i) => {
    const p = worldToScreen(wp.x, wp.y, view);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

// Bright solid line: the planned route home (only present after a return is requested).
function drawReturnPath(update, view) {
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

function drawHome(view) {
  if (!home) return;
  const p = worldToScreen(home.x, home.y, view);
  const r = Math.max(6, view.scale * 1.6);
  ctx.beginPath();
  ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
  ctx.fillStyle = COLOR.home;
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#ffffff";
  ctx.stroke();
  ctx.fillStyle = COLOR.homeText;
  ctx.font = "bold 12px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.fillText("START", p.x, p.y - r - 5);
}

function drawPose(update, view) {
  const pose = update.pose;
  if (!pose) return;
  const p = worldToScreen(pose.x, pose.y, view);
  const size = Math.max(8, view.scale * 3);

  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(-pose.theta); // world theta is CCW from +x; screen y is flipped
  ctx.beginPath();
  ctx.moveTo(size, 0); // nose, along heading
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
 * Render one MapUpdate frame. ws_client (and the fake feed) call this per frame.
 * Input: update - a MapUpdate object. Output: none. Draws to #map and updates the HUD.
 */
export function renderMap(update) {
  recordHistory(update.pose);

  const { w, h } = fitCanvas();
  ctx.clearRect(0, 0, w, h);

  const view = computeView(update, w, h);
  drawCells(update, view);
  drawTrail(view);
  drawReturnPath(update, view);
  drawHome(view);
  drawPose(update, view);

  const knownCells = update.cells.filter((v) => v !== -1).length;
  const pct = ((knownCells / update.cells.length) * 100).toFixed(0);
  const returning = (update.return_path || []).length > 0;
  hud.textContent =
    `Recon Rover - mapped ${pct}% - ` +
    (returning ? "RETURNING to start" : "exploring");
}

// Store the most recent frame so a resize can redraw it at the new size.
let lastUpdate = null;
window.__onMapUpdate = (update) => {
  lastUpdate = update;
  renderMap(update);
};
window.addEventListener("resize", () => {
  if (lastUpdate) renderMap(lastUpdate);
});
