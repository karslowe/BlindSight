/*
 * viewer.js - render the live map onto the 2D canvas, with pan/zoom.
 *
 * Draws one MapUpdate per frame:
 *   - the occupancy grid (cells),
 *   - a faint breadcrumb TRAIL of everywhere the rover has driven (client-side history),
 *   - a START marker at the rover's first seen pose (where it returns to),
 *   - the planned RETURN PATH home (bright green, from return_path),
 *   - the rover pose (oriented marker).
 *
 * A camera (zoom + pan) sits on top of an auto-fit so the map fills the screen by default
 * but can be dragged and pinched/scrolled to inspect detail. Strokes and labels stay a
 * constant screen size at any zoom, so it reads crisply on a phone.
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
  trail: "rgba(236, 240, 241, 0.35)",
  path: "#2ecc71",
  pose: "#f1c40f",
  poseStroke: "#7a5c00",
  home: "#1abc9c",
  homeText: "#ecf0f1",
  target: "#e84393",
  targetText: "#ffffff",
};

// ---- Camera: a user zoom/pan applied on top of the auto-fit ----
const cam = { zoom: 1, panX: 0, panY: 0 };
const ZOOM_MIN = 0.4;
const ZOOM_MAX = 10;
let currentView = null; // most recent baked view, for input inversion
let lastUpdate = null; // most recent frame, for redraw on resize / interaction

// ---- Client-side history derived from the pose stream ----
let home = null;
const trail = [];
const TRAIL_MIN_SPACING_M = 0.08;

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

function fitCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || window.innerWidth;
  const cssH = canvas.clientHeight || window.innerHeight;
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: cssW, h: cssH };
}

// Auto-fit the grid centered in the canvas, then bake the camera (zoom/pan) into the
// scale and offsets. With zoom=1, pan=0 this is exactly the centered fit.
function computeView(update, cssW, cssH) {
  const margin = 16;
  const baseScale = Math.min(
    (cssW - 2 * margin) / update.width,
    (cssH - 2 * margin) / update.height
  );
  const baseOffsetX = (cssW - update.width * baseScale) / 2;
  const baseOffsetY = (cssH - update.height * baseScale) / 2;
  return {
    scale: baseScale * cam.zoom,
    offsetX: baseOffsetX * cam.zoom + cam.panX,
    offsetY: baseOffsetY * cam.zoom + cam.panY,
    width: update.width,
    height: update.height,
    resolution_m: update.resolution_m,
    origin: update.origin,
  };
}

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
      const sy = offsetY + (height - 1 - r) * scale;
      ctx.fillRect(sx, sy, scale + 0.5, scale + 0.5);
    }
  }
}

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
  const r = 9;
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
  const size = 11;
  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(-pose.theta);
  ctx.beginPath();
  ctx.moveTo(size, 0);
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
  currentView = view;
  drawCells(update, view);
  drawTrail(view);
  drawReturnPath(update, view);
  drawHome(view);
  drawTargets(update, view);
  drawPose(update, view);

  const knownCells = update.cells.filter((v) => v !== -1).length;
  const pct = ((knownCells / update.cells.length) * 100).toFixed(0);
  const returning = (update.return_path || []).length > 0;
  const targets = update.targets || [];
  let status = returning ? "RETURNING to start" : "exploring";
  if (targets.length > 0) status += ` - TARGET FOUND (${targets[0].label})`;
  hud.textContent = `mapped ${pct}% - ${status}`;
}

// Detected objects of interest (YOLO), drawn as a labeled magenta crosshair marker.
function drawTargets(update, view) {
  const targets = update.targets || [];
  for (const t of targets) {
    const p = worldToScreen(t.x, t.y, view);
    const r = 10;
    ctx.strokeStyle = COLOR.target;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.moveTo(p.x - r - 4, p.y);
    ctx.lineTo(p.x + r + 4, p.y);
    ctx.moveTo(p.x, p.y - r - 4);
    ctx.lineTo(p.x, p.y + r + 4);
    ctx.stroke();
    ctx.fillStyle = COLOR.targetText;
    ctx.font = "bold 12px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(t.label.toUpperCase(), p.x, p.y - r - 8);
  }
}

function redraw() {
  if (lastUpdate) renderMap(lastUpdate);
}

window.__onMapUpdate = (update) => {
  lastUpdate = update;
  renderMap(update);
};
window.addEventListener("resize", redraw);

// ---- Camera controls ----------------------------------------------------------------
// Zoom about a screen pivot so the point under the cursor / pinch stays put.
function zoomAt(pivotX, pivotY, factor) {
  const newZoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, cam.zoom * factor));
  const fitX = (pivotX - cam.panX) / cam.zoom; // invert camera to fit-space
  const fitY = (pivotY - cam.panY) / cam.zoom;
  cam.panX = pivotX - fitX * newZoom;
  cam.panY = pivotY - fitY * newZoom;
  cam.zoom = newZoom;
  redraw();
}

window.__resetView = () => {
  cam.zoom = 1;
  cam.panX = 0;
  cam.panY = 0;
  redraw();
};

function canvasPoint(e) {
  const rect = canvas.getBoundingClientRect();
  return { x: e.clientX - rect.left, y: e.clientY - rect.top };
}

// Pointer Events unify mouse + touch and give us multi-touch for pinch.
const pointers = new Map();
let gesture = null; // { x, y } for pan, or { dist, midX, midY } for pinch

function reinitGesture() {
  const pts = [...pointers.values()];
  if (pts.length === 1) {
    gesture = { x: pts[0].x, y: pts[0].y };
  } else if (pts.length >= 2) {
    const [a, b] = pts;
    gesture = {
      dist: Math.hypot(b.x - a.x, b.y - a.y),
      midX: (a.x + b.x) / 2,
      midY: (a.y + b.y) / 2,
    };
  } else {
    gesture = null;
  }
}

canvas.addEventListener("pointerdown", (e) => {
  canvas.setPointerCapture(e.pointerId);
  const p = canvasPoint(e);
  pointers.set(e.pointerId, p);
  reinitGesture();
});

canvas.addEventListener("pointermove", (e) => {
  if (!pointers.has(e.pointerId)) return;
  pointers.set(e.pointerId, canvasPoint(e));
  const pts = [...pointers.values()];

  if (pts.length === 1 && gesture) {
    // Single-pointer drag = pan.
    const p = pts[0];
    cam.panX += p.x - gesture.x;
    cam.panY += p.y - gesture.y;
    gesture = { x: p.x, y: p.y };
    redraw();
  } else if (pts.length >= 2 && gesture && gesture.dist != null) {
    // Two-pointer pinch = zoom about the midpoint, plus pan by the midpoint drag.
    const [a, b] = pts;
    const dist = Math.hypot(b.x - a.x, b.y - a.y);
    const midX = (a.x + b.x) / 2;
    const midY = (a.y + b.y) / 2;
    cam.panX += midX - gesture.midX;
    cam.panY += midY - gesture.midY;
    if (gesture.dist > 0) zoomAt(midX, midY, dist / gesture.dist);
    gesture = { dist, midX, midY };
  }
});

function endPointer(e) {
  if (pointers.has(e.pointerId)) pointers.delete(e.pointerId);
  reinitGesture();
}
canvas.addEventListener("pointerup", endPointer);
canvas.addEventListener("pointercancel", endPointer);
canvas.addEventListener("pointerleave", endPointer);

canvas.addEventListener(
  "wheel",
  (e) => {
    e.preventDefault();
    const p = canvasPoint(e);
    zoomAt(p.x, p.y, e.deltaY < 0 ? 1.1 : 1 / 1.1);
  },
  { passive: false }
);

// Double-tap / double-click to reset the view.
canvas.addEventListener("dblclick", () => window.__resetView());

// ---- Wire the on-screen control buttons ----
const btnReset = document.getElementById("btn-reset");
if (btnReset) btnReset.addEventListener("click", () => window.__resetView());

const btnFull = document.getElementById("btn-fullscreen");
if (btnFull) {
  btnFull.addEventListener("click", () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen?.();
    } else {
      document.exitFullscreen?.();
    }
  });
}
