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

// BLINDSIGHT palette: dark base, violet structure, neon-purple routes.
const COLOR = {
  unknown: "#14121c",
  free: "#2f2b45",
  occupied: "#7c5cf0", // walls render as violet structure
  trail: "rgba(199, 186, 236, 0.5)",
  path: "#b388ff", // neon route home (glows)
  pose: "#d6c8ff",
  poseStroke: "#3a2f66",
  home: "#2dd4bf", // mint anchor for "home"
  homeText: "#ecf0f1",
  target: "#ec4899",
  targetText: "#ffffff",
  // 3D rover
  roverTop: "#d6c8ff",
  roverSide: "#7c5cf0",
  roverLight: "#f0e6ff",
  wheel: "#15121d",
};

// ---- Camera: a user zoom/pan applied on top of the auto-fit ----
const cam = { zoom: 1, panX: 0, panY: 0 };
const ZOOM_MIN = 0.4;
const ZOOM_MAX = 10;
let currentView = null; // most recent baked view, for input inversion
let lastUpdate = null; // most recent frame, for redraw on resize / interaction
let mode = "iso"; // "iso" = 2.5D extruded view (default), "flat" = top-down. Toggle button.

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
  ctx.lineWidth = 2.5;
  ctx.setLineDash([2, 6]); // dotted
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

// ====================================================================================
// 2.5D ISOMETRIC RENDERING ("3D appearance"). The SAME MapUpdate, projected into an
// isometric view: free cells are flat floor tiles, occupied cells are extruded into
// boxes (walls), and the overlays (trail, route home, rover, start, targets) are
// projected onto the floor plane. Pure 2D canvas - no Three.js. Toggle with the view
// button. Unknown cells are skipped, so the explored region floats on the dark bg.
// ====================================================================================
const WALL_CELLS = 2.1; // obstacle extrusion height, in cell-widths (visual only)

function darken(hex, f) {
  const n = parseInt(hex.slice(1), 16);
  return `rgb(${Math.round(((n >> 16) & 255) * f)},${Math.round(((n >> 8) & 255) * f)},${Math.round((n & 255) * f)})`;
}

// Build the isometric view as a ZOOM-INDEPENDENT fit; the camera (pan/zoom) is applied
// uniformly in isoProject(), so the existing zoomAt()/pan logic keeps working unchanged.
function computeIsoView(update, cssW, cssH) {
  const W = update.width, H = update.height, margin = 40;
  const footW = W + H;                       // iso footprint width  (uBase units)
  const footH = (W + H) * 0.5 + WALL_CELLS;  // iso footprint height (uBase units)
  const uBase = Math.min((cssW - 2 * margin) / footW, (cssH - 2 * margin) / footH);
  return {
    iso: true, W, H, uBase,
    centerX: cssW / 2, centerY: cssH / 2,
    cxBase: ((W - H) / 2) * uBase,           // iso scene center, x
    cyBase: ((W + H - 2) * 0.25) * uBase,    // iso scene center, y (floor plane)
    resolution_m: update.resolution_m, origin: update.origin,
  };
}

// Grid (col, row, height h in cell-widths) -> screen. Built in zoom-1 fit space, then
// pan + zoom applied uniformly (the camera model zoomAt() assumes: screen = pan + zoom*fit).
function isoProject(col, row, h, v) {
  const fx = v.centerX + ((col - row) * v.uBase - v.cxBase);
  const fy = v.centerY + ((col + row) * v.uBase * 0.5 - v.cyBase) - h * v.uBase;
  return { x: cam.panX + cam.zoom * fx, y: cam.panY + cam.zoom * fy };
}

function worldToIso(wx, wy, v, h = 0) {
  return isoProject((wx - v.origin.x) / v.resolution_m, (wy - v.origin.y) / v.resolution_m, h, v);
}

function fillPoly(pts, fill) {
  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.lineWidth = 1;
  ctx.strokeStyle = fill; // seal hairline seams between adjacent tiles
  ctx.stroke();
}

function strokeIsoPath(points, color, width, dash) {
  if (points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = "round";
  ctx.setLineDash(dash || []);
  ctx.beginPath();
  points.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawIsoCells(update, v) {
  const { W, H } = v;
  // Painter's order: back-to-front by ascending (col+row), so nearer boxes overlap farther.
  for (let s = 0; s <= W + H - 2; s++) {
    const cLo = Math.max(0, s - (H - 1));
    const cHi = Math.min(W - 1, s);
    for (let c = cLo; c <= cHi; c++) {
      const r = s - c;
      const val = update.cells[r * W + c];
      if (val === -1) continue; // unknown -> skipped (floats on the dark background)
      if (val >= 100) {
        const At = isoProject(c, r, WALL_CELLS, v), Bt = isoProject(c + 1, r, WALL_CELLS, v);
        const Ct = isoProject(c + 1, r + 1, WALL_CELLS, v), Dt = isoProject(c, r + 1, WALL_CELLS, v);
        const Bb = isoProject(c + 1, r, 0, v), Cb = isoProject(c + 1, r + 1, 0, v), Db = isoProject(c, r + 1, 0, v);
        fillPoly([Bt, Ct, Cb, Bb], darken(COLOR.occupied, 0.7)); // right side face
        fillPoly([Ct, Dt, Db, Cb], darken(COLOR.occupied, 0.5)); // left side face
        fillPoly([At, Bt, Ct, Dt], COLOR.occupied);              // top face
      } else {
        const A = isoProject(c, r, 0, v), B = isoProject(c + 1, r, 0, v);
        const C = isoProject(c + 1, r + 1, 0, v), D = isoProject(c, r + 1, 0, v);
        fillPoly([A, B, C, D], COLOR.free);
      }
    }
  }
}

// A little 3D rover: oriented chassis box + wheels + a glowing headlight, projected from
// world space so it rotates with pose.theta. Footprint is in meters (real size on the map);
// heights are visual cell-widths, sized to sit shorter than the walls.
function drawRover3D(pose, v) {
  const HL = 0.12, HW = 0.08;   // meters: footprint half-length / half-width
  const Z0 = 0.2, Z1 = 1.0;     // chassis bottom / top (cell-widths, visual)
  const WHZ = 0.34;             // wheel height (cell-widths)
  const fx = Math.cos(pose.theta), fy = Math.sin(pose.theta);
  const lx = -Math.sin(pose.theta), ly = Math.cos(pose.theta);
  // body-frame (s forward in [-1,1], t left in [-1,1]) at visual height z -> screen
  const p3 = (s, t, z) =>
    worldToIso(pose.x + s * HL * fx + t * HW * lx, pose.y + s * HL * fy + t * HW * ly, v, z);
  const scale = v.uBase * cam.zoom;

  // grounding shadow
  const c0 = p3(0, 0, 0);
  ctx.save();
  ctx.fillStyle = "rgba(0, 0, 0, 0.4)";
  ctx.beginPath();
  ctx.ellipse(c0.x, c0.y, scale * 1.7, scale * 1.0, 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  // wheels (dark treads just outside the chassis sides)
  for (const [s, t] of [[0.78, 1.15], [0.78, -1.15], [-0.78, 1.15], [-0.78, -1.15]]) {
    fillPoly([p3(s - 0.2, t, WHZ), p3(s + 0.2, t, WHZ), p3(s + 0.2, t, 0), p3(s - 0.2, t, 0)], COLOR.wheel);
  }

  // chassis side faces, drawn far -> near, then the bright top
  const rect = [[1, -1], [1, 1], [-1, 1], [-1, -1]];
  const faces = [];
  for (let i = 0; i < 4; i++) {
    const [s1, t1] = rect[i];
    const [s2, t2] = rect[(i + 1) % 4];
    const a = p3(s1, t1, Z0), b = p3(s2, t2, Z0);
    faces.push({
      pts: [p3(s1, t1, Z1), p3(s2, t2, Z1), b, a],
      y: (a.y + b.y) / 2,
      front: s1 === 1 && s2 === 1,
    });
  }
  faces.sort((p, q) => p.y - q.y);
  for (const f of faces) fillPoly(f.pts, darken(COLOR.roverSide, f.front ? 0.85 : 0.6));
  fillPoly([p3(1, -1, Z1), p3(1, 1, Z1), p3(-1, 1, Z1), p3(-1, -1, Z1)], COLOR.roverTop);

  // glowing headlight at the nose
  const hl = p3(1, 0, Z1);
  ctx.save();
  ctx.shadowColor = COLOR.roverLight;
  ctx.shadowBlur = 10;
  ctx.fillStyle = COLOR.roverLight;
  ctx.beginPath();
  ctx.arc(hl.x, hl.y, Math.max(2, scale * 0.5), 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawIsoScene(update, v) {
  drawIsoCells(update, v);
  // Trail (dotted), then the neon route home with a glow.
  strokeIsoPath(trail.map((p) => worldToIso(p.x, p.y, v)), COLOR.trail, 2.5, [2, 6]);
  ctx.save();
  ctx.shadowColor = COLOR.path;
  ctx.shadowBlur = 12;
  strokeIsoPath((update.return_path || []).map((p) => worldToIso(p.x, p.y, v)), COLOR.path, 3);
  ctx.restore();
  // Start marker.
  if (home) {
    const p = worldToIso(home.x, home.y, v);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 8, 0, Math.PI * 2);
    ctx.fillStyle = COLOR.home;
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#fff";
    ctx.stroke();
    ctx.fillStyle = COLOR.homeText;
    ctx.font = "bold 12px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("START", p.x, p.y - 14);
  }
  // Targets (magenta crosshair).
  for (const t of update.targets || []) {
    const p = worldToIso(t.x, t.y, v), rr = 10;
    ctx.strokeStyle = COLOR.target;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(p.x, p.y, rr, 0, Math.PI * 2);
    ctx.moveTo(p.x - rr - 4, p.y);
    ctx.lineTo(p.x + rr + 4, p.y);
    ctx.moveTo(p.x, p.y - rr - 4);
    ctx.lineTo(p.x, p.y + rr + 4);
    ctx.stroke();
    ctx.fillStyle = COLOR.targetText;
    ctx.font = "bold 12px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(t.label.toUpperCase(), p.x, p.y - rr - 8);
  }
  // Rover: a 3D chassis projected from world space (rotates with heading).
  if (update.pose) drawRover3D(update.pose, v);
}

/*
 * Render one MapUpdate frame. ws_client (and the fake feed) call this per frame.
 * Input: update - a MapUpdate object. Output: none. Draws to #map and updates the HUD.
 */
export function renderMap(update) {
  // The brain tells us the true home position; use it (don't guess from the first pose,
  // which is wrong if the browser connected after the rover had already moved).
  if (update.start) home = { x: update.start.x, y: update.start.y };
  recordHistory(update.pose);

  const { w, h } = fitCanvas();
  ctx.clearRect(0, 0, w, h);

  if (mode === "iso") {
    const view = computeIsoView(update, w, h);
    currentView = view;
    drawIsoScene(update, view);
  } else {
    const view = computeView(update, w, h);
    currentView = view;
    drawCells(update, view);
    drawTrail(view);
    drawReturnPath(update, view);
    drawHome(view);
    drawTargets(update, view);
    drawPose(update, view);
  }

  const knownCells = update.cells.filter((v) => v !== -1).length;
  const pct = ((knownCells / update.cells.length) * 100).toFixed(0);
  const returning = (update.return_path || []).length > 0;
  const targets = update.targets || [];
  let status = returning ? "RETURNING to start" : "exploring";
  if (targets.length > 0) status += ` &middot; TARGET FOUND (${targets[0].label})`;
  hud.innerHTML = `mapped <b>${pct}%</b> &middot; ${status}`;
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

// View toggle: 2.5D isometric <-> flat top-down. Button shows the mode you'll switch TO.
const btnView = document.getElementById("btn-view");
if (btnView) {
  btnView.textContent = mode === "iso" ? "2D" : "3D";
  btnView.addEventListener("click", () => {
    mode = mode === "iso" ? "flat" : "iso";
    btnView.textContent = mode === "iso" ? "2D" : "3D";
    window.__resetView(); // reframe the reprojected scene (also redraws)
  });
}

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
