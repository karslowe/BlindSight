/*
 * fake_map.js - a DEV-ONLY fake MapUpdate source with a full mission story.
 *
 * Generates animated MapUpdate frames that match docs/message-schemas.md exactly, so you
 * can build, polish, and DEMO the viewer with zero dependency on the navigation team's
 * server or any hardware.
 *
 * The story it plays out:
 *   EXPLORE  - a simulated rover drives a loop while the occupancy grid fills in around
 *              it. return_path is empty (per the contract: empty until a return is asked).
 *   RETURN   - when requestReturn() is called (the "Return to start" button), the rover
 *              computes a route back along the path it actually drove and follows it home,
 *              with that route shown in return_path so the viewer draws it bright green.
 *
 * INTEGRATION: when the real rover server is up, this file is not loaded. ws_client only
 * falls back to it when no websocket is reachable (dev) or with ?fake in the URL. Delete
 * this folder for production; nothing else depends on it.
 */

const WIDTH = 64; // cells
const HEIGHT = 48; // cells
const RESOLUTION_M = 0.05; // 5 cm per cell -> 3.2 m x 2.4 m arena
const ORIGIN = { x: -1.6, y: -1.2 }; // world coord of cell (0,0), the lower-left corner

const UNKNOWN = -1;
const FREE = 0;
const OCCUPIED = 100;

const SENSOR_RADIUS_M = 0.55;
const SPEED_MPS = 0.25;

// The rover's exploration path: a rectangular loop through the free space. PATH[0] is home.
const PATH = [
  { x: -1.2, y: -0.8 },
  { x: 1.1, y: -0.8 },
  { x: 1.1, y: 0.8 },
  { x: -1.2, y: 0.8 },
];

function buildTruth() {
  const truth = new Int8Array(WIDTH * HEIGHT).fill(FREE);
  for (let r = 0; r < HEIGHT; r++) {
    for (let c = 0; c < WIDTH; c++) {
      const isBorder = c === 0 || c === WIDTH - 1 || r === 0 || r === HEIGHT - 1;
      const isObstacle = c >= 40 && c <= 48 && r >= 18 && r <= 30;
      if (isBorder || isObstacle) truth[r * WIDTH + c] = OCCUPIED;
    }
  }
  return truth;
}

function worldToCell(wx, wy) {
  return {
    c: Math.floor((wx - ORIGIN.x) / RESOLUTION_M),
    r: Math.floor((wy - ORIGIN.y) / RESOLUTION_M),
  };
}

function cellCenter(c, r) {
  return {
    x: ORIGIN.x + (c + 0.5) * RESOLUTION_M,
    y: ORIGIN.y + (r + 0.5) * RESOLUTION_M,
  };
}

function revealAround(known, truth, wx, wy) {
  const rad = SENSOR_RADIUS_M;
  const min = worldToCell(wx - rad, wy - rad);
  const max = worldToCell(wx + rad, wy + rad);
  for (let r = Math.max(0, min.r); r <= Math.min(HEIGHT - 1, max.r); r++) {
    for (let c = Math.max(0, min.c); c <= Math.min(WIDTH - 1, max.c); c++) {
      const ctr = cellCenter(c, r);
      const dx = ctr.x - wx;
      const dy = ctr.y - wy;
      if (dx * dx + dy * dy <= rad * rad) {
        const i = r * WIDTH + c;
        known[i] = truth[i];
      }
    }
  }
}

// Position and heading at `distance` traveled along a polyline of {x,y} points.
// loop=true wraps forever (exploration); loop=false clamps at the end (returning).
function pointAlongPolyline(points, distance, loop) {
  const segs = [];
  let total = 0;
  const n = loop ? points.length : points.length - 1;
  for (let i = 0; i < n; i++) {
    const a = points[i];
    const b = points[(i + 1) % points.length];
    const len = Math.hypot(b.x - a.x, b.y - a.y);
    segs.push({ a, b, len, start: total });
    total += len;
  }
  if (total === 0) {
    return { x: points[0].x, y: points[0].y, theta: 0, done: true };
  }
  let d = loop ? ((distance % total) + total) % total : Math.min(distance, total);
  const done = !loop && distance >= total;
  for (const s of segs) {
    if (d <= s.start + s.len || s === segs[segs.length - 1]) {
      const t = s.len > 0 ? (d - s.start) / s.len : 0;
      return {
        x: s.a.x + (s.b.x - s.a.x) * t,
        y: s.a.y + (s.b.y - s.a.y) * t,
        theta: Math.atan2(s.b.y - s.a.y, s.b.x - s.a.x),
        done,
      };
    }
  }
  const last = segs[segs.length - 1];
  return { x: last.b.x, y: last.b.y, theta: 0, done: true };
}

/*
 * Start emitting fake MapUpdate frames.
 *
 * Input:
 *   onUpdate: callback invoked with one MapUpdate object per tick (schema-shaped).
 *   intervalMs: tick period (default 150 ms).
 * Output:
 *   a controller { stop(), requestReturn() }:
 *     stop()          - halt the feed.
 *     requestReturn() - switch the rover into RETURN mode (the button calls this).
 */
export function startFakeFeed(onUpdate, intervalMs = 150) {
  if (window.__resetTrail) window.__resetTrail(); // fresh history for this run

  const truth = buildTruth();
  const known = new Int8Array(WIDTH * HEIGHT).fill(UNKNOWN);

  let mode = "explore"; // "explore" | "return"
  let exploreDist = 0;
  let returnDist = 0;
  let returnRoute = null; // [{x,y}, ...] from current position back to home
  const drivenTrail = []; // the rover's own record of where it actually drove
  const TRAIL_SPACING_M = 0.06;

  function recordDriven(x, y) {
    const last = drivenTrail[drivenTrail.length - 1];
    if (!last || Math.hypot(x - last.x, y - last.y) >= TRAIL_SPACING_M) {
      drivenTrail.push({ x, y });
    }
  }

  const dt = intervalMs / 1000;

  const tick = () => {
    let pose;
    let returnPath = [];

    if (mode === "explore") {
      exploreDist += SPEED_MPS * dt;
      pose = pointAlongPolyline(PATH, exploreDist, true);
      recordDriven(pose.x, pose.y);
    } else {
      returnDist += SPEED_MPS * dt;
      pose = pointAlongPolyline(returnRoute, returnDist, false);
      // Show the full planned route home so the viewer draws it bright green.
      returnPath = returnRoute.map((wp) => ({ x: wp.x, y: wp.y }));
    }

    revealAround(known, truth, pose.x, pose.y);

    onUpdate({
      type: "MapUpdate",
      width: WIDTH,
      height: HEIGHT,
      resolution_m: RESOLUTION_M,
      origin: { x: ORIGIN.x, y: ORIGIN.y },
      cells: Array.from(known),
      pose: {
        type: "Pose",
        x: pose.x,
        y: pose.y,
        theta: pose.theta,
        covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0],
        timestamp: performance.now() / 1000,
      },
      return_path: returnPath,
    });
  };

  tick();
  const handle = setInterval(tick, intervalMs);

  return {
    stop() {
      clearInterval(handle);
    },
    requestReturn() {
      if (mode === "return") return;
      // Plan the route home: retrace the driven trail in reverse back to the start.
      const reversed = drivenTrail.slice().reverse();
      if (reversed.length >= 2) {
        returnRoute = reversed;
      } else {
        // Pressed almost immediately: just head straight to home.
        returnRoute = [{ x: drivenTrail[0]?.x ?? PATH[0].x, y: drivenTrail[0]?.y ?? PATH[0].y }, { ...PATH[0] }];
      }
      returnDist = 0;
      mode = "return";
    },
  };
}
