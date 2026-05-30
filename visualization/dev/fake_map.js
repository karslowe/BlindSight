/*
 * fake_map.js - a DEV-ONLY fake MapUpdate source.
 *
 * Generates animated MapUpdate frames that match docs/message-schemas.md exactly, so you
 * can build and polish the viewer with zero dependency on the navigation team's server or
 * any hardware. A simulated rover drives a loop while the occupancy grid fills in around
 * it, exactly like the real mapping pipeline will feed you.
 *
 * INTEGRATION: when the real rover server is up, this file is not loaded at all. ws_client
 * only falls back to it when no websocket is reachable (dev), or when the page URL has
 * ?fake. Delete this folder for production; nothing else depends on it.
 *
 * The frames it emits are indistinguishable, to viewer.js, from frames off the real rover.
 */

// ---- Simulated world geometry (everything in SI: meters, radians) ----
const WIDTH = 64; // cells
const HEIGHT = 48; // cells
const RESOLUTION_M = 0.05; // 5 cm per cell -> 3.2 m x 2.4 m arena
const ORIGIN = { x: -1.6, y: -1.2 }; // world coord of cell (0,0), the lower-left corner

// Cell encoding, matching the schema: -1 unknown, 0 free, 100 occupied.
const UNKNOWN = -1;
const FREE = 0;
const OCCUPIED = 100;

// How far the rover "sees" and reveals cells each step.
const SENSOR_RADIUS_M = 0.55;

// ---- Build the ground-truth map the rover will gradually discover ----
// Outer walls plus one interior obstacle block. The rover never knows this directly;
// it only reveals cells within SENSOR_RADIUS_M as it drives.
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

// ---- The rover's path: a rectangular loop through the free space ----
const PATH = [
  { x: -1.2, y: -0.8 },
  { x: 1.1, y: -0.8 },
  { x: 1.1, y: 0.8 },
  { x: -1.2, y: 0.8 },
];

// World point -> integer cell (col, row). Row 0 is the bottom (origin is lower-left).
function worldToCell(wx, wy) {
  const c = Math.floor((wx - ORIGIN.x) / RESOLUTION_M);
  const r = Math.floor((wy - ORIGIN.y) / RESOLUTION_M);
  return { c, r };
}

// Center of cell (c, r) in world meters.
function cellCenter(c, r) {
  return {
    x: ORIGIN.x + (c + 0.5) * RESOLUTION_M,
    y: ORIGIN.y + (r + 0.5) * RESOLUTION_M,
  };
}

// Reveal every truth cell within the sensor radius of (wx, wy) into the known grid.
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

// Position and heading at a given distance traveled along the looping PATH.
function poseAlongPath(distance) {
  // Build cumulative segment lengths once.
  const segs = [];
  let total = 0;
  for (let i = 0; i < PATH.length; i++) {
    const a = PATH[i];
    const b = PATH[(i + 1) % PATH.length];
    const len = Math.hypot(b.x - a.x, b.y - a.y);
    segs.push({ a, b, len, start: total });
    total += len;
  }
  let d = distance % total; // wrap to loop forever
  for (const s of segs) {
    if (d <= s.start + s.len) {
      const t = (d - s.start) / s.len;
      const x = s.a.x + (s.b.x - s.a.x) * t;
      const y = s.a.y + (s.b.y - s.a.y) * t;
      const theta = Math.atan2(s.b.y - s.a.y, s.b.x - s.a.x);
      return { x, y, theta };
    }
  }
  const last = segs[segs.length - 1];
  return { x: last.b.x, y: last.b.y, theta: 0 };
}

/*
 * Start emitting fake MapUpdate frames.
 *
 * Input:
 *   onUpdate: callback invoked with one MapUpdate object per tick (schema-shaped).
 *   intervalMs: tick period (default 150 ms, ~6.7 Hz).
 * Output:
 *   a stop() function that halts the feed.
 */
export function startFakeFeed(onUpdate, intervalMs = 150) {
  const truth = buildTruth();
  const known = new Int8Array(WIDTH * HEIGHT).fill(UNKNOWN);
  let distance = 0;
  const speed = 0.25; // m/s, simulated drive speed

  const tick = () => {
    distance += speed * (intervalMs / 1000);
    const p = poseAlongPath(distance);
    revealAround(known, truth, p.x, p.y);

    // The "return path" shown to the viewer: the loop waypoints ahead of the rover.
    const returnPath = PATH.map((wp) => ({ x: wp.x, y: wp.y }));

    const update = {
      type: "MapUpdate",
      width: WIDTH,
      height: HEIGHT,
      resolution_m: RESOLUTION_M,
      origin: { x: ORIGIN.x, y: ORIGIN.y },
      cells: Array.from(known),
      pose: {
        type: "Pose",
        x: p.x,
        y: p.y,
        theta: p.theta,
        covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0],
        timestamp: performance.now() / 1000,
      },
      return_path: returnPath,
    };
    onUpdate(update);
  };

  tick();
  const handle = setInterval(tick, intervalMs);
  return function stop() {
    clearInterval(handle);
  };
}
