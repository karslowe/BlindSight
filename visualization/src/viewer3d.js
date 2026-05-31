/*
 * viewer3d.js - 3D renderer for the live map (rung 2 of the visualization).
 *
 * ADDITIVE and ISOLATED: it consumes the exact same MapUpdate stream as the 2D viewer
 * (via the same window.__onMapUpdate hook fed by ws_client.js), and touches no backend, no
 * contract, and not the 2D viewer.js. The 2D view at "/" is unaffected.
 *
 * Renders the existing 2D occupancy grid as a 2.5D scene: a floor plane, occupied cells
 * extruded into wall blocks (instanced for speed), plus the rover, START, targets, and the
 * route home. No new data is needed - this is built from today's MapUpdate. (A true dense
 * point cloud is a later phase that needs the phone's depth.)
 *
 * Message contract: ../docs/message-schemas.md (MapUpdate). cells: row-major, row 0 at the
 * bottom, -1 unknown / 0 free / 100 occupied.
 *
 * World -> 3D mapping: world (x, y) -> THREE (x, height, -y), up = +Y. Heading theta is a
 * rotation about +Y.
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const canvas = document.getElementById("c3d");
const hud = document.getElementById("hud");

const WALL_HEIGHT = 0.3; // meters: how tall to extrude an occupied cell
const MAX_WALLS = 30000; // instanced-mesh capacity

// ---- scene / camera / renderer ----
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x11151c);
scene.fog = new THREE.Fog(0x11151c, 8, 22);

const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.05, 200);
camera.position.set(0, 4, 4);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
renderer.setSize(window.innerWidth, window.innerHeight);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.75));
const sun = new THREE.DirectionalLight(0xffffff, 0.8);
sun.position.set(4, 8, 3);
scene.add(sun);

// ---- persistent objects ----
let floor = null; // rebuilt only when the grid bounds change
let framedOnce = false;
let cloudFramed = false; // camera jumped to the live cloud once

const wallMesh = new THREE.InstancedMesh(
  new THREE.BoxGeometry(1, 1, 1),
  new THREE.MeshLambertMaterial({ color: 0xe74c3c }),
  MAX_WALLS
);
wallMesh.count = 0;
scene.add(wallMesh);

// Point cloud (Flavor B): the real depth-derived 3D points. Fixed-capacity buffers updated
// in place each frame; colored by height. Shown when MapUpdate.point_cloud is non-empty,
// in which case the extruded walls are hidden.
const POINT_CAP = 50000;
const ptPositions = new Float32Array(POINT_CAP * 3);
const ptColors = new Float32Array(POINT_CAP * 3);
const ptGeo = new THREE.BufferGeometry();
ptGeo.setAttribute("position", new THREE.BufferAttribute(ptPositions, 3));
ptGeo.setAttribute("color", new THREE.BufferAttribute(ptColors, 3));
const points = new THREE.Points(ptGeo, new THREE.PointsMaterial({ size: 0.045, vertexColors: true }));
points.visible = false;
scene.add(points);
const _col = new THREE.Color();

// Rover: a cone re-oriented to point along +X, then rotated about Y by the heading.
const roverGeo = new THREE.ConeGeometry(0.08, 0.24, 18);
roverGeo.rotateZ(-Math.PI / 2); // tip now points +X
const rover = new THREE.Mesh(roverGeo, new THREE.MeshLambertMaterial({ color: 0xf1c40f }));
rover.visible = false;
scene.add(rover);

const startMarker = new THREE.Mesh(
  new THREE.CylinderGeometry(0.08, 0.08, 0.05, 22),
  new THREE.MeshLambertMaterial({ color: 0x1abc9c })
);
startMarker.visible = false;
scene.add(startMarker);

let routeLine = null;
const targetsGroup = new THREE.Group();
scene.add(targetsGroup);

const _m = new THREE.Matrix4();
const _q = new THREE.Quaternion();
const _p = new THREE.Vector3();
const _s = new THREE.Vector3();

let gridKey = "";

function makeLabel(text, x, y, z) {
  const cnv = document.createElement("canvas");
  cnv.width = 256;
  cnv.height = 64;
  const ctx = cnv.getContext("2d");
  ctx.fillStyle = "#ffffff";
  ctx.font = "bold 40px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, 128, 34);
  const tex = new THREE.CanvasTexture(cnv);
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }));
  spr.position.set(x, y, z);
  spr.scale.set(0.55, 0.14, 1);
  return spr;
}

function clearGroup(g) {
  for (let i = g.children.length - 1; i >= 0; i--) {
    const ch = g.children[i];
    g.remove(ch);
    if (ch.geometry) ch.geometry.dispose();
    if (ch.material) {
      if (ch.material.map) ch.material.map.dispose();
      ch.material.dispose();
    }
  }
}

/*
 * Render one MapUpdate. ws_client.js (and the dev fake feed) call this per frame via the
 * window.__onMapUpdate hook below.
 */
function renderMap(update) {
  const { width, height, resolution_m, origin, cells } = update;
  if (!width || !height || !cells || !cells.length) return;

  // Floor: rebuild only when the grid bounds change (cheap, infrequent).
  const key = `${width}x${height}@${resolution_m}:${origin.x},${origin.y}`;
  if (key !== gridKey) {
    gridKey = key;
    const w = width * resolution_m;
    const h = height * resolution_m;
    const cx = origin.x + w / 2;
    const cy = origin.y + h / 2;
    if (floor) {
      scene.remove(floor);
      floor.geometry.dispose();
    }
    const geo = new THREE.PlaneGeometry(w, h);
    geo.rotateX(-Math.PI / 2); // lay flat in the XZ plane
    floor = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: 0xcfd8dc }));
    floor.position.set(cx, -0.01, -cy);
    scene.add(floor);

    controls.target.set(cx, 0, -cy);
    if (!framedOnce) {
      framedOnce = true;
      const d = Math.max(w, h);
      camera.position.set(cx, d * 0.9 + 1, -cy + d * 0.9 + 1);
    }
  }

  // 3D structure: the real point cloud (Flavor B) if present, else extruded walls (Flavor A).
  const pc = update.point_cloud || [];
  const numPts = Math.min((pc.length / 3) | 0, POINT_CAP);
  let cloudMode = false;
  if (numPts >= 2) {
    cloudMode = true;
    // Pass 1: bounds, for height-adaptive color and for framing the camera on the scan.
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity, zmin = Infinity, zmax = -Infinity;
    for (let i = 0; i < numPts; i++) {
      const x = pc[3 * i], y = pc[3 * i + 1], z = pc[3 * i + 2];
      if (x < xmin) xmin = x; if (x > xmax) xmax = x;
      if (y < ymin) ymin = y; if (y > ymax) ymax = y;
      if (z < zmin) zmin = z; if (z > zmax) zmax = z;
    }
    const zr = Math.max(0.3, zmax - zmin); // adaptive height span (avoids all-red saturation)
    // Pass 2: positions + color normalized over the cloud's actual height range.
    for (let i = 0; i < numPts; i++) {
      const mx = pc[3 * i], my = pc[3 * i + 1], mz = pc[3 * i + 2];
      ptPositions[3 * i] = mx;
      ptPositions[3 * i + 1] = mz; // height -> 3D up (y)
      ptPositions[3 * i + 2] = -my; // world y -> 3D -z
      const t = Math.max(0, Math.min(1, (mz - zmin) / zr));
      _col.setHSL(0.62 - 0.62 * t, 0.85, 0.55); // blue (low) -> red (high)
      ptColors[3 * i] = _col.r;
      ptColors[3 * i + 1] = _col.g;
      ptColors[3 * i + 2] = _col.b;
    }
    ptGeo.setDrawRange(0, numPts);
    ptGeo.attributes.position.needsUpdate = true;
    ptGeo.attributes.color.needsUpdate = true;
    ptGeo.computeBoundingSphere();
    points.visible = true;
    wallMesh.visible = false;

    // Frame the camera on the scan once (so walking away from START does not leave it
    // off-screen), then leave the controls alone so orbit/zoom/pan keep working.
    const cenX = (xmin + xmax) / 2, cenY = (ymin + ymax) / 2, cenZ = (zmin + zmax) / 2;
    const span = Math.max(xmax - xmin, ymax - ymin, 1.0);
    if (!cloudFramed && numPts > 200) {
      cloudFramed = true;
      controls.target.set(cenX, cenZ, -cenY);
      camera.position.set(cenX + span, cenZ + span * 1.2, -cenY + span);
    }
    // In a pure walk-around (no occupancy map yet) the START floor is meaningless, so slide
    // the reference floor under the scan at its ground level. The autonomy view keeps its
    // grid-aligned floor (it has mapped cells, so this branch is skipped there).
    const known = cells.filter((v) => v !== -1).length;
    if (known === 0 && floor) {
      floor.position.set(cenX, zmin - 0.02, -cenY);
      floor.scale.setScalar((span * 1.6 + 1.0) / (width * resolution_m));
    }
  } else {
    // Flavor A fallback: extrude each occupied cell into a wall block.
    let n = 0;
    for (let r = 0; r < height && n < MAX_WALLS; r++) {
      for (let c = 0; c < width && n < MAX_WALLS; c++) {
        if (cells[r * width + c] === 100) {
          const wx = origin.x + (c + 0.5) * resolution_m;
          const wy = origin.y + (r + 0.5) * resolution_m;
          _p.set(wx, WALL_HEIGHT / 2, -wy);
          _q.identity();
          _s.set(resolution_m, WALL_HEIGHT, resolution_m);
          _m.compose(_p, _q, _s);
          wallMesh.setMatrixAt(n, _m);
          n++;
        }
      }
    }
    wallMesh.count = n;
    wallMesh.instanceMatrix.needsUpdate = true;
    wallMesh.visible = true;
    points.visible = false;
  }

  // Rover.
  if (update.pose) {
    rover.position.set(update.pose.x, 0.13, -update.pose.y);
    rover.rotation.y = update.pose.theta;
    rover.visible = true;
  }

  // START marker (authoritative position from update.start).
  if (update.start) {
    startMarker.position.set(update.start.x, 0.04, -update.start.y);
    startMarker.visible = true;
  }

  // Route home (only present while returning).
  if (routeLine) {
    scene.remove(routeLine);
    routeLine.geometry.dispose();
    routeLine.material.dispose();
    routeLine = null;
  }
  const rp = update.return_path || [];
  if (rp.length >= 2) {
    const pts = rp.map((wp) => new THREE.Vector3(wp.x, 0.07, -wp.y));
    routeLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0x2ecc71 })
    );
    scene.add(routeLine);
  }

  // Targets (YOLO detections).
  clearGroup(targetsGroup);
  for (const t of update.targets || []) {
    const marker = new THREE.Mesh(
      new THREE.OctahedronGeometry(0.1),
      new THREE.MeshLambertMaterial({ color: 0xe84393 })
    );
    marker.position.set(t.x, 0.16, -t.y);
    targetsGroup.add(marker);
    targetsGroup.add(makeLabel(String(t.label).toUpperCase(), t.x, 0.34, -t.y));
  }

  // HUD.
  const known = cells.filter((v) => v !== -1).length;
  const pct = ((known / cells.length) * 100).toFixed(0);
  const returning = rp.length > 0;
  const found = (update.targets || []).length > 0;
  hud.textContent =
    `3D (${cloudMode ? "point cloud" : "extruded"}) - mapped ${pct}% - ` +
    `${returning ? "RETURNING to start" : "exploring"}` +
    (found ? ` - TARGET FOUND` : "");
}

// The single entry point ws_client.js / the dev fake feed call for every frame.
window.__onMapUpdate = renderMap;

// ---- render loop + resize ----
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
