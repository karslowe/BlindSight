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

const wallMesh = new THREE.InstancedMesh(
  new THREE.BoxGeometry(1, 1, 1),
  new THREE.MeshLambertMaterial({ color: 0xe74c3c }),
  MAX_WALLS
);
wallMesh.count = 0;
scene.add(wallMesh);

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

  // Walls: one box instance per occupied cell.
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
    `3D - mapped ${pct}% - ${returning ? "RETURNING to start" : "exploring"}` +
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
