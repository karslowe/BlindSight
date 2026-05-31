/*
 * viewer_mesh.js - renders the "3D scan": a solid triangle MESH instead of a point cloud.
 *
 * ADDITIVE and ISOLATED, exactly like viewer3d.js: it consumes the same MapUpdate stream via
 * window.__onMapUpdate (fed by ws_client.js, reused unchanged) and touches no backend, no
 * contract change beyond reading the optional MapUpdate.mesh field, and not the other viewers.
 *
 * It reads MapUpdate.mesh = { vertices: [x,y,z,...], faces: [i,j,k,...] } (map-frame, flat),
 * builds a THREE.Mesh with computed normals and height-based vertex colors, and draws it over
 * the floor with the rover + START markers. When no mesh is present it just shows the floor.
 *
 * World -> 3D mapping matches the other viewers: world (x, y, z) -> THREE (x, z, -y), up = +Y.
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const canvas = document.getElementById("c3d");
const hud = document.getElementById("hud");

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x11151c);
scene.fog = new THREE.Fog(0x11151c, 10, 26);

const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.05, 200);
camera.position.set(0, 4, 4);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
renderer.setSize(window.innerWidth, window.innerHeight);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const sun = new THREE.DirectionalLight(0xffffff, 0.9);
sun.position.set(4, 8, 3);
scene.add(sun);
const sun2 = new THREE.DirectionalLight(0xffffff, 0.4);
sun2.position.set(-5, 6, -4);
scene.add(sun2);

let floor = null;
let framedOnce = false;
let meshFramed = false; // camera jumped to the live mesh once
let gridKey = "";

// The reconstructed scan mesh, rebuilt in place whenever a new mesh arrives.
const scanMesh = new THREE.Mesh(
  new THREE.BufferGeometry(),
  new THREE.MeshStandardMaterial({
    vertexColors: true, roughness: 0.95, metalness: 0.0, side: THREE.DoubleSide, flatShading: false,
  })
);
scanMesh.visible = false;
scene.add(scanMesh);

const roverGeo = new THREE.ConeGeometry(0.08, 0.24, 18);
roverGeo.rotateZ(-Math.PI / 2);
const rover = new THREE.Mesh(roverGeo, new THREE.MeshLambertMaterial({ color: 0xf1c40f }));
rover.visible = false;
scene.add(rover);

const startMarker = new THREE.Mesh(
  new THREE.CylinderGeometry(0.08, 0.08, 0.05, 22),
  new THREE.MeshLambertMaterial({ color: 0x1abc9c })
);
startMarker.visible = false;
scene.add(startMarker);

const _col = new THREE.Color();

// Build/replace the scan mesh geometry from the flat vertices + faces in MapUpdate.mesh.
function updateScanMesh(mesh) {
  const v = mesh.vertices || [];
  const f = mesh.faces || [];
  const nVerts = (v.length / 3) | 0;
  if (nVerts < 3 || f.length < 3) {
    scanMesh.visible = false;
    return null;
  }
  // Pass 1: bounds, for height-adaptive color and framing.
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity, zmin = Infinity, zmax = -Infinity;
  for (let i = 0; i < nVerts; i++) {
    const x = v[3 * i], y = v[3 * i + 1], z = v[3 * i + 2];
    if (x < xmin) xmin = x; if (x > xmax) xmax = x;
    if (y < ymin) ymin = y; if (y > ymax) ymax = y;
    if (z < zmin) zmin = z; if (z > zmax) zmax = z;
  }
  const zr = Math.max(0.3, zmax - zmin);
  // Pass 2: positions + color over the mesh's actual height range.
  const pos = new Float32Array(nVerts * 3);
  const col = new Float32Array(nVerts * 3);
  for (let i = 0; i < nVerts; i++) {
    const mx = v[3 * i], my = v[3 * i + 1], mz = v[3 * i + 2]; // map-frame x, y, height
    pos[3 * i] = mx;
    pos[3 * i + 1] = mz;      // height -> 3D up (y)
    pos[3 * i + 2] = -my;     // world y -> 3D -z
    const t = Math.max(0, Math.min(1, (mz - zmin) / zr)); // blue low -> red high
    _col.setHSL(0.62 - 0.62 * t, 0.8, 0.55);
    col[3 * i] = _col.r;
    col[3 * i + 1] = _col.g;
    col[3 * i + 2] = _col.b;
  }
  const geo = scanMesh.geometry;
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
  geo.setIndex(new THREE.BufferAttribute(Uint32Array.from(f), 1));
  geo.computeVertexNormals();
  geo.computeBoundingSphere();
  scanMesh.visible = true;
  return {
    nTris: (f.length / 3) | 0,
    cenX: (xmin + xmax) / 2, cenY: (ymin + ymax) / 2, cenZ: (zmin + zmax) / 2,
    span: Math.max(xmax - xmin, ymax - ymin, 1.0), zmin,
  };
}

function renderMap(update) {
  const { width, height, resolution_m, origin, cells } = update;
  if (!width || !height || !cells || !cells.length) return;

  const key = `${width}x${height}@${resolution_m}:${origin.x},${origin.y}`;
  if (key !== gridKey) {
    gridKey = key;
    const w = width * resolution_m;
    const h = height * resolution_m;
    const cx = origin.x + w / 2;
    const cy = origin.y + h / 2;
    if (floor) { scene.remove(floor); floor.geometry.dispose(); }
    const geo = new THREE.PlaneGeometry(w, h);
    geo.rotateX(-Math.PI / 2);
    floor = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0xcfd8dc, roughness: 1 }));
    floor.position.set(cx, -0.01, -cy);
    scene.add(floor);
    controls.target.set(cx, 0, -cy);
    if (!framedOnce) {
      framedOnce = true;
      const d = Math.max(w, h);
      camera.position.set(cx, d * 0.9 + 1, -cy + d * 0.9 + 1);
    }
  }

  let nTris = 0;
  const info = update.mesh ? updateScanMesh(update.mesh) : null;
  if (info) {
    nTris = info.nTris;
    // Frame the camera on the scan once, then leave the controls for the user.
    if (!meshFramed && nTris > 100) {
      meshFramed = true;
      controls.target.set(info.cenX, info.cenZ, -info.cenY);
      camera.position.set(info.cenX + info.span, info.cenZ + info.span * 1.2, -info.cenY + info.span);
    }
    // Slide the reference floor under the scan at its ground level (no occupancy map here).
    if (floor) {
      floor.position.set(info.cenX, info.zmin - 0.02, -info.cenY);
      floor.scale.setScalar((info.span * 1.6 + 1.0) / (width * resolution_m));
    }
  } else {
    scanMesh.visible = false;
  }

  if (update.pose) {
    rover.position.set(update.pose.x, 0.13, -update.pose.y);
    rover.rotation.y = update.pose.theta;
    rover.visible = true;
  }
  if (update.start) {
    startMarker.position.set(update.start.x, 0.04, -update.start.y);
    startMarker.visible = true;
  }

  hud.textContent = `3D scan (mesh) - ${nTris.toLocaleString()} triangles`;
}

window.__onMapUpdate = renderMap;

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
