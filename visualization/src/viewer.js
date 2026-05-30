/*
 * viewer.js - render the live map.
 *
 * Stub. Draws a MapUpdate (occupancy grid + robot pose + planned return path).
 * Start with a 2D canvas grid; swap to a Three.js point cloud later for the 3D bonus.
 *
 * Message contract: ../../docs/message-schemas.md (MapUpdate). Field names match the
 * Python and C++ sides exactly: width, height, resolution_m, origin, cells, pose,
 * return_path.
 */

const canvas = document.getElementById("map");
const hud = document.getElementById("hud");

/*
 * Render one MapUpdate frame.
 *
 * Input: update - a MapUpdate object:
 *   { width, height, resolution_m, origin: {x, y}, cells: int[],
 *     pose: { x, y, theta, ... }, return_path: [{x, y}, ...] }
 * Output: none. Draws to the #map canvas.
 *
 * Cell encoding: -1 unknown, 0 free, 100 occupied.
 *
 * TODO: draw cells as a grid (gray unknown, white free, dark occupied), draw the robot
 *       pose as an oriented marker, and draw return_path as a polyline. Map world meters
 *       to pixels using resolution_m and origin.
 */
export function renderMap(update) {
  // TODO: implement the 2D canvas grid renderer.
  hud.textContent =
    `Recon Rover - ${update.width}x${update.height} cells @ ` +
    `${update.resolution_m} m, path: ${update.return_path.length} pts`;
}

// ws_client.js calls window.__onMapUpdate(update) for each frame.
window.__onMapUpdate = renderMap;

// TODO: size the canvas to the viewport and handle resize.
void canvas;
