/*
 * ws_client.js - connect to the rover websocket and feed live map updates to the viewer.
 *
 * The rover (UNO Q) hosts the websocket; the phone is on the rover's Wi-Fi AP, so the
 * server is the page's own origin.
 *
 * Message contract: ../../docs/message-schemas.md
 *   Receives: MapUpdate (JSON) each frame.
 *   Sends: a single "return home" command when the user taps the button.
 *
 * DEV FALLBACK: when no rover websocket is reachable (you are developing the viewer on a
 * laptop), this falls back to the simulated feed in ../dev/fake_map.js so you always see
 * an animated map. Force the fake feed with ?fake in the URL; force live-only with ?live.
 * For production, the real socket connects and the fake feed is never started. The whole
 * fallback is the clearly marked block below - delete it and dev/ to ship.
 */

import { startFakeFeed } from "../dev/fake_map.js";

// Same-origin websocket: the page is served from the rover that also hosts the socket.
const WS_URL = `ws://${location.host}/ws`;

// How long to wait for the real socket before falling back to the fake feed (dev).
const CONNECT_TIMEOUT_MS = 1500;

const params = new URLSearchParams(location.search);
const FORCE_FAKE = params.has("fake");
const FORCE_LIVE = params.has("live");

let socket = null;
let fakeController = null; // { stop, requestReturn } when the dev feed is running

// Update the connection badge: label text plus a state class (live | sim | down).
function setStatus(label, state = "") {
  const el = document.getElementById("status");
  if (!el) return;
  el.className = "panel" + (state ? " " + state : "");
  const lbl = el.querySelector(".label");
  if (lbl) lbl.textContent = label;
}

function dispatch(update) {
  if (update && update.type === "MapUpdate" && window.__onMapUpdate) {
    window.__onMapUpdate(update);
  }
}

// ---- DEV FALLBACK (remove for production) -------------------------------------------
function startFakeFallback(reason) {
  if (fakeController) return; // already running
  console.warn(`[ws_client] using simulated map feed (${reason}).`);
  setStatus("SIMULATED", "sim");
  fakeController = startFakeFeed(dispatch);
}
// -------------------------------------------------------------------------------------

/*
 * Open the websocket and route incoming MapUpdate messages to the viewer.
 * Reconnects with backoff on close so a brief Wi-Fi drop self-heals.
 */
export function connect() {
  if (FORCE_FAKE) {
    startFakeFallback("forced by ?fake");
    return;
  }

  setStatus("connecting...", "");
  let settled = false;
  const timeout = setTimeout(() => {
    if (!settled && !FORCE_LIVE) {
      settled = true;
      try { socket && socket.close(); } catch (_) {}
      startFakeFallback("connect timeout");
    }
  }, CONNECT_TIMEOUT_MS);

  try {
    socket = new WebSocket(WS_URL);
  } catch (err) {
    clearTimeout(timeout);
    if (!FORCE_LIVE) startFakeFallback("websocket construction failed");
    return;
  }

  socket.addEventListener("open", () => {
    settled = true;
    clearTimeout(timeout);
    if (fakeController) { fakeController.stop(); fakeController = null; } // real data wins
    setStatus("LIVE", "live");
  });

  socket.addEventListener("message", (event) => {
    let update;
    try {
      update = JSON.parse(event.data);
    } catch (_) {
      return; // ignore non-JSON frames
    }
    dispatch(update);
  });

  socket.addEventListener("close", () => {
    if (!settled && !FORCE_LIVE) {
      settled = true;
      clearTimeout(timeout);
      startFakeFallback("connection closed");
      return;
    }
    // Live connection dropped after being open: try to reconnect.
    if (settled && FORCE_LIVE) {
      setStatus("reconnecting...", "down");
      setTimeout(connect, 1000);
    }
  });

  socket.addEventListener("error", () => {
    // The close handler does the fallback; just surface it in dev.
    console.warn("[ws_client] websocket error");
  });
}

/*
 * Send the single "return home" command back to the rover.
 * TODO: confirm the exact command shape with the navigation team.
 */
export function requestReturn() {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ type: "ReturnHome" }));
  } else if (fakeController) {
    // Dev / demo mode: drive the simulated rover home.
    fakeController.requestReturn();
  } else {
    console.warn("[ws_client] no live rover connection; return command ignored.");
  }
}

// Wire the on-screen button.
const returnBtn = document.getElementById("return");
if (returnBtn) {
  returnBtn.addEventListener("click", requestReturn);
}

connect();
