/*
 * ws_client.js — feed live map updates to the viewer (App Lab brain, Branch B).
 *
 * The brain (python/main.py) renders nothing server-side: it serves MapUpdate JSON at
 * /mapupdate and this client polls it and hands each frame to the viewer's window.__onMapUpdate
 * hook (the real client-side renderer in viewer.js). The "Return home" button POSTs /return.
 *
 * (This replaced the original websocket transport: the App Lab brain has no websocket server
 * — polling keeps the map rendering on the browser and the Dragonwing free of rasterization.)
 *
 * Message contract: ../../docs/message-schemas.md (MapUpdate JSON each frame).
 */

const POLL_MS = 150; // ~7 fps; the map changes slowly, this is plenty and easy on the brain.

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

let live = false;

async function poll() {
  try {
    const res = await fetch("/mapupdate?t=" + Date.now(), { cache: "no-store" });
    if (res.status === 200) {
      dispatch(await res.json());
      if (!live) { live = true; setStatus("LIVE", "live"); }
    } else {
      // 503 = brain up but no map yet (phone not streaming / no frames folded).
      live = false;
      setStatus("waiting for map...", "down");
    }
  } catch (_) {
    live = false;
    setStatus("brain unreachable", "down");
  }
}

setStatus("connecting...", "");
setInterval(poll, POLL_MS);
poll();

// "Return home" button -> POST /return (the brain's Navigator.request_return).
export function requestReturn() {
  fetch("/return", { method: "POST" }).catch(() => {});
}

const returnBtn = document.getElementById("return");
if (returnBtn) {
  returnBtn.addEventListener("click", requestReturn);
}
