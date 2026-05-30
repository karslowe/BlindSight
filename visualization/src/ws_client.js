/*
 * ws_client.js - connect to the rover websocket and feed live map updates to the viewer.
 *
 * Stub. The rover (UNO Q) hosts the websocket; the phone is on the rover's Wi-Fi AP, so
 * the server is the page's own origin.
 *
 * Message contract: ../../docs/message-schemas.md
 *   Receives: MapUpdate (JSON) each frame.
 *   Sends: a single "return home" command when the user taps the button.
 */

// Same-origin websocket: the page is served from the rover that also hosts the socket.
const WS_URL = `ws://${location.host}/ws`;

let socket = null;

/*
 * Open the websocket and route incoming MapUpdate messages to the viewer.
 * Input: none. Output: none.
 * TODO: reconnect with backoff on close/error so a brief Wi-Fi drop self-heals.
 */
export function connect() {
  socket = new WebSocket(WS_URL);

  socket.addEventListener("message", (event) => {
    // TODO: validate the message type before dispatching.
    const update = JSON.parse(event.data);
    if (update.type === "MapUpdate" && window.__onMapUpdate) {
      window.__onMapUpdate(update);
    }
  });

  // TODO: handle "open", "close", "error" (status in the HUD, reconnect on close).
}

/*
 * Send the single "return home" command back to the rover.
 * Input: none. Output: none.
 * TODO: define the exact command shape with the navigation team (a small JSON object).
 */
export function requestReturn() {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ type: "ReturnHome" }));
  }
}

// Wire the on-screen button.
const returnBtn = document.getElementById("return");
if (returnBtn) {
  returnBtn.addEventListener("click", requestReturn);
}

connect();
