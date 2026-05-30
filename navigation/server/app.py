"""Web server: serve the visualization and broadcast MapUpdate to the phone.

FastAPI app plus a websocket. Runs on the UNO Q, which also hosts the Wi-Fi access point
the phone joins. No cloud dependency.

Outbound: every MapUpdate the orchestrator produces is fanned out to all connected phones
over the websocket. Inbound: the single "return home" command from a phone is dispatched
to a registered callback (the orchestrator's request_return).

Threading model: the orchestrator runs a synchronous tick loop; the server runs an async
event loop in a background thread. The bridge is publish(), which is thread-safe and
schedules the fan-out onto the server's loop. The pure-logic parts (serialization, inbound
dispatch, fan-out) are written so they can be unit-tested without FastAPI installed.
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
from pathlib import Path
from typing import Callable, Optional, Set

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import MapUpdate  # noqa: E402

# Keep the module importable (and the core logic testable) before deps are installed.
try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.staticfiles import StaticFiles

    _FASTAPI = True
except ImportError:  # pragma: no cover - exercised only without deps
    _FASTAPI = False
    WebSocket = object  # type: ignore


# Path to the static visualization served from the rover.
VIZ_DIR = Path(__file__).resolve().parents[2] / "visualization"


class MapServer:
    """Holds connected websocket clients and broadcasts MapUpdate to them."""

    def __init__(self) -> None:
        self.clients: Set = set()
        # Set by the orchestrator: called when a phone sends the "return home" command.
        self.on_return_request: Optional[Callable[[], None]] = None
        # The most recent frame (serialized), so a newly connected phone is not blank.
        self._latest: Optional[str] = None
        # The server's event loop, captured on startup; needed by publish() (other thread).
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    # ---- Pure logic (no FastAPI required; unit-testable) ----

    @staticmethod
    def _serialize(update: MapUpdate) -> str:
        """Serialize a MapUpdate to the JSON string sent over the wire."""
        return json.dumps(update.to_dict())

    def _handle_inbound(self, raw: str) -> None:
        """Dispatch one inbound text message from a phone.

        Currently the only command is {"type": "ReturnHome"}, which triggers the
        registered on_return_request callback. Unknown or malformed messages are ignored.
        """
        try:
            data = json.loads(raw)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict) and data.get("type") == "ReturnHome":
            if self.on_return_request is not None:
                self.on_return_request()

    async def _send_to_all(self, payload: str) -> None:
        """Send one serialized payload to every client, dropping any that fail."""
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_text(payload)
            except Exception:  # noqa: BLE001 - a dead socket is expected, just drop it
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def broadcast(self, update: MapUpdate) -> None:
        """Async broadcast of one MapUpdate to all clients (use from within the loop)."""
        payload = self._serialize(update)
        self._latest = payload
        await self._send_to_all(payload)

    def publish(self, update: MapUpdate) -> None:
        """Thread-safe broadcast. THE bridge from the sync orchestrator into the server.

        Inputs: update - a MapUpdate to push to every connected phone.
        Output: none. Caches the frame and, if the server loop is running, schedules the
                fan-out onto it. Safe to call before any client connects or before the
                loop is up (the frame is cached and sent to the next client to connect).
        """
        payload = self._serialize(update)
        self._latest = payload
        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(self._send_to_all(payload), self._loop)

    # ---- FastAPI wiring ----

    def create_app(self):
        """Build and return the FastAPI app (websocket route + static viz)."""
        if not _FASTAPI:
            raise RuntimeError(
                "fastapi/uvicorn not installed; run: pip install -r requirements.txt"
            )
        app = FastAPI(title="Recon Rover")

        @app.on_event("startup")
        async def _capture_loop() -> None:
            # publish() (called from the orchestrator thread) needs this loop handle.
            self._loop = asyncio.get_running_loop()

        @app.websocket("/ws")
        async def ws_endpoint(ws: WebSocket) -> None:
            await ws.accept()
            self.clients.add(ws)
            if self._latest is not None:
                await ws.send_text(self._latest)  # show the latest map immediately
            try:
                while True:
                    raw = await ws.receive_text()
                    self._handle_inbound(raw)
            except WebSocketDisconnect:
                pass
            finally:
                self.clients.discard(ws)

        # Serve the static visualization. Mounted last so the /ws route takes precedence.
        if VIZ_DIR.exists():
            app.mount("/", StaticFiles(directory=str(VIZ_DIR), html=True), name="viz")
        return app

    def run_in_thread(self, host: str = "0.0.0.0", port: int = 8000) -> threading.Thread:
        """Start uvicorn in a daemon thread so the sync run loop can keep ticking.

        Returns the thread. The orchestrator calls this once at setup, then drives
        publish() each tick.
        """
        app = self.create_app()
        config = uvicorn.Config(app, host=host, port=port, log_level="warning")
        server = uvicorn.Server(config)
        thread = threading.Thread(target=server.run, daemon=True, name="map-server")
        thread.start()
        return thread
