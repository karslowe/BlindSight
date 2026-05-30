"""Web server: serve the visualization and stream MapUpdate to the phone.

Interface stub only. FastAPI app plus a websocket. Runs on the UNO Q, which also hosts the
Wi-Fi access point the phone joins. No cloud dependency.

Pushes MapUpdate messages (occupancy grid + current pose + planned return path) to every
connected client. Receives the single "return home" command back from the phone.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import MapUpdate  # noqa: E402

try:
    from fastapi import FastAPI, WebSocket
    from fastapi.staticfiles import StaticFiles
except ImportError:  # keep the module importable before deps are installed
    FastAPI = None  # type: ignore
    WebSocket = object  # type: ignore
    StaticFiles = None  # type: ignore


# Path to the built visualization served statically from the rover.
VIZ_DIR = Path(__file__).resolve().parents[2] / "visualization"


class MapServer:
    """Holds connected websocket clients and broadcasts MapUpdate to them."""

    def __init__(self) -> None:
        self.clients: List[WebSocket] = []
        # TODO: a hook the orchestrator calls when the phone requests "return home".
        self.on_return_request = None

    def create_app(self):
        """Build and return the FastAPI app.

        Output: a FastAPI instance with a websocket route and static file serving.
        TODO: mount VIZ_DIR as static files; add a /ws websocket route that registers
              the client, then reads inbound "return home" messages and dispatches them
              to self.on_return_request.
        """
        if FastAPI is None:
            raise RuntimeError("fastapi not installed; see requirements.txt")
        app = FastAPI(title="Recon Rover")

        @app.websocket("/ws")
        async def ws_endpoint(ws: WebSocket):  # noqa: ANN202
            # TODO: accept the socket, append to self.clients, loop reading commands.
            raise NotImplementedError("websocket endpoint not implemented yet")

        # TODO: app.mount("/", StaticFiles(directory=VIZ_DIR, html=True), name="viz")
        return app

    async def broadcast(self, update: MapUpdate) -> None:
        """Push one MapUpdate to every connected client.

        Inputs: update, the MapUpdate to send.
        Output: none.
        TODO: json-encode update.to_dict() and send_text to each client; drop dead ones.
        """
        raise NotImplementedError("broadcast not implemented yet")
