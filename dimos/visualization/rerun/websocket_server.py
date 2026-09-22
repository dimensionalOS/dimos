# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import logging
import threading
import time
from typing import Any, Literal, TypedDict, Union

import websockets
import websockets.asyncio.server as ws_server

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ClickMsg(TypedDict):
    type: Literal["click"]
    x: float
    y: float
    z: float
    entity_path: str
    timestamp_ms: int


class TwistMsg(TypedDict):
    type: Literal["twist"]
    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float


class StopMsg(TypedDict):
    type: Literal["stop"]


class HeartbeatMsg(TypedDict):
    type: Literal["heartbeat"]
    timestamp_ms: int


ViewerMsg = Union[ClickMsg, TwistMsg, StopMsg, HeartbeatMsg]


def _handshake_noise_filter(record: logging.LogRecord) -> bool:
    """Drop noisy "opening handshake failed" records from port scanners etc."""
    msg = record.getMessage()
    return not ("opening handshake failed" in msg or "did not receive a valid HTTP request" in msg)


class RerunWebSocketServer(Module):
    """This handles outputs from dimos-viewer (like keyboard controls)"""

    clicked_point: Out[PointStamped]
    tele_cmd_vel: Out[Twist]

    _STARTUP_TIMEOUT = 10.0

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event: asyncio.Event | None = None
        self._server_ready = threading.Event()
        self._serve_future: concurrent.futures.Future[None] | None = None
        self._bound_port: int | None = None
        # Set on the first WebSocket client connection. Tests use this to verify
        # an external client (e.g. dimos-viewer --connect) actually connected.
        self.client_connected = threading.Event()

    @property
    def host(self) -> str:
        return self.config.g.rerun_host or self.config.g.listen_host

    @property
    def port(self) -> int:
        return self.config.g.rerun_websocket_server_port

    @property
    def bound_port(self) -> int:
        """Actual listening port (differs from `port` when configured as 0)."""
        assert self._bound_port is not None, "server not started"
        return self._bound_port

    @rpc
    def start(self) -> None:
        if self._loop is None:
            # A Module's loop dies with stop(); a stopped server cannot come back.
            raise RuntimeError("RerunWebSocketServer cannot be restarted; construct a new one")
        super().start()
        self._serve_future = asyncio.run_coroutine_threadsafe(self._serve(), self._loop)
        deadline = time.monotonic() + self._STARTUP_TIMEOUT
        while not self._server_ready.wait(timeout=0.05):
            if self._serve_future.done():
                self._serve_future.result()  # surfaces bind failures loudly
                raise RuntimeError("RerunWebSocketServer exited before becoming ready")
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"RerunWebSocketServer did not become ready on "
                    f"{self.host}:{self.port} within {self._STARTUP_TIMEOUT}s"
                )

    @rpc
    def stop(self) -> None:
        future = self._serve_future
        self._serve_future = None
        if (
            future is not None
            and not future.done()
            and self._loop is not None
            and not self._loop.is_closed()
            and self._stop_event is not None
        ):
            self._loop.call_soon_threadsafe(self._stop_event.set)
            try:
                # Wait for the serve coroutine to finish so the listening
                # socket is actually closed (and the port free) before the
                # module loop is torn down.
                future.result(timeout=5.0)
            except Exception:
                logger.warning("RerunWebSocketServer: serve task did not shut down cleanly")
        super().stop()
        self._server_ready.clear()
        self._bound_port = None
        self.client_connected.clear()

    async def _serve(self) -> None:
        self._stop_event = asyncio.Event()

        ws_logger = logging.getLogger("websockets.server")
        ws_logger.addFilter(_handshake_noise_filter)

        async with ws_server.serve(
            self._handle_client,
            host=self.host,
            port=self.port,
            ping_interval=30,
            ping_timeout=30,
            logger=ws_logger,
        ) as server:
            self._bound_port = next(iter(server.sockets)).getsockname()[1]
            self._server_ready.set()
            await self._stop_event.wait()

    async def _handle_client(self, websocket: Any) -> None:
        if hasattr(websocket, "request") and websocket.request.path != "/ws":
            await websocket.close(1008, "Not Found")
            return
        addr = websocket.remote_address
        logger.info(f"RerunWebSocketServer: viewer connected from {addr}")
        self.client_connected.set()
        try:
            async for raw in websocket:
                self._dispatch(raw)
        except websockets.ConnectionClosed:
            pass

    def _dispatch(self, raw: str | bytes) -> None:
        try:
            msg: dict[str, Any] = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning(f"RerunWebSocketServer: ignoring non-JSON message: {raw!r}")
            return

        if not isinstance(msg, dict):
            return

        msg_type = msg.get("type")

        # dict.get's default is only used when the key is missing — if Rerun
        # sends a 2D-panel click the "z" key is present with value None, and
        # `float(None)` raises.  Coerce explicitly.
        def _num(v: Any) -> float:
            return float(v) if v is not None else 0.0

        if msg_type == "click":
            self.clicked_point.publish(
                PointStamped(
                    x=_num(msg.get("x")),
                    y=_num(msg.get("y")),
                    z=_num(msg.get("z")),
                    ts=_num(msg.get("timestamp_ms")) / 1000.0,
                    frame_id=str(msg.get("entity_path", "")),
                )
            )

        elif msg_type == "twist":
            self.tele_cmd_vel.publish(
                Twist(
                    linear=Vector3(
                        _num(msg.get("linear_x")),
                        _num(msg.get("linear_y")),
                        _num(msg.get("linear_z")),
                    ),
                    angular=Vector3(
                        _num(msg.get("angular_x")),
                        _num(msg.get("angular_y")),
                        _num(msg.get("angular_z")),
                    ),
                )
            )

        elif msg_type == "stop":
            self.tele_cmd_vel.publish(Twist.zero())
