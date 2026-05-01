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
import json
import logging
import threading
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

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event: asyncio.Event | None = None
        self._server_ready = threading.Event()

    @property
    def host(self) -> str:
        return self.config.g.rerun_host or self.config.g.listen_host

    @property
    def port(self) -> int:
        return self.config.g.rerun_websocket_server_port

    @rpc
    def start(self) -> None:
        super().start()
        assert self._loop is not None
        asyncio.run_coroutine_threadsafe(self._serve(), self._loop)
        self._server_ready.wait()

    @rpc
    def stop(self) -> None:
        if not self._server_ready.is_set():
            super().stop()
            return
        if self._loop is not None and not self._loop.is_closed() and self._stop_event is not None:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        super().stop()

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
        ):
            self._server_ready.set()
            await self._stop_event.wait()

    async def _handle_client(self, websocket: Any) -> None:
        if hasattr(websocket, "request") and websocket.request.path != "/ws":
            await websocket.close(1008, "Not Found")
            return
        addr = websocket.remote_address
        logger.info(f"RerunWebSocketServer: viewer connected from {addr}")
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

        if msg_type == "click":
            self.clicked_point.publish(
                PointStamped(
                    x=float(msg.get("x", 0)),
                    y=float(msg.get("y", 0)),
                    z=float(msg.get("z", 0)),
                    ts=float(msg.get("timestamp_ms", 0)) / 1000.0,
                    frame_id=str(msg.get("entity_path", "")),
                )
            )

        elif msg_type == "twist":
            self.tele_cmd_vel.publish(
                Twist(
                    linear=Vector3(
                        float(msg.get("linear_x", 0)),
                        float(msg.get("linear_y", 0)),
                        float(msg.get("linear_z", 0)),
                    ),
                    angular=Vector3(
                        float(msg.get("angular_x", 0)),
                        float(msg.get("angular_y", 0)),
                        float(msg.get("angular_z", 0)),
                    ),
                )
            )

        elif msg_type == "stop":
            self.tele_cmd_vel.publish(Twist.zero())
