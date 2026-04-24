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

"""WebSocket server module that receives events from dimos-viewer.

When dimos-viewer is started with ``--connect``, LCM multicast is unavailable
across machines. The viewer falls back to sending click, twist, and stop events
as JSON over a WebSocket connection. This module acts as the server-side
counterpart: it listens for those connections and translates incoming messages
into DimOS stream publishes.

Message format (newline-delimited JSON, ``"type"`` discriminant):

    {"type":"heartbeat","timestamp_ms":1234567890}
    {"type":"click","x":1.0,"y":2.0,"z":3.0,"entity_path":"/world","timestamp_ms":1234567890}
    {"type":"twist","linear_x":0.5,"linear_y":0.0,"linear_z":0.0,
                    "angular_x":0.0,"angular_y":0.0,"angular_z":0.8}
    {"type":"stop"}
"""

from __future__ import annotations

import asyncio
import json
import logging
import socket
import threading
from typing import Any, Literal, TypedDict, Union

import websockets
import websockets.asyncio.server as ws_server

from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.generic import get_local_ips
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

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


class Config(ModuleConfig):
    host: str | None = None
    port: int = 3030
    start_timeout: float = 10.0


class RerunWebSocketServer(Module):
    """Receives dimos-viewer WebSocket events and publishes them as DimOS streams.

    The viewer connects to this module (not the other way around) when running
    in ``--connect`` mode. Each click event is converted to a ``PointStamped``
    and published on the ``clicked_point`` stream so downstream modules (e.g.
    ``ReplanningAStarPlanner``) can consume it without modification.

    Outputs:
        clicked_point: 3-D world-space point from the most recent viewer click.
        tele_cmd_vel: Twist velocity commands from keyboard teleop, including stop events.

    Note: ``stop_movement`` is owned by ``MovementManager`` — it will fire
    that signal when it sees the first teleop twist arrive here.
    """

    config: Config

    clicked_point: Out[PointStamped]
    tele_cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event: asyncio.Event | None = None
        self._server_ready = threading.Event()
        self.host = self.config.host if self.config.host is not None else global_config.listen_host

    @rpc
    def start(self) -> None:
        super().start()
        assert self._loop is not None
        asyncio.run_coroutine_threadsafe(self._serve(), self._loop)
        self._server_ready.wait(timeout=self.config.start_timeout)
        self._log_connect_hints()

    @rpc
    def stop(self) -> None:
        self._server_ready.wait(timeout=self.config.start_timeout)
        if self._loop is not None and not self._loop.is_closed() and self._stop_event is not None:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        super().stop()

    def _log_connect_hints(self) -> None:
        """Log full dimos-viewer commands that viewers can use to connect."""
        local_ips = get_local_ips()
        hostname = socket.gethostname()
        host = self.host
        ws_url = f"ws://{host}:{self.config.port}/ws"
        grpc_url = f"rerun+http://{host}:{RERUN_GRPC_PORT}/proxy"

        lines = [
            "",
            "=" * 60,
            f"RerunWebSocketServer listening on {ws_url}",
            "",
            "Connect a viewer:",
            f"  dimos-viewer --connect {grpc_url} --ws-url {ws_url}",
        ]
        if local_ips:
            lines.append("")
            lines.append("From another machine on the network:")
            for ip, iface in local_ips:
                remote_grpc = f"rerun+http://{ip}:{RERUN_GRPC_PORT}/proxy"
                remote_ws = f"ws://{ip}:{self.config.port}/ws"
                lines.append(
                    f"  dimos-viewer --connect {remote_grpc} --ws-url {remote_ws}  # {iface}"
                )
            lines.append("")
        lines.append(f"  hostname: {hostname}")
        lines.append("=" * 60)
        lines.append("")

        logger.info("\n".join(lines))

    async def _serve(self) -> None:
        self._stop_event = asyncio.Event()

        ws_logger = logging.getLogger("websockets.server")
        ws_logger.addFilter(_handshake_noise_filter)

        async with ws_server.serve(
            self._handle_client,
            host=self.host,
            port=self.config.port,
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
