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

import asyncio
import json
import threading
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import websockets

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Config(ModuleConfig):
    # Intentionally binds 0.0.0.0 by default so the viewer can connect from
    # any machine on the network (the typical robot deployment scenario).
    host: str = "0.0.0.0"
    port: int = 3030
    start_timeout: float = 10.0  # seconds to wait for the server to bind


class RerunWebSocketServer(Module[Config]):
    """Receives dimos-viewer WebSocket events and publishes them as DimOS streams.

    The viewer connects to this module (not the other way around) when running
    in ``--connect`` mode. Each click event is converted to a ``PointStamped``
    and published on the ``clicked_point`` stream so downstream modules (e.g.
    ``ReplanningAStarPlanner``) can consume it without modification.

    Outputs:
        clicked_point: 3-D world-space point from the most recent viewer click.
        tele_cmd_vel: Twist velocity commands from keyboard teleop, including stop events.
        stop_movement: Published when teleop starts — signals nav to cancel the active goal.
    """

    default_config = Config

    clicked_point: Out[PointStamped]
    cmd_vel: Out[Twist]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._teleop_clients: set[int] = set()  # ids of clients currently in teleop
        self._ws_loop: asyncio.AbstractEventLoop | None = None
        self._server_thread: threading.Thread | None = None
        self._stop_event: asyncio.Event | None = None
        self._server_ready = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True, name="rerun-ws-server"
        )
        self._server_thread.start()
        self._server_ready.wait(timeout=self.config.start_timeout)
        self._log_connect_hints()

    @rpc
    def stop(self) -> None:
        # Wait briefly for the server thread to initialise _stop_event so we
        # don't silently skip the shutdown signal (race with _serve()).
        self._server_ready.wait(timeout=self.config.start_timeout)
        if (
            self._ws_loop is not None
            and not self._ws_loop.is_closed()
            and self._stop_event is not None
        ):
            self._ws_loop.call_soon_threadsafe(self._stop_event.set)
        super().stop()

    def _log_connect_hints(self) -> None:
        """Log the WebSocket URL(s) that viewers should connect to."""
        import socket

        from dimos.utils.generic import get_local_ips

        local_ips = get_local_ips()
        hostname = socket.gethostname()
        ws_url = f"ws://127.0.0.1:{self.config.port}/ws"

        lines = [
            "",
            "=" * 60,
            f"RerunWebSocketServer listening on {ws_url}",
            "",
        ]
        if local_ips:
            lines.append("From another machine on the network:")
            for ip, iface in local_ips:
                lines.append(f"  ws://{ip}:{self.config.port}/ws  # {iface}")
            lines.append("")
        lines.append(f"  hostname: {hostname}")
        lines.append("=" * 60)
        lines.append("")

        logger.info("\n".join(lines))

    def _run_server(self) -> None:
        """Entry point for the background server thread."""
        self._ws_loop = asyncio.new_event_loop()
        try:
            self._ws_loop.run_until_complete(self._serve())
        except Exception:
            logger.error("RerunWebSocketServer failed to start", exc_info=True)
        finally:
            self._server_ready.set()  # unblock stop() even on failure
            self._ws_loop.close()

    async def _serve(self) -> None:
        import websockets.asyncio.server as ws_server

        self._stop_event = asyncio.Event()

        async with ws_server.serve(
            self._handle_client,
            host=self.config.host,
            port=self.config.port,
            # Ping every 30 s, allow 30 s for pong — generous enough to
            # survive brief network hiccups while still detecting dead clients.
            ping_interval=30,
            ping_timeout=30,
        ):
            self._server_ready.set()
            await self._stop_event.wait()

    async def _handle_client(self, websocket: Any) -> None:
        if hasattr(websocket, "request") and websocket.request.path != "/ws":
            await websocket.close(1008, "Not Found")
            return
        addr = websocket.remote_address
        client_id = id(websocket)
        logger.info(f"RerunWebSocketServer: viewer connected from {addr}")
        try:
            async for raw in websocket:
                self._dispatch(raw, client_id)
        except websockets.ConnectionClosed as exc:
            logger.debug(f"RerunWebSocketServer: client {addr} disconnected ({exc})")
        finally:
            self._teleop_clients.discard(client_id)

    def _dispatch(self, raw: str | bytes, client_id: int) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning(f"RerunWebSocketServer: ignoring non-JSON message: {raw!r}")
            return

        if not isinstance(msg, dict):
            logger.warning(f"RerunWebSocketServer: expected JSON object, got {type(msg).__name__}")
            return

        msg_type = msg.get("type")

        if msg_type == "click":
            pt = PointStamped(
                x=float(msg.get("x", 0)),
                y=float(msg.get("y", 0)),
                z=float(msg.get("z", 0)),
                ts=float(msg.get("timestamp_ms", 0)) / 1000.0,
                frame_id=str(msg.get("entity_path", "")),
            )
            logger.debug(f"RerunWebSocketServer: click → {pt}")
            self.clicked_point.publish(pt)

        elif msg_type == "twist":
            twist = Twist(
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
            logger.debug(f"RerunWebSocketServer: twist → {twist}")
            if not self._teleop_clients:
                self.stop_movement.publish(Bool(data=True))
            self._teleop_clients.add(client_id)
            self.cmd_vel.publish(twist)

        elif msg_type == "stop":
            logger.debug("RerunWebSocketServer: stop")
            self._teleop_clients.discard(client_id)
            self.cmd_vel.publish(Twist.zero())

        elif msg_type == "heartbeat":
            logger.debug(f"RerunWebSocketServer: heartbeat ts={msg.get('timestamp_ms')}")

        else:
            logger.warning(f"RerunWebSocketServer: unknown message type {msg_type!r}")
