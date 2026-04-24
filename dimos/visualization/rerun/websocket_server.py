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
import logging
import threading
from typing import Any

from pydantic import BaseModel
import websockets

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _handshake_noise_filter(record: logging.LogRecord) -> bool:
    """Drop noisy "opening handshake failed" records from port scanners etc."""
    msg = record.getMessage()
    return not ("opening handshake failed" in msg or "did not receive a valid HTTP request" in msg)


class CmdVelScaling(BaseModel):
    """Per-dimension multipliers applied to outgoing teleop cmd_vel twists.

    ``x``/``y``/``z`` scale ``linear.x``/``linear.y``/``linear.z``.
    ``roll``/``pitch``/``yaw`` scale ``angular.x``/``angular.y``/``angular.z``
    (ROS convention: roll around X, pitch around Y, yaw around Z).

    Defaults are all ``1.0`` — identity passthrough. Set to ``0.0`` to
    lock out a dimension entirely, or to a fraction (e.g. ``0.3``) to
    cap the operator's effective speed on that axis.
    """

    x: float = 1.0
    y: float = 1.0
    z: float = 1.0
    roll: float = 1.0
    pitch: float = 1.0
    yaw: float = 1.0


class Config(ModuleConfig):
    # Intentionally binds 0.0.0.0 by default so the viewer can connect from
    # any machine on the network (the typical robot deployment scenario).
    host: str = "0.0.0.0"
    port: int = 3030
    start_timeout: float = 10.0  # seconds to wait for the server to bind
    cmd_vel_scaling: CmdVelScaling = CmdVelScaling()


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
        # Join the server thread so tests that check for thread leaks pass,
        # and so a subsequent start() doesn't race with a still-running
        # previous instance on the same port.
        if self._server_thread is not None and self._server_thread.is_alive():
            self._server_thread.join(timeout=self.config.start_timeout)
        super().stop()

    def _log_connect_hints(self) -> None:
        """Log full dimos-viewer commands that viewers can use to connect."""
        import socket

        from dimos.utils.generic import get_local_ips
        from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

        local_ips = get_local_ips()
        hostname = socket.gethostname()
        ws_url = f"ws://127.0.0.1:{self.config.port}/ws"
        grpc_url = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

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

        # Filter out handshake failures from port scanners / gRPC probes /
        # health checks — they log at ERROR level with the message
        # "opening handshake failed" and aren't actionable.
        ws_logger = logging.getLogger("websockets.server")
        ws_logger.addFilter(_handshake_noise_filter)

        async with ws_server.serve(
            self._handle_client,
            host=self.config.host,
            port=self.config.port,
            # Ping every 30 s, allow 30 s for pong — generous enough to
            # survive brief network hiccups while still detecting dead clients.
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
        except websockets.ConnectionClosed as exc:
            logger.debug(f"RerunWebSocketServer: client {addr} disconnected ({exc})")

    def _dispatch(self, raw: str | bytes) -> None:
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
            s = self.config.cmd_vel_scaling
            twist = Twist(
                linear=Vector3(
                    float(msg.get("linear_x", 0)) * s.x,
                    float(msg.get("linear_y", 0)) * s.y,
                    float(msg.get("linear_z", 0)) * s.z,
                ),
                angular=Vector3(
                    float(msg.get("angular_x", 0)) * s.roll,
                    float(msg.get("angular_y", 0)) * s.pitch,
                    float(msg.get("angular_z", 0)) * s.yaw,
                ),
            )
            logger.debug(f"RerunWebSocketServer: twist → {twist}")
            self.tele_cmd_vel.publish(twist)

        elif msg_type == "stop":
            logger.debug("RerunWebSocketServer: stop")
            self.tele_cmd_vel.publish(Twist.zero())

        elif msg_type == "heartbeat":
            logger.debug(f"RerunWebSocketServer: heartbeat ts={msg.get('timestamp_ms')}")

        else:
            logger.warning(f"RerunWebSocketServer: unknown message type {msg_type!r}")
