# Copyright 2026 Dimensional Inc.
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

"""Standalone server for the LCM <-> WebSocket bridge.

Add it to any blueprint and every bus topic becomes reachable from a
browser tab on the same host/LAN:

    autoconnect(
        my_robot_blueprint,
        LcmWebSocketBridgeModule.blueprint(port=9669),
    )

Browser side (see ``static/lcm_client.js``, served at ``/lcm_client.js``):

    <script type="module" src="http://<host>:9669/lcm_client.js"></script>
    dimosLcm.subscribe("/odom", dimosMsgs.geometry_msgs.PoseStamped, cb)
    dimosLcm.publish("/cmd_vel", new dimosMsgs.geometry_msgs.Twist(...))

``GET /`` serves a JSON status page (client count, forward/drop counters,
active filter config) for quick debugging.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import threading
from typing import Any

from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Route
import uvicorn

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.utils.logging_config import setup_logger
from dimos.web.lcm_bridge.bridge import LcmWebSocketBridge

logger = setup_logger()

_STATIC_DIR = Path(__file__).parent / "static"


class LcmWebSocketBridgeModule(Module):
    """Serves :class:`LcmWebSocketBridge` on its own port.

    The module has no In/Out ports: it taps the LCM bus directly with a
    subscribe-all handle, like the rerun bridge, so it composes into any
    blueprint without wiring. Constructor arguments mirror the bridge's
    (fnmatch patterns for filtering and rate caps); requires the ``web``
    extra (starlette + uvicorn).
    """

    def __init__(
        self,
        port: int = 9669,
        host: str = "0.0.0.0",
        channel_rate_hz: Mapping[str, float] | None = None,
        topic_allowlist: list[str] | None = None,
        topic_blocklist: list[str] | None = None,
        lcm_url: str | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._port = port
        self._host = host
        self._bridge = LcmWebSocketBridge(
            lcm_url=lcm_url,
            channel_rate_hz=channel_rate_hz,
            topic_allowlist=topic_allowlist,
            topic_blocklist=topic_blocklist,
        )
        self._uvicorn_server: uvicorn.Server | None = None
        self._server_thread: threading.Thread | None = None

    @property
    def bridge(self) -> LcmWebSocketBridge:
        return self._bridge

    @property
    def port(self) -> int:
        return self._port

    @rpc
    def start(self) -> None:
        super().start()
        self._bridge.start()
        config = uvicorn.Config(
            self._create_app(), host=self._host, port=self._port, log_level="warning"
        )
        self._uvicorn_server = uvicorn.Server(config)
        self._server_thread = threading.Thread(
            target=self._uvicorn_server.run,
            name="lcm-ws-bridge-server",
            daemon=True,
        )
        self._server_thread.start()
        logger.info("LCM websocket bridge: ws://localhost:%s/lcm-ws", self._port)

    @rpc
    def stop(self) -> None:
        self._bridge.stop()
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True
        if self._server_thread is not None and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        super().stop()

    def _create_app(self) -> Starlette:
        return Starlette(
            routes=[
                Route("/", self._status),
                Route("/lcm_client.js", self._client_js),
                *self._bridge.routes(),
            ]
        )

    async def _status(self, request: Request) -> JSONResponse:
        bridge = self._bridge
        return JSONResponse(
            {
                "clients": bridge.client_count,
                "forwarded": bridge.forwarded,
                "rate_capped": bridge.rate_capped,
                "filtered": bridge.filtered,
                "published_from_clients": bridge.published_from_clients,
                "channel_rate_hz": bridge._channel_rate_hz,
                "topic_allowlist": bridge._topic_allowlist,
                "topic_blocklist": bridge._topic_blocklist,
            }
        )

    async def _client_js(self, request: Request) -> FileResponse:
        return FileResponse(_STATIC_DIR / "lcm_client.js", media_type="text/javascript")
