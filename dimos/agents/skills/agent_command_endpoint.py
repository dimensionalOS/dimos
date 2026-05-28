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

"""Token-guarded HTTP endpoint to inject agent commands from a remote web app.

Exposes a single `POST /agent-command` that, given a valid bearer token,
publishes the text to the LCM `/human_input` topic — exactly what the
`agent_send` MCP tool (and `dimos agent-send`) does — so the running agent
receives it as a HumanMessage and acts on it.

Designed to sit behind a tunnel (e.g. `ngrok http 8770`): the robomoo web app
forwards browser commands here with `Authorization: Bearer $AGENT_COMMAND_TOKEN`,
keeping the URL + token server-side. Unlike the unauthenticated WebInput server
on :5555, this endpoint requires the shared secret, so a leaked URL alone can't
drive the robot.

Env: AGENT_COMMAND_TOKEN (required — endpoint rejects everything until set).
"""

from collections.abc import Callable
import os
from threading import Thread
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.transport import pLCMTransport
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def build_app(token: str, publish: Callable[[str], None]) -> Any:
    """Build the FastAPI app: a single token-guarded POST /agent-command that
    forwards `text` to `publish`. Factored out so it can be tested without the
    module/transport framework."""
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse

    app = FastAPI()

    @app.get("/health")
    async def health() -> dict[str, bool]:
        return {"ok": True}

    @app.post("/agent-command")
    async def agent_command(request: Request) -> JSONResponse:
        auth = request.headers.get("authorization", "")
        presented = auth[7:] if auth.lower().startswith("bearer ") else ""
        if not token or presented != token:
            return JSONResponse({"ok": False, "error": "unauthorized"}, status_code=401)
        try:
            body = await request.json()
        except Exception:
            return JSONResponse({"ok": False, "error": "bad json"}, status_code=400)
        text = (body or {}).get("text", "")
        if not isinstance(text, str) or not text.strip():
            return JSONResponse({"ok": False, "error": "missing 'text'"}, status_code=400)
        publish(text)
        logger.info("agent-command received: %s", text[:100])
        return JSONResponse({"ok": True})

    return app


class AgentCommandEndpointConfig(ModuleConfig):
    port: int = 8770
    host: str = "0.0.0.0"
    token: str = os.getenv("AGENT_COMMAND_TOKEN", "")


class AgentCommandEndpoint(Module):
    config: AgentCommandEndpointConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._transport: pLCMTransport[str] | None = None
        self._server: Any | None = None
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.token:
            logger.warning(
                "AgentCommandEndpoint disabled: AGENT_COMMAND_TOKEN not set"
            )
            return

        import uvicorn

        self._transport = pLCMTransport("/human_input")
        self._transport.start()
        transport = self._transport

        app = build_app(self.config.token, lambda text: transport.publish(text))

        config = uvicorn.Config(app, host=self.config.host, port=self.config.port, log_level="warning")
        self._server = uvicorn.Server(config)
        self._thread = Thread(target=self._server.run, daemon=True, name="agent-command-endpoint")
        self._thread.start()
        logger.info("AgentCommandEndpoint listening on %s:%d", self.config.host, self.config.port)

    @rpc
    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._transport is not None:
            self._transport.stop()
        super().stop()
