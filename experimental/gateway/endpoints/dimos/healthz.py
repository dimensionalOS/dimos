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

from __future__ import annotations

from fastapi import FastAPI
from fastapi.responses import PlainTextResponse

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState


def register(app: FastAPI, state: ServerState) -> None:
    @app.get(
        "/dimos/healthz",
        response_class=PlainTextResponse,
        **route_doc(
            "server",
            "The dimos gateway's liveness: `ok`",
            "Answers `ok` (text/plain) while the gateway runs; no side effects. Desktop polls it after starting the server.",
            errors=(),
            ok={"content": {"text/plain": {"schema": {"type": "string", "example": "ok"}}}},
            answer="`ok`",
        ),
    )
    async def healthz() -> str:
        return "ok"
