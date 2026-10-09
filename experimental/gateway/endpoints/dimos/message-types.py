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

from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models


def register(app: FastAPI, state: ServerState) -> None:
    discovery = state.discovery
    assert discovery is not None

    @app.get(
        "/dimos/message-types",
        response_model=models.MessageTypes,
        **route_doc(
            "discovery",
            "Every message type on a module's stream, with the modules that publish and read it",
            "Answers from the discovery cache. No side effects.",
            agent=True,
            answer="`{ types: [{ type, publishers, subscribers }] }`",
        ),
    )
    async def message_types() -> dict[str, Any]:
        return {"types": discovery.message_types()}
