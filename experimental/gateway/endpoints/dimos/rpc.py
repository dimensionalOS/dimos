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

import asyncio
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models, skills


def register(app: FastAPI, state: ServerState) -> None:
    @app.get(
        "/dimos/rpc",
        response_model=models.RpcList,
        **route_doc(
            "skills",
            "The running blueprint's module RPC methods: module, method, params, docstring",
            "Every `@rpc` method of the running blueprint's modules (`Coordinator/list_modules`), skills included, with its signature and docstring when its class imports in the gateway. The lifecycle methods (start, stop, build, set_transport, set_module_ref) are left out. Empty `rpcs` and null `run` when nothing runs. No side effects.",
            agent=True,
            answer="`{ rpcs: [{ module, method, class, known, params: [{ name, type, default, required, kind }], return_type, doc, skill, runId, blueprint }], run: { runId, blueprint } | null }`",
        ),
    )
    async def rpc_list() -> dict[str, Any]:
        return await asyncio.to_thread(skills.list_rpcs)
