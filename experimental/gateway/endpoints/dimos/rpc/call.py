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
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    @app.post(
        "/dimos/rpc/call",
        response_model=models.RpcCallResult,
        **route_doc(
            "skills",
            "Call a module's RPC method and wait for its answer",
            "Calls `<module>/<method>` over dimos's module RPC. `args` is an object (by name) or an array (by position), checked against its `params`. This can act on the robot. Waits up to 300 s. `ok` false when the method raised. 400 for start or stop (or another lifecycle method: the coordinator runs those) and for a missing, extra or unknown argument, 404 when the running blueprint has no such module or method, 409 when nothing runs, 500 when it didn't answer in time.",
            errors=(400, 404, 409, 500),
            agent=True,
            answer="`{ module, method, runId, blueprint, ok, result, text }`",
        ),
    )
    async def rpc_call(request: models.RpcCallRequest) -> dict[str, Any]:
        try:
            return await asyncio.to_thread(
                skills.call_rpc, request.module, request.method, request.args
            )
        except skills.SkillError as error:
            raise ApiError(error.status, str(error))
