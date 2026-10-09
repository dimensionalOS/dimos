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
from experimental.gateway.utils import models, skills
from experimental.gateway.utils.http import ApiError
from experimental.gateway.utils.skills_helpers import SkillsHelpers


def register(app: FastAPI, state: ServerState) -> None:
    helpers = SkillsHelpers(state)

    @app.post(
        "/dimos/skills/call",
        response_model=models.SkillCallResult,
        **route_doc(
            "skills",
            "Call a skill of the running blueprint and wait for its answer",
            "Calls `<module>/<skill>` over dimos's module RPC, as the coordinator calls a module's start. A skill that holds a capability (`uses`) goes through the run's McpServer instead when one answers (`via: mcp`), so its agent's capability locks cover it: it waits or is refused while another skill holds one. A background skill answers at once. This acts on the robot. Waits up to 300 s. `ok` false when the skill itself failed. 400 for a missing or unknown argument (checked against `params`) or a name two modules share without `module`, 404 when the running blueprint has no such skill (or `runId` isn't it), 409 when nothing runs, 500 when the MCP call failed or it didn't answer in time.",
            errors=(400, 404, 409, 500),
            agent=True,
            answer="`{ skill, module, runId, blueprint, via, ok, text, content }`",
        ),
    )
    async def skill_call(request: models.SkillCallRequest) -> dict[str, Any]:
        try:
            return await helpers.called(request.skill, request.args, request.module, request.runId)
        except skills.SkillError as error:
            raise ApiError(error.status, str(error))
