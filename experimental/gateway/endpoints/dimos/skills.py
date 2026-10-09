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
from experimental.gateway.utils.skills_helpers import SkillsHelpers


def register(app: FastAPI, state: ServerState) -> None:
    helpers = SkillsHelpers(state)

    @app.get(
        "/dimos/skills",
        response_model=models.SkillList,
        **route_doc(
            "skills",
            "The running blueprint's skills: name, module, description, params",
            "Every `@skill` method of the running blueprint's modules, whether or not it has an agent: over dimos's module RPC, `Coordinator/list_modules` then each module's `get_skills` (the JSON schema McpServer gives an agent). Empty `skills` and null `run` when nothing runs; `errors` names a module that didn't answer. No side effects.",
            agent=True,
            answer="`{ skills: [{ name, module, description, params, required, lifecycle, uses, runId, blueprint }], run: { runId, blueprint } | null, errors: [{ module, error }] }`",
        ),
    )
    async def skill_list() -> dict[str, Any]:
        return await helpers.listed()
