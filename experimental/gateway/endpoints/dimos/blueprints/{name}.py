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
from experimental.gateway.utils.api_helpers import ApiHelpers
from experimental.gateway.utils.http import BlueprintParam


def register(app: FastAPI, state: ServerState) -> None:
    helpers = ApiHelpers(state)

    @app.get(
        "/dimos/blueprints/{name}",
        response_model=models.Blueprint,
        **route_doc(
            "blueprints",
            "A blueprint's modules and each module's streams (topics, types, in/out), docstring, RPC methods, skills and source location",
            "Imports the blueprint in a child process (180 s timeout; cached 10 min) and lists its modules with their streams' names, message types, directions and wired topics, each module's docstring (whole, and its first paragraph as `summary`), RPC methods and skills (signature and docstring) and where its class is defined (`file`, `line`; GET /dimos/source reads the file). 400 for a name that can't be one, 500 when the blueprint can't be found or imported.",
            errors=(400, 500),
            agent=True,
            answer="`{ name, modules: [{ name, class, doc, summary, file, line, rpcs, skills, streams: [{ name, type, direction, topic }] }] }`",
        ),
    )
    async def blueprint(name: BlueprintParam) -> Any:
        helpers.check_name(name)
        return await helpers.introspected(f"bp:{name}", ["blueprint", name])
