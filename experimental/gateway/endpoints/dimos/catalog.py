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


def register(app: FastAPI, state: ServerState) -> None:
    helpers = ApiHelpers(state)

    @app.get(
        "/dimos/catalog",
        response_model=models.Catalog,
        **route_doc(
            "blueprints",
            "Every blueprint, module and skill (imports them all, in a child process: slow the first time)",
            "Imports every built-in blueprint and module in a child process (180 s timeout; cached 10 min) and lists blueprints with their robot and modules, modules with their streams and skills, and every skill with its parameters. What fails to import is listed in `errors`; the rest still answers.",
            answer="`{ blueprints: [{ name, ref, robot, modules }], modules: [{ name, class, doc, robots, inputs, outputs, skills }], skills: [{ name, doc, params, module, robots }], errors }`",
        ),
    )
    async def catalog() -> Any:
        return await helpers.introspected("catalog", ["catalog"])
