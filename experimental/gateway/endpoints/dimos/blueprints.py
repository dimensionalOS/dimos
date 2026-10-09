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
from experimental.gateway.utils.http import FreshQuery


def register(app: FastAPI, state: ServerState) -> None:
    helpers = ApiHelpers(state)

    @app.get(
        "/dimos/blueprints",
        response_model=models.BlueprintList,
        **route_doc(
            "blueprints",
            "Every blueprint dimos can run (name, builtin/external) and whether it imports",
            "What `dimos list` prints: built-in blueprints (without demo-*), then external ones from installed packages. Cached for 60 s and re-listed (in a child) whenever dimos/robot or site-packages change, with a `blueprints` event; `fresh` re-lists first. No other side effects. `importable`, `import_error` and `missing_module` come from the discovery cache (null until the scan reaches the blueprint: GET /dimos/discovery).",
            errors=(400, 500),
            agent=True,
            answer='`{ blueprints: [{ name, kind: "builtin"|"external", importable, import_error, missing_module }] }`',
        ),
    )
    async def blueprint_list(fresh: FreshQuery = False) -> dict[str, Any]:
        return await helpers.blueprint_list(fresh)
