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
        "/dimos/modules",
        response_model=models.ModuleList,
        **route_doc(
            "discovery",
            "Every module from the discovery cache: streams in and out, skills, and how many blueprints use it",
            "Answers from the cache (no import): the registry's modules and every module a blueprint uses, with `blueprint_count` (importable blueprints using it) and `robots`. No side effects.",
            agent=True,
            answer="`{ stale, modules: [{ name, class, doc, inputs, outputs, skills, blueprint_count, robots }] }`",
        ),
    )
    async def module_list() -> dict[str, Any]:
        return {"stale": bool(discovery.status["stale"]), "modules": discovery.module_list()}
