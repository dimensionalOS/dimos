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
from packaging.markers import default_environment

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import extras, models


def register(app: FastAPI, state: ServerState) -> None:
    s = state
    discovery = state.discovery
    assert discovery is not None

    @app.get(
        "/dimos/discovery/blueprints",
        response_model=models.DiscoveredBlueprints,
        **route_doc(
            "discovery",
            "Every blueprint from the discovery cache: whether it imports (and why not), its robot, its modules and their streams with topics",
            "Answers from the cache (no import): every blueprint scanned so far, in registry order, with `suggested_extras` for one that's missing a package. No side effects.",
            agent=True,
            answer="`{ stale, blueprints: [{ name, ref, robot, importable, import_error, missing_module, suggested_extras, modules: [{ name, class, module, streams: [{ name, type, direction, topic }] }] }] }`",
        ),
    )
    async def discovered_blueprints() -> dict[str, Any]:
        providers = extras.providing_extras(
            s.dimos_dir, {key: str(value) for (key, value) in default_environment().items()}
        )
        records = [
            {**record, "suggested_extras": providers(record.get("missing_module"))}
            for record in discovery.blueprint_list()
        ]
        return {"stale": bool(discovery.status["stale"]), "blueprints": records}
