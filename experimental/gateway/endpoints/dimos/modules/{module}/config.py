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

from typing import Annotated, Any

from fastapi import FastAPI, Path as PathParam

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models
from experimental.gateway.utils.http import ApiError

ModuleParam = Annotated[
    str,
    PathParam(
        description="a module's registry name, class name or `module.Class`",
        examples=["go2-connection"],
    ),
]


def register(app: FastAPI, state: ServerState) -> None:
    discovery = state.discovery
    assert discovery is not None

    @app.get(
        "/dimos/modules/{module}/config",
        response_model=models.ModuleConfigAnswer,
        **route_doc(
            "discovery",
            "A module's config fields and defaults: name, type, default, description, enum values, and whether it converts to and from JSON",
            "Read from the module's real config class (pydantic model or dataclass) by the discovery scan; a registry module the scan hasn't reached yet is imported now, in a child process. A field that isn't `json_compatible` (a class, a callable, an array) can't be set from a form: leave it out. 404 for a module that isn't in the registry or any blueprint; 500 when it can't be imported.",
            errors=(404, 500),
            agent=True,
            answer="`{ module, class, fields: [{ name, type, default, description, required, base, enum, json_compatible, reason }], error }`",
        ),
    )
    async def module_config(module: ModuleParam) -> dict[str, Any]:
        record = discovery.find_module(module)
        if record is None:
            record = await discovery.module_now(module)
        if record is None:
            raise ApiError(404, f"no module {module} (not in dimos's registry or any blueprint)")
        if "error" in record and "class" not in record:
            raise ApiError(500, f"module {module} doesn't import: {record['error']}")
        return {
            "module": record["name"],
            "class": record["class"],
            "fields": record.get("config", []),
            "error": record.get("config_error"),
        }
