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
from experimental.gateway.utils import blueprints, config, models, overrides as launch_overrides
from experimental.gateway.utils.api_helpers import ApiHelpers
from experimental.gateway.utils.http import ApiError, BlueprintParam


def register(app: FastAPI, state: ServerState) -> None:
    helpers = ApiHelpers(state)

    @app.get(
        "/dimos/blueprints/{name}/config",
        response_model=models.BlueprintConfig,
        **route_doc(
            "blueprints",
            "A blueprint's configurable args per module: name, type, default, description, and the value the blueprint sets (a module that can't be read carries its own error)",
            "Imports the blueprint in a child process (180 s timeout; cached 10 min) and reads each module's pydantic `config` model: every field a person can set, its type, default, description, whether it's required or inherited from ModuleConfig, its choices (Enum/Literal) and the blueprint's value. A module whose config can't be read has `error` instead of failing the whole answer.",
            errors=(400, 500),
            agent=True,
            answer="`{ name, modules: [{ module, class, args: [{ name, type, default, description, required, base, choices?, value? }], error? }] }`",
        ),
    )
    async def blueprint_config(name: BlueprintParam) -> Any:
        helpers.check_name(name)
        value = await helpers.introspected(f"config:{name}", ["config", name])
        return blueprints.shown_config(name, value)

    @app.put(
        "/dimos/blueprints/{name}/config",
        response_model=models.BlueprintConfig,
        **route_doc(
            "blueprints",
            "Save Desktop's module config for a blueprint; it becomes `--<module>.<field>=value` on every launch of it",
            "Replaces config.yaml's `dimos.module_config.<name>` with `overrides` ({module: {field: value}}; null drops a field, a module left empty is dropped, all empty removes the blueprint's entry), after checking each module and field against the blueprint's config (as a launch's `overrides.modules`). A secret sent as ••• keeps its saved value. 400 for a bad name or a value its field refuses. Answers like GET.",
            errors=(400, 500),
            answer="`{ name, modules: [...], overrides }`, with the saved module config",
        ),
    )
    async def put_blueprint_config(
        name: BlueprintParam, update: models.BlueprintConfigUpdate
    ) -> Any:
        helpers.check_name(name)
        value = await helpers.introspected(f"config:{name}", ["config", name])
        saved = config.module_config(name)
        values = {
            module: launch_overrides.keep_hidden(
                fields, saved.get(module, {}), launch_overrides.is_secret_name
            )
            for (module, fields) in update.overrides.items()
        }
        try:
            launch_overrides.validate_modules(values, value, "overrides")
        except ValueError as error:
            raise ApiError(400, str(error))
        await asyncio.to_thread(
            config.set_module_config, name, launch_overrides.merge_modules({}, values)
        )
        return blueprints.shown_config(name, value)
