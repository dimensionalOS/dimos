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
import re
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import config, models, overrides as launch_overrides
from experimental.gateway.utils.api_helpers import ApiHelpers
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    helpers = ApiHelpers(state)

    @app.get(
        "/dimos/global-config",
        response_model=models.GlobalConfig,
        **route_doc(
            "global-config",
            "dimos GlobalConfig: JSON schema, defaults, Desktop's overrides",
            "GlobalConfig's JSON Schema and defaults (cached 10 min) and Desktop's saved overrides from config.yaml `dimos.global_config`. No side effects.",
            agent=True,
            answer="`{ schema, defaults, overrides }`",
        ),
    )
    async def global_config() -> dict[str, Any]:
        return await helpers.global_config_value()

    @app.put(
        "/dimos/global-config",
        response_model=models.GlobalConfig,
        **route_doc(
            "global-config",
            "Save Desktop's GlobalConfig overrides (null removes one); they become `--key=value` on every launch",
            "Replaces config.yaml's `dimos.global_config` with `overrides` (null values dropped), keeping the rest of the file. Takes effect at the next launch; a running blueprint is untouched. 400 for a key that isn't a GlobalConfig field `dimos` takes as a flag, or a value GlobalConfig refuses. Answers like GET.",
            errors=(400, 500),
            answer="`{ schema, defaults, overrides }`, with the saved overrides",
        ),
    )
    async def put_global_config(update: models.GlobalConfigUpdate) -> dict[str, Any]:
        for key in update.overrides:
            if not re.fullmatch("[A-Za-z0-9_]+", key):
                raise ApiError(400, f"bad config key: {key}")
        values = launch_overrides.keep_hidden(
            update.overrides, config.global_config_overrides(), launch_overrides.is_secret_name
        )
        schema = (await helpers.global_config_value())["schema"]
        try:
            launch_overrides.validate_global(values, schema, "overrides")
        except ValueError as error:
            raise ApiError(400, str(error))
        helpers.checked_overrides(values)
        await asyncio.to_thread(config.set_global_config_overrides, values)
        return await helpers.global_config_value()
