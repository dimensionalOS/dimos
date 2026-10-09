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
import json
from typing import Any

from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import (
    blueprints,
    config,
    models,
    overrides as launch_overrides,
    runs,
)
from experimental.gateway.utils.http import INTROSPECT_TTL_S, LIST_TTL_S, ApiError, FreshQuery


class ApiHelpers:
    def __init__(self, state: ServerState) -> None:
        self.state = state

    async def introspected(self, key: str, args: list[str]) -> Any:
        s = self.state
        try:
            return await s.cache.get(
                key, INTROSPECT_TTL_S, lambda: blueprints.introspect(s.dimos_dir, args)
            )
        except blueprints.IntrospectError as error:
            raise ApiError(500, str(error))

    def checked_overrides(self, overrides: dict[str, Any]) -> None:
        try:
            config.check_overrides(overrides)
        except ValueError as error:
            raise ApiError(400, str(error))

    def check_name(self, name: str) -> None:
        if not blueprints.valid_name(name):
            raise ApiError(400, f"bad blueprint name: {name}")

    async def checked_args(self, name: str, args: list[str]) -> dict[str, Any]:
        text = json.dumps(args)
        checked: dict[str, Any] = await self.introspected(
            f"args:{name}:{text}", ["args", name, text]
        )
        return checked

    async def global_config_value(self) -> dict[str, Any]:
        s = self.state

        async def compute() -> dict[str, Any]:
            return await asyncio.to_thread(blueprints.global_config_schema)

        value: dict[str, Any] = await s.cache.get("gc", INTROSPECT_TTL_S, compute)
        secrets = [
            key
            for key in value["schema"].get("properties", {})
            if launch_overrides.is_secret_name(key)
        ]
        (shown, _) = launch_overrides.redact(config.global_config_overrides(), {}, secrets)
        defaults = {**value["defaults"], **config.LAUNCH_GLOBAL_DEFAULTS}
        return {**value, "defaults": defaults, "overrides": shown, "secrets": secrets}

    async def launch_config_of(self, request: models.LaunchRequest) -> runs.LaunchConfig:
        """Desktop's saved global config and this blueprint's saved module config, the request's own values on top
        (checked against dimos's schemas first; null drops a saved value, ••• keeps it), plus `replay`."""
        try:
            one_off = launch_overrides.parse(request.overrides)
        except ValueError as error:
            raise ApiError(400, str(error))
        one_off.global_ = {
            k: v for (k, v) in one_off.global_.items() if v != launch_overrides.HIDDEN
        }
        one_off.modules = {
            m: {k: v for (k, v) in f.items() if v != launch_overrides.HIDDEN}
            for (m, f) in one_off.modules.items()
        }
        if request.replay:
            one_off.global_["replay"] = True
        try:
            if one_off.global_:
                schema = (await self.global_config_value())["schema"]
                launch_overrides.validate_global(
                    one_off.global_, schema, "overrides.global", one_off.secrets
                )
            if one_off.modules:
                value = await self.introspected(
                    f"config:{request.blueprint}", ["config", request.blueprint]
                )
                launch_overrides.validate_modules(
                    one_off.modules, value, "overrides.modules", one_off.secrets
                )
        except ValueError as error:
            raise ApiError(400, str(error))
        args = list(request.args or [])
        if args:
            checked = await self.checked_args(request.blueprint, args)
            refused = [
                f"{' '.join(arg['tokens'])}: {arg['error']}"
                for arg in checked["args"]
                if arg["error"]
            ]
            if refused:
                raise ApiError(400, "dimos wouldn't take these args: " + "; ".join(refused))
        return self.with_saved(request.blueprint, one_off, args)

    def with_saved(
        self,
        blueprint: str,
        one_off: launch_overrides.LaunchOverrides,
        args: list[str] | None = None,
    ) -> runs.LaunchConfig:
        """A launch's own values on top of Desktop's saved global config and the blueprint's saved module config, as
        saved now."""
        effective = launch_overrides.merge(
            launch_overrides.merge(config.LAUNCH_GLOBAL_DEFAULTS, config.global_config_overrides()),
            one_off.global_,
        )
        self.checked_overrides(effective)
        return runs.LaunchConfig(
            effective,
            launch_overrides.merge_modules(config.module_config(blueprint), one_off.modules),
            one_off,
            list(args or []),
        )

    async def blueprint_list(self, fresh: FreshQuery = False) -> dict[str, Any]:
        s = self.state
        assert s.discovery is not None
        discovered = s.discovery
        if fresh:
            s.cache.forget("list")
            if s.watch and s.watch.listed is not None:
                await s.watch.relist()

        async def compute() -> dict[str, Any]:
            listed = s.watch.listed if s.watch else None
            return {"blueprints": listed or await asyncio.to_thread(blueprints.blueprint_list)}

        result: dict[str, Any] = await s.cache.get("list", LIST_TTL_S, compute)
        return {"blueprints": [discovered.import_status(entry) for entry in result["blueprints"]]}
