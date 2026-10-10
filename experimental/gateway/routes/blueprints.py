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
import re
from typing import Any

from fastapi import APIRouter
from pydantic import BaseModel, JsonValue

from experimental.gateway import introspect, overrides as ov, robots, store
from experimental.gateway.state import ApiError, FreshQuery, GatewayState, State

router = APIRouter()


class ConfigUpdate(BaseModel):
    overrides: dict[str, JsonValue]


class ModuleConfigUpdate(BaseModel):
    overrides: dict[str, dict[str, JsonValue]]


class ArgsCheckRequest(BaseModel):
    args: list[str]


def check_name(name: str) -> None:
    if not name or name.startswith("-") or any(c.isspace() for c in name):
        raise ApiError(400, f"bad blueprint name: {name}")


async def blueprint_list(state: State, fresh: bool = False) -> dict[str, Any]:
    if fresh:
        await state.relist()
    listed = state.listed or (await asyncio.to_thread(introspect.blueprint_list))["blueprints"]
    return {"blueprints": [state.scanner.import_status(entry) for entry in listed]}


async def global_config() -> dict[str, Any]:
    from dimos.core.global_config import GlobalConfig

    defaults = {}
    for key, field in GlobalConfig.model_fields.items():
        if field.default_factory is None:
            defaults[key] = json.loads(json.dumps(field.default, default=str))
    schema = GlobalConfig.model_json_schema()
    names = [*GlobalConfig.model_fields, *schema.get("properties", {})]
    secrets = sorted({key for key in names if ov.is_secret_name(key)})
    shown, _ = ov.redact(store.global_config_overrides(), {}, secrets)
    return {
        "schema": schema,
        "defaults": {**defaults, **ov.LAUNCH_GLOBAL_DEFAULTS},
        "overrides": shown,
        "secrets": secrets,
    }


def shown_config(name: str, value: dict[str, Any]) -> dict[str, Any]:
    saved = store.module_config(name)
    _, hidden = ov.redact({}, saved, ov.secret_paths({}, saved))
    modules = []
    for module in value.get("modules", []):
        args = []
        for arg in module.get("args", []):
            secret = ov.is_secret_name(arg["name"])
            arg = {**arg, "secret": secret}
            if secret and arg.get("value") is not None:
                arg["value"] = ov.HIDDEN
            args.append(arg)
        modules.append({**module, "args": args})
    return {**value, "modules": modules, "overrides": hidden}


async def refused_args(state: State, name: str, args: list[str]) -> list[str]:
    checked = await state.introspected(["args", name], args)
    return [f"{' '.join(a['tokens'])}: {a['error']}" for a in checked["args"] if a["error"]]


@router.get("/dimos/blueprints")
async def list_blueprints(state: GatewayState, fresh: FreshQuery = False) -> dict[str, Any]:
    return await blueprint_list(state, fresh)


@router.get("/dimos/blueprints/{name}")
async def blueprint(state: GatewayState, name: str) -> Any:
    check_name(name)
    return await state.introspected(["blueprint", name])


@router.get("/dimos/blueprints/{name}/config")
async def blueprint_config(state: GatewayState, name: str) -> dict[str, Any]:
    check_name(name)
    return shown_config(name, await state.introspected(["config", name]))


@router.put("/dimos/blueprints/{name}/config")
async def put_blueprint_config(
    state: GatewayState, name: str, update: ModuleConfigUpdate
) -> dict[str, Any]:
    check_name(name)
    value = await state.introspected(["config", name])
    saved = store.module_config(name)
    values = {m: ov.keep_hidden(dict(f), saved.get(m, {})) for m, f in update.overrides.items()}
    refused = await refused_args(state, name, ov.module_flags(values))
    if refused:
        raise ApiError(400, "; ".join(refused))
    await asyncio.to_thread(store.set_module_config, name, ov.merge_modules({}, values))
    return shown_config(name, value)


@router.post("/dimos/blueprints/{name}/args")
async def check_blueprint_args(state: GatewayState, name: str, request: ArgsCheckRequest) -> Any:
    check_name(name)
    return await state.introspected(["args", name], request.args)


@router.get("/dimos/catalog")
async def catalog(state: GatewayState) -> dict[str, Any]:
    await state.scanner.wait()
    return state.scanner.catalog()


@router.get("/dimos/robots")
async def robot_list(state: GatewayState) -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints

    names = [entry["name"] for entry in state.listed or []] or list(all_blueprints)
    checkout = state.dimos_dir / "experimental" / "gateway" / robots.SOURCE_FILE.name
    source = checkout if checkout.exists() else robots.SOURCE_FILE
    return await asyncio.to_thread(lambda: robots.resolved(robots.load(source), names))


@router.get("/dimos/global-config")
async def get_global_config() -> dict[str, Any]:
    return await global_config()


@router.put("/dimos/global-config")
async def put_global_config(update: ConfigUpdate) -> dict[str, Any]:
    for key in update.overrides:
        if not re.fullmatch("[A-Za-z0-9_]+", key):
            raise ApiError(400, f"bad config key: {key}")
    values = ov.keep_hidden(dict(update.overrides), store.global_config_overrides())
    try:
        ov.check_global(values)
    except ValueError as error:
        raise ApiError(400, str(error))
    await asyncio.to_thread(store.set_global_config_overrides, values)
    return await global_config()
