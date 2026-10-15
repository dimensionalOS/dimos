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

from experimental.gateway import introspect, overrides as ov, responses, robots, store
from experimental.gateway.state import ApiError, FreshQuery, GatewayState, State

router = APIRouter()
# imports dimos in a child process: seconds, or minutes for the first /catalog
SLOW = {"x-slow": True}


class ConfigUpdate(BaseModel):
    overrides: dict[str, JsonValue]


class ModuleConfigUpdate(BaseModel):
    overrides: dict[str, dict[str, JsonValue]]


def check_name(name: str) -> None:
    if not name or name.startswith("-") or any(c.isspace() for c in name):
        raise ApiError(400, f"bad blueprint name: {name}")


async def blueprint_list(
    state: State, fresh: bool = False, fields: list[str] | None = None
) -> dict[str, Any]:
    if fresh:
        await state.relist()
    listed = state.listed or (await asyncio.to_thread(introspect.blueprint_list))["blueprints"]
    scanned = [state.scanner.scanned(entry) for entry in listed]
    if fields:
        kept = {"name", *fields}
        scanned = [{k: v for k, v in entry.items() if k in kept} for entry in scanned]
    return {"blueprints": scanned, "scan": await asyncio.to_thread(state.scanner.snapshot)}


def field_list(fields: str | None) -> list[str] | None:
    return [f.strip() for f in fields.split(",") if f.strip()] if fields else None


async def global_config() -> dict[str, Any]:
    from dimos.core.global_config import GlobalConfig

    defaults = {
        key: json.loads(json.dumps(field.default, default=str))
        for key, field in GlobalConfig.model_fields.items()
        if field.default_factory is None
    }
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
    modules = [
        {**module, "args": [shown_arg(arg) for arg in module["args"]]}
        for module in value["modules"]
    ]
    return {**value, "modules": modules, "overrides": hidden}


def shown_arg(arg: dict[str, Any]) -> dict[str, Any]:
    secret = ov.is_secret_name(arg["name"])
    shown = {**arg, "secret": secret}
    if secret and arg.get("value") is not None:
        shown["value"] = ov.HIDDEN
    return shown


async def refused_args(state: State, name: str, args: list[str]) -> list[str]:
    checked = await state.introspected(["args", name], args)
    return [f"{' '.join(a['tokens'])}: {a['error']}" for a in checked["args"] if a["error"]]


@router.get("/blueprints", response_model=responses.BlueprintList)
async def list_blueprints(
    state: GatewayState, fresh: FreshQuery = False, fields: str | None = None
) -> dict[str, Any]:
    """Every blueprint and what the last scan found; ?fields=kind,importable keeps only those (and name)."""
    return await blueprint_list(state, fresh, field_list(fields))


class ArgsCheckRequest(BaseModel):
    args: list[str]


@router.post("/blueprints/{name}/args", response_model=responses.CheckedArgs, openapi_extra=SLOW)
async def check_blueprint_args(state: GatewayState, name: str, request: ArgsCheckRequest) -> Any:
    """Read CLI args as `dimos run <name> <args>` would, without launching: what each sets, and why it'd refuse."""
    check_name(name)
    return await state.introspected(["args", name], request.args)


@router.get("/blueprints/{name}", response_model=responses.Blueprint, openapi_extra=SLOW)
async def blueprint(state: GatewayState, name: str) -> Any:
    """A blueprint's modules (streams, RPCs, skills), its file and line, and its configurable args with the saved values."""
    check_name(name)
    found, config = await asyncio.gather(
        state.introspected(["blueprint", name]), state.introspected(["config", name])
    )
    return {**found, "config": shown_config(name, config)}


@router.put("/blueprints/{name}/config", response_model=responses.BlueprintConfig)
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


@router.get("/catalog", response_model=responses.Catalog)
async def catalog(
    state: GatewayState, module: str | None = None, blueprint: str | None = None
) -> dict[str, Any]:
    """Every blueprint, module and skill, from the last scan (POST /catalog/scan), with how stale it is.

    ?module= (a name or class) keeps that module, its skills and the blueprints using it; ?blueprint= keeps that
    blueprint, its modules and their skills.
    """
    found = state.scanner.catalog()
    if blueprint is not None:
        found["blueprints"] = [b for b in found["blueprints"] if b["name"] == blueprint]
        names = {name for b in found["blueprints"] for name in b["modules"]}
        found["modules"] = [m for m in found["modules"] if m["name"] in names]
    if module is not None:
        found["modules"] = [m for m in found["modules"] if module in (m["name"], m["class"])]
        names = {m["name"] for m in found["modules"]}
        found["blueprints"] = [b for b in found["blueprints"] if names & set(b["modules"])]
    kept = {m["name"] for m in found["modules"]}
    found["skills"] = [s for s in found["skills"] if s["module"] in kept]
    return {**found, "scan": await asyncio.to_thread(state.scanner.snapshot)}


@router.post("/catalog/scan", status_code=202, response_model=responses.ScanStarted)
async def scan(state: GatewayState) -> dict[str, Any]:
    """Scan every blueprint and module again (or join the running scan); the cache changes when it finishes. Follow it at /jobs/{job}."""
    return {"job": state.scanner.start("requested").id}


@router.get("/robots", response_model=responses.Robots)
async def robot_list(state: GatewayState) -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints

    names = [entry["name"] for entry in state.listed or []] or list(all_blueprints)
    checkout = state.dimos_dir / "experimental" / "gateway" / robots.SOURCE_FILE.name
    source = checkout if checkout.exists() else robots.SOURCE_FILE
    return await asyncio.to_thread(lambda: robots.resolved(robots.load(source), names))


@router.get("/global-config", response_model=responses.GlobalConfig)
async def get_global_config() -> dict[str, Any]:
    return await global_config()


@router.put("/global-config", response_model=responses.GlobalConfig)
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
