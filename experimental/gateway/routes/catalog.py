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

from fastapi import APIRouter

from experimental.gateway import catalog, constants, introspect, launches, responses
from experimental.gateway.state import ApiError, GatewayState, State

router = APIRouter(prefix="/catalog")


def check_name(name: str) -> None:
    if not name or name.startswith("-") or any(char.isspace() for char in name):
        raise ApiError(400, f"bad name: {name}")


def picked(answer: dict[str, Any], fields: str | None) -> dict[str, Any]:
    return catalog.pick(answer, fields)


async def listed(state: State) -> list[dict[str, Any]]:
    return state.listed or (await asyncio.to_thread(introspect.blueprint_list))["blueprints"]


async def blueprint_list(state: State) -> list[dict[str, Any]]:
    entries = await listed(state)
    notes = await asyncio.to_thread(
        catalog.notes_for, state.dimos_dir, [entry["name"] for entry in entries]
    )
    return catalog.blueprints(entries, state.scanner.cache(), notes)


async def find_blueprint(state: State, name: str) -> dict[str, Any]:
    """A blueprint the gateway's checkout has, or one running from another checkout."""
    check_name(name)
    found = next(
        (candidate for candidate in await blueprint_list(state) if candidate["name"] == name), None
    )
    if found is None and await asyncio.to_thread(running_source, state, name) is not None:
        found = catalog.blueprints([{"name": name}], None, {"robots": {}})[0]
    if found is None:
        raise ApiError(404, f"no blueprint {name}")
    return found


async def has_locally(state: State, name: str) -> bool:
    return any(entry["name"] == name for entry in await listed(state))


def running_source(state: State, name: str) -> launches.RunSource | None:
    """Where the live run of this blueprint came from, when that isn't the gateway's checkout."""
    run = launches.run_of(name)
    source = launches.run_source(str(run["run_id"])) if run else None
    if source is None or source.checkout.resolve() == state.dimos_dir.resolve():
        return None
    return source


async def from_run_checkout(state: State, args: list[str], local: bool) -> dict[str, Any]:
    """Introspect from the checkout the live run came from (falling back to the gateway's when it has the blueprint)."""
    source = await asyncio.to_thread(running_source, state, args[1])
    if source is None:
        return {**await state.introspected(args), "checkout": None, "checkoutError": None}
    try:
        found = await state.introspected(args, source=source)
        return {**found, "checkout": str(source.checkout), "checkoutError": None}
    except ApiError as error:
        failed = f"reading it from {source.checkout} failed: {error}"
        if not local:
            raise ApiError(500, failed)
        return {**await state.introspected(args), "checkout": None, "checkoutError": failed}


async def scan(state: State) -> dict[str, Any]:
    return await asyncio.to_thread(state.scanner.snapshot)


@router.get("", response_model=responses.Catalog)
async def everything(
    state: GatewayState,
    module: str | None = None,
    blueprint: str | None = None,
    fields: str | None = None,
) -> dict[str, Any]:
    entries = await listed(state)
    notes = await asyncio.to_thread(
        catalog.notes_for, state.dimos_dir, [entry["name"] for entry in entries]
    )
    cache = state.scanner.cache()
    blueprints, modules = catalog.blueprints(entries, cache, notes), catalog.modules(cache)
    if blueprint is not None:
        blueprints = [entry for entry in blueprints if entry["name"] == blueprint]
        names = {name for entry in blueprints for name in entry["modules"]}
        modules = [entry for entry in modules if entry["name"] in names]
    if module is not None:
        modules = [entry for entry in modules if module in (entry["name"], entry["class"])]
        names = {entry["name"] for entry in modules}
        blueprints = [entry for entry in blueprints if names & set(entry["modules"])]
    kept = {entry["name"] for entry in modules}
    answer = {
        "blueprints": blueprints,
        "modules": modules,
        "rpcs": [rpc_record for rpc_record in catalog.rpcs(cache) if rpc_record["module"] in kept],
        "robots": catalog.robot_list(notes),
        "types": notes.get("types", {}),
        "errors": catalog.errors(cache),
        "scan": await scan(state),
    }
    return picked(answer, fields)


@router.post("/scan", status_code=202, response_model=responses.ScanStarted)
async def scan_all(
    state: GatewayState, timeout: float = constants.SCAN_ITEM_TIMEOUT_S
) -> dict[str, Any]:
    return {"job": state.scanner.start("requested", timeout).id}


@router.post("/scan/blueprints/{name}", status_code=202, response_model=responses.ScanStarted)
async def scan_blueprint(
    state: GatewayState, name: str, timeout: float = constants.SCAN_ITEM_TIMEOUT_S
) -> dict[str, Any]:
    await find_blueprint(state, name)
    return {"job": state.scanner.start_one(name, timeout).id}


@router.get("/blueprints", response_model=responses.BlueprintList)
async def blueprints(state: GatewayState, fields: str | None = None) -> dict[str, Any]:
    return picked({"blueprints": await blueprint_list(state), "scan": await scan(state)}, fields)


@router.get(
    "/blueprints/{name}", response_model=responses.BlueprintDetail, openapi_extra={"x-slow": True}
)
async def blueprint(state: GatewayState, name: str, fields: str | None = None) -> dict[str, Any]:
    found = await find_blueprint(state, name)
    wanted = {field.strip().split(".")[0] for field in fields.split(",")} if fields else None
    if wanted is None or wanted & {"file", "line", "modules", "checkout", "checkoutError"}:
        found.update(
            await from_run_checkout(state, ["blueprint", name], await has_locally(state, name))
        )
        found.pop("config", None)
    return picked(found, fields)


@router.get(
    "/blueprints/{name}/config",
    response_model=responses.BlueprintSchema,
    openapi_extra={"x-slow": True},
)
async def blueprint_config(state: GatewayState, name: str) -> Any:
    await find_blueprint(state, name)
    found = await from_run_checkout(state, ["config", name], await has_locally(state, name))
    found.pop("checkout")
    found.pop("checkoutError")
    return found


async def find_module(state: State, name: str) -> dict[str, Any]:
    check_name(name)
    found = next(
        (
            candidate
            for candidate in catalog.modules(state.scanner.cache())
            if name in (candidate["name"], candidate["class"])
        ),
        None,
    )
    if found is None:
        raise ApiError(404, f"no module {name} in the last scan")
    return found


async def importable_module(state: State, name: str) -> dict[str, Any]:
    found = await find_module(state, name)
    if "<locals>" in found["class"]:
        raise ApiError(
            404, f"{found['class']} is defined inside a function, so it can't be loaded on its own"
        )
    return found


@router.get("/config", response_model=responses.GlobalConfig)
async def global_config() -> dict[str, Any]:
    from dimos.core.global_config import GlobalConfig

    defaults = {
        key: json.loads(json.dumps(field.default, default=str))
        for key, field in GlobalConfig.model_fields.items()
        if field.default_factory is None
    }
    schema = GlobalConfig.model_json_schema()
    names = [*GlobalConfig.model_fields, *schema.get("properties", {})]
    return {
        "schema": schema,
        "defaults": defaults,
        "secrets": sorted({key for key in names if launches.is_secret_name(key)}),
    }


@router.get("/modules", response_model=responses.ModuleList)
async def modules(state: GatewayState, fields: str | None = None) -> dict[str, Any]:
    return picked(
        {"modules": catalog.modules(state.scanner.cache()), "scan": await scan(state)}, fields
    )


@router.get(
    "/modules/{name}", response_model=responses.ModuleDetail, openapi_extra={"x-slow": True}
)
async def module(state: GatewayState, name: str) -> dict[str, Any]:
    found = await importable_module(state, name)
    detail = await state.introspected(["module", found["class"]])
    return {**found, **{key: detail[key] for key in ("rpcs", "file", "line")}}


@router.get(
    "/modules/{name}/config", response_model=responses.ModuleSchema, openapi_extra={"x-slow": True}
)
async def module_config(state: GatewayState, name: str) -> Any:
    found = await importable_module(state, name)
    return await state.introspected(["module_config", found["class"]])


@router.get("/rpcs", response_model=responses.CatalogRpcs)
async def rpcs(
    state: GatewayState, namespace: str | None = None, fields: str | None = None
) -> dict[str, Any]:
    found = catalog.rpcs(state.scanner.cache(), namespace)
    return picked({"rpcs": found, "scan": await scan(state)}, fields)


@router.get("/rpcs/{module}/{name}", response_model=responses.CatalogRpc)
async def rpc(
    state: GatewayState, module: str, name: str, namespace: str | None = None
) -> dict[str, Any]:
    wanted = f"{namespace}/{module}" if namespace else module
    for found in catalog.rpcs(state.scanner.cache(), namespace):
        if found["name"] == name and found["module"] in (module, wanted):
            return found
    raise ApiError(404, f"no RPC {module}/{name} in the last scan")


@router.get("/robots", response_model=responses.Robots)
async def robots(state: GatewayState, fields: str | None = None) -> dict[str, Any]:
    entries = await listed(state)
    notes = await asyncio.to_thread(
        catalog.notes_for, state.dimos_dir, [entry["name"] for entry in entries]
    )
    return picked({"types": notes.get("types", {}), "robots": catalog.robot_list(notes)}, fields)


@router.get("/msgs/{name}", response_model=responses.MessageType)
async def message_type(name: str) -> dict[str, Any]:
    check_name(name)
    return {"name": name, "definition": None}
