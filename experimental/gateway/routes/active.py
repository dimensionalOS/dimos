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

from fastapi import APIRouter
from pydantic import BaseModel, JsonValue

from experimental.gateway import launches, responses
from experimental.gateway.launches import LaunchConfig
from experimental.gateway.live import NothingRunningError
from experimental.gateway.routes.history import read_log
from experimental.gateway.state import ApiError, GatewayState, State

router = APIRouter(prefix="/active")


class LaunchRequest(BaseModel):
    """A blueprint or a module to launch, with all of its `config`: `{global, modules, secrets?}`."""

    blueprint: str | None = None
    module: str | None = None
    config: JsonValue = None


class ExecRequest(BaseModel):
    args: list[str]


class CallRequest(BaseModel):
    args: list[JsonValue] = []
    kwargs: dict[str, JsonValue] = {}


async def started(state: State, name: str, config: LaunchConfig) -> dict[str, Any]:
    launch = await asyncio.to_thread(launches.start, state.dimos_dir, name, config)
    state.bus.launch(launch)
    return launch


async def running_id() -> str:
    run_id = (await asyncio.to_thread(launches.active))["runId"]
    if run_id is None:
        raise NothingRunningError()
    return str(run_id)


@router.get("", response_model=responses.Active)
async def active() -> dict[str, Any]:
    return await asyncio.to_thread(launches.active)


@router.get("/id", response_model=responses.ActiveId)
async def active_id() -> dict[str, Any]:
    return {"runId": await running_id()}


@router.get("/log", response_model=responses.RunLog)
async def active_log(
    state: GatewayState,
    after: int | None = None,
    level: str | None = None,
    q: str | None = None,
    limit: int | None = None,
) -> dict[str, Any]:
    return await read_log(state, await running_id(), after, level, q, limit)


@router.post("/launch", response_model=responses.Launch)
async def launch(state: GatewayState, request: LaunchRequest) -> dict[str, Any]:
    if (request.blueprint is None) == (request.module is None):
        raise ApiError(400, "give exactly one of blueprint or module")
    name = str(request.blueprint or request.module)
    if not name or name.startswith("-"):
        raise ApiError(400, f"bad name: {name}")
    launch_config = launches.parse(request.config)
    launches.check_global(launch_config.global_)
    return await started(state, name, launch_config)


@router.post("/exec", response_model=responses.Launch)
async def exec_run(state: GatewayState, request: ExecRequest) -> dict[str, Any]:
    if not request.args or request.args[0].startswith("-"):
        raise ApiError(400, "args must start with a blueprint or module name")
    name, *rest = request.args
    return await started(state, name, LaunchConfig(args=rest, exact=True))


@router.post("/relaunch", response_model=responses.Launch)
async def relaunch(state: GatewayState) -> dict[str, Any]:
    launch = await asyncio.to_thread(launches.relaunch, state.dimos_dir)
    state.bus.launch(launch)
    return launch


@router.post("/stop", response_model=responses.Stopped)
async def stop(state: GatewayState) -> dict[str, Any]:
    try:
        return {"output": await asyncio.to_thread(launches.stop)}
    finally:
        state.bus.launch(await asyncio.to_thread(launches.current_launch))


@router.get("/blueprint", response_model=responses.ActiveBlueprint)
async def blueprint(state: GatewayState) -> dict[str, Any]:
    now = await asyncio.to_thread(launches.active)
    if not now["running"]:
        raise NothingRunningError()
    launch = now["launch"]
    ours = launch is not None and launch["runId"] == now["runId"]
    modules = await asyncio.to_thread(state.live.modules, None)
    return {
        "name": now["blueprint"],
        "config": launch["config"] if ours else {"global": {}, "modules": {}},
        "modules": [live_module["id"] for live_module in modules],
    }


@router.get("/modules", response_model=responses.Instances)
async def modules(state: GatewayState) -> dict[str, Any]:
    return {"modules": await asyncio.to_thread(state.live.modules, state.scanner.cache())}


@router.get("/modules/{id}", response_model=responses.Instance)
async def module(state: GatewayState, id: str) -> dict[str, Any]:
    return await asyncio.to_thread(state.live.module, state.scanner.cache(), id)


@router.post("/modules/{id}/restart", response_model=responses.ModuleAction)
async def restart_module(state: GatewayState, id: str) -> dict[str, Any]:
    return await asyncio.to_thread(state.live.restart, id)


@router.post("/modules/{id}/stop", response_model=responses.ModuleAction)
async def stop_module(state: GatewayState, id: str) -> dict[str, Any]:
    return await asyncio.to_thread(state.live.unload, id)


@router.get("/modules/{id}/rpcs", response_model=responses.LiveRpcs)
async def module_rpcs(state: GatewayState, id: str) -> dict[str, Any]:
    found = await asyncio.to_thread(state.live.module, state.scanner.cache(), id)
    return {"rpcs": found["rpcs"]}


@router.post("/modules/{id}/rpcs/{name}", response_model=responses.CallResult)
async def call(
    state: GatewayState, id: str, name: str, request: CallRequest | None = None
) -> dict[str, Any]:
    request = request or CallRequest()
    return await asyncio.to_thread(
        state.live.call, state.scanner.cache(), id, name, request.args, request.kwargs
    )


@router.get("/modules/{id}/streams", response_model=responses.LiveStreams)
async def module_streams(state: GatewayState, id: str) -> dict[str, Any]:
    found = await asyncio.to_thread(state.live.module, None, id)
    return {"streams": found["streams"]}


@router.get("/skills", response_model=responses.Skills)
async def skills(state: GatewayState) -> dict[str, Any]:
    found = await asyncio.to_thread(state.live.modules, state.scanner.cache())
    return {"skills": [rpc for live_module in found for rpc in live_module["rpcs"] if rpc["skill"]]}


@router.get("/streams", response_model=responses.LiveStreams)
async def streams(state: GatewayState) -> dict[str, Any]:
    return {"streams": await asyncio.to_thread(state.live.streams)}


# no timeout given: Dimos.peek_stream's own default
@router.get("/streams/{name}/peek", response_model=responses.Peeked)
async def peek(state: GatewayState, name: str, timeout: float | None = None) -> dict[str, Any]:
    return await asyncio.to_thread(state.live.peek, name, timeout)


@router.get("/stats", response_model=responses.ProcessStats)
async def process_stats(state: GatewayState) -> dict[str, Any]:
    return await asyncio.to_thread(state.process_stats.snapshot)
