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
from typing import Annotated, Any

from fastapi import APIRouter, Path as PathParam
from pydantic import BaseModel, JsonValue

from experimental.gateway import launches, logs, overrides as ov, responses, store
from experimental.gateway.launches import LaunchConfig
from experimental.gateway.routes.blueprints import refused_args
from experimental.gateway.state import ApiError, GatewayState, State

router = APIRouter()


class LaunchRequest(BaseModel):
    blueprint: str
    replay: bool = False
    overrides: JsonValue = None
    args: list[str] | None = None


class RunRequest(BaseModel):
    """Which run (the gateway's own launches have their runId from the start)."""

    runId: str


async def is_own_launch(run_id: str) -> bool:
    launch = await asyncio.to_thread(launches.current_launch)
    return launch is not None and launch["runId"] == run_id


def with_saved(blueprint: str, one_off: ov.LaunchOverrides, args: list[str]) -> LaunchConfig:
    saved = ov.merge(ov.LAUNCH_GLOBAL_DEFAULTS, store.global_config_overrides())
    effective = ov.merge(saved, one_off.global_)
    try:
        ov.check_global(effective)
    except ValueError as error:
        raise ApiError(400, str(error))
    modules = ov.merge_modules(store.module_config(blueprint), one_off.modules)
    return LaunchConfig(effective, modules, one_off, list(args))


async def launch_config(state: State, request: LaunchRequest) -> LaunchConfig:
    try:
        one_off = ov.parse(request.overrides)
        one_off.global_ = ov.without_hidden(one_off.global_)
        one_off.modules = {m: ov.without_hidden(f) for m, f in one_off.modules.items()}
        if request.replay:
            one_off.global_["replay"] = True
        ov.check_global(one_off.global_)
    except ValueError as error:
        raise ApiError(400, str(error))
    args = list(request.args or [])
    checked = ov.module_flags(one_off.modules) + args
    refused = await refused_args(state, request.blueprint, checked) if checked else []
    if refused:
        raise ApiError(400, "dimos wouldn't take these args: " + "; ".join(refused))
    return with_saved(request.blueprint, one_off, args)


@router.get("/runs", response_model=responses.Runs)
async def run_list() -> dict[str, Any]:
    return {
        "runs": await asyncio.to_thread(launches.registry_runs),
        "launch": await asyncio.to_thread(launches.current_launch),
    }


@router.post("/runs", response_model=responses.Launch)
async def launch(state: GatewayState, request: LaunchRequest) -> dict[str, Any]:
    """Launch a blueprint with Desktop's saved config plus this launch's overrides and args."""
    if not request.blueprint or request.blueprint.startswith("-"):
        raise ApiError(400, "bad blueprint name")
    config = await launch_config(state, request)
    try:
        started = await asyncio.to_thread(
            launches.start, state.dimos_dir, request.blueprint, config
        )
    except launches.StillRunningError as error:
        raise ApiError(400, str(error))
    except launches.RunError as error:
        raise ApiError(500, str(error))
    state.bus.launch(started)
    return started


@router.post("/runs/stop", response_model=responses.Stopped)
async def stop(state: GatewayState, request: RunRequest) -> dict[str, Any]:
    """Stop the run `runId`."""
    try:
        return {"output": await launches.stop(request.runId)}
    except launches.NoSuchRunError as error:
        raise ApiError(400, str(error))
    except launches.RunError as error:
        raise ApiError(500, str(error))
    finally:
        state.bus.launch(await asyncio.to_thread(launches.current_launch))


@router.post("/runs/restart", response_model=responses.Launch)
async def restart(state: GatewayState, request: RunRequest) -> dict[str, Any]:
    """Stop the run `runId` and launch it again.

    The gateway's own launch starts again with the config saved now; a run it didn't launch starts again with the
    same command line, directory and environment.
    """
    run_id, ours = request.runId, await is_own_launch(request.runId)
    current = await asyncio.to_thread(launches.current_launch)
    active = ours and current is not None and current["phase"] in launches.ACTIVE
    # TODO: after gateway is on main, drop this fallback for runs the gateway didn't launch
    try:
        run = (
            None
            if ours and not active
            else await asyncio.to_thread(launches.run_to_relaunch, run_id)
        )
    except launches.RunError as error:
        raise ApiError(400, str(error))
    if run is not None:
        try:
            command, cwd, env = await asyncio.to_thread(launches.snapshot, int(run["pid"]))
            await launches.stop(run_id)
            started = await asyncio.to_thread(
                launches.relaunch, str(run["blueprint"]), command, cwd, env
            )
        except launches.RunError as error:
            raise ApiError(500, str(error))
        state.bus.launch(started)
        return started
    last = launches.last_launch_args()
    if last is None:
        raise ApiError(400, "the dimos gateway hasn't launched anything yet")
    blueprint, previous = last
    config = with_saved(blueprint, previous.one_off, previous.args)
    try:
        if active:
            await launches.stop(run_id)
        started = await asyncio.to_thread(launches.start, state.dimos_dir, blueprint, config)
    except launches.RunError as error:
        raise ApiError(500, str(error))
    state.bus.launch(started)
    return started


@router.get("/runs/{runId}/log", response_model=responses.RunLog)
async def run_log(
    state: GatewayState,
    run_id: Annotated[str, PathParam(alias="runId")],
    after: int | None = None,
    level: str | None = None,
    q: str | None = None,
    limit: int | None = None,
) -> dict[str, Any]:
    file = await asyncio.to_thread(logs.find, state.dimos_dir, run_id)
    return await asyncio.to_thread(
        logs.read, file, run_id, after, limit or 1000, q or None, level or None
    )
