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
from contextlib import closing
from pathlib import Path
import sqlite3
from typing import Annotated, Any

from fastapi import APIRouter, Path as PathParam
from pydantic import BaseModel, JsonValue

from experimental.gateway import launches, logs, overrides as ov, store
from experimental.gateway.launches import LaunchConfig
from experimental.gateway.routes.blueprints import refused_args
from experimental.gateway.state import ApiError, GatewayState, State

router = APIRouter()


class LaunchRequest(BaseModel):
    blueprint: str
    replay: bool = False
    overrides: JsonValue = None
    args: list[str] | None = None


class StopRequest(BaseModel):
    runId: str | None = None


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


@router.get("/dimos/runs")
async def run_list() -> dict[str, Any]:
    return {
        "runs": await asyncio.to_thread(launches.registry_runs),
        "launch": await asyncio.to_thread(launches.current_launch),
    }


@router.post("/dimos/runs")
async def launch(state: GatewayState, request: LaunchRequest) -> dict[str, Any]:
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


@router.post("/dimos/runs/stop")
async def stop(state: GatewayState, request: StopRequest | None = None) -> dict[str, Any]:
    try:
        return {"output": await launches.stop(request.runId if request else None)}
    except launches.RunError as error:
        raise ApiError(500, str(error))
    finally:
        state.bus.launch(await asyncio.to_thread(launches.current_launch))


@router.post("/dimos/runs/restart")
async def restart(state: GatewayState) -> dict[str, Any]:
    last = launches.last_launch_args()
    if last is None:
        raise ApiError(400, "the dimos gateway hasn't launched anything yet")
    blueprint, previous = last
    config = with_saved(blueprint, previous.one_off, previous.args)
    current = await asyncio.to_thread(launches.current_launch)
    try:
        if current and current["phase"] in launches.ACTIVE:
            await launches.stop(None)
        started = await asyncio.to_thread(launches.start, state.dimos_dir, blueprint, config)
    except launches.RunError as error:
        raise ApiError(500, str(error))
    state.bus.launch(started)
    return started


@router.get("/dimos/runs/{runId}/log")
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


def replay(path: Path) -> dict[str, Any]:
    stat = path.stat()
    entry: dict[str, Any] = {
        "name": path.stem,
        "path": str(path),
        "size": stat.st_size,
        "modified": stat.st_mtime,
        "streams": [],
        "duration": None,
        "error": None,
    }
    if stat.st_size < 1024 and path.read_bytes().startswith(b"version https://git-lfs"):
        return {**entry, "error": "not downloaded yet (a git LFS pointer): run `git lfs pull`"}
    try:
        with closing(sqlite3.connect(f"file:{path}?mode=ro", uri=True)) as db:
            names = [row[0] for row in db.execute("SELECT name FROM _streams ORDER BY name")]
            clock = next((n for n in ("go2_odom", "odom") if n in names), None)
            span = (
                db.execute(f'SELECT max(ts) - min(ts) FROM "{clock}"').fetchone() if clock else None
            )
    except sqlite3.Error as error:
        return {**entry, "error": f"not a dimos recording: {error}"}
    return {**entry, "streams": names, "duration": span[0] if span else None}


@router.get("/dimos/replays")
async def replays(state: GatewayState) -> dict[str, Any]:
    folder = state.dimos_dir / "data"
    samples = (
        sorted(p for p in folder.glob("*.db") if not p.name.startswith("."))
        if folder.is_dir()
        else []
    )
    return {"replays": await asyncio.to_thread(lambda: [replay(p) for p in samples])}
