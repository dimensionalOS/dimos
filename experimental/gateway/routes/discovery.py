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
from pydantic import BaseModel, Field

from experimental.gateway import discovery, store
from experimental.gateway.state import ApiError, GatewayState, State

router = APIRouter()


class RefreshRequest(BaseModel):
    full: bool = False


class ExtrasInstall(BaseModel):
    extras: list[str] = Field(min_length=1)
    app: str | None = None


async def probe(state: State) -> dict[str, Any]:
    answer: dict[str, Any] = await state.introspected(["packages"])
    return answer


@router.get("/dimos/discovery")
async def status(state: GatewayState) -> dict[str, Any]:
    state.scanner.ensure("startup")
    return state.scanner.snapshot()


@router.post("/dimos/discovery/refresh")
async def refresh(state: GatewayState, request: RefreshRequest | None = None) -> dict[str, Any]:
    state.scanner.refresh("requested")
    return state.scanner.snapshot()


@router.get("/dimos/discovery/blueprints")
async def discovered(state: GatewayState) -> dict[str, Any]:
    state.scanner.ensure("startup")
    return await asyncio.to_thread(state.scanner.discovered)


@router.get("/dimos/extras")
async def extras(state: GatewayState) -> dict[str, Any]:
    found = await probe(state)
    listed = await asyncio.to_thread(discovery.extras_status, state.dimos_dir, found)
    mode = "checkout" if discovery.is_checkout(state.dimos_dir) else "library"
    return {"mode": mode, "python": found["python"], "extras": listed}


@router.post("/dimos/extras/install")
async def install_extras(state: GatewayState, request: ExtrasInstall) -> dict[str, Any]:
    declared = discovery.declared(state.dimos_dir, await probe(state))
    unknown = [name for name in request.extras if name not in declared]
    if unknown:
        raise ApiError(
            400, f"unknown extra(s): {', '.join(unknown)} (known: {', '.join(declared)})"
        )
    running = state.jobs.running("extras")
    if running is not None:
        raise ApiError(409, f"an extras install is already running: job {running.id}")
    uv = discovery.find_uv()
    if uv is None:
        raise ApiError(500, "uv isn't installed (https://docs.astral.sh/uv/)")
    wanted = list(dict.fromkeys(request.extras))
    command = discovery.install_command(state.dimos_dir, wanted, uv)
    env = (
        {"VIRTUAL_ENV": str(store.venv_dir(state.dimos_dir))}
        if discovery.is_checkout(state.dimos_dir)
        else {}
    )

    def then() -> None:
        state.cache.forget("packages")
        state.scanner.refresh("extras installed")

    job = state.jobs.start(
        f"Install extras: {', '.join(wanted)}", "extras", command, state.dimos_dir, env, then
    )
    return {"shell": None, "job": job.id, "command": command}


@router.get("/dimos/jobs/{job}/log")
async def job_log(state: GatewayState, job: str, after: int = 0) -> dict[str, Any]:
    try:
        return state.jobs.get(job).log(after)
    except KeyError as error:
        raise ApiError(404, error.args[0])


@router.get("/dimos/docs/custom-robot")
async def custom_robot(state: GatewayState) -> dict[str, Any]:
    answer = await asyncio.to_thread(discovery.custom_robot, state.dimos_dir)
    if answer is None:
        raise ApiError(404, f"no guide to adding a robot in {state.dimos_dir / 'docs'}")
    return answer
