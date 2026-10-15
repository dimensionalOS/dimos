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
from fastapi.responses import RedirectResponse
from pydantic import BaseModel, Field

from experimental.gateway import discovery, responses, store
from experimental.gateway.state import ApiError, GatewayState, State

router = APIRouter()


class ExtrasInstall(BaseModel):
    extras: list[str] = Field(min_length=1)
    app: str | None = None


@router.get("/extras", response_model=responses.Extras)
async def extras(state: GatewayState) -> dict[str, Any]:
    found = await state.introspected(["packages"])
    listed = await asyncio.to_thread(discovery.extras_status, state.dimos_dir, found)
    mode = "checkout" if discovery.is_checkout(state.dimos_dir) else "library"
    return {"mode": mode, "python": found["python"], "extras": listed}


@router.post("/extras/install", status_code=202, response_model=responses.JobStarted)
async def install_extras(state: GatewayState, request: ExtrasInstall) -> dict[str, Any]:
    """Install dimos extras with uv, as a job."""
    probed = await state.introspected(["packages"])
    declared = discovery.declared(state.dimos_dir, probed)
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

    builds = await asyncio.to_thread(discovery.builds_cyclonedds, state.dimos_dir, probed, wanted)
    prepare = discovery.prepare_cyclonedds(state.dimos_dir) if builds else None
    title = f"Install extras: {', '.join(wanted)}"
    job = state.jobs.start(title, "extras", command, state.dimos_dir, env, then, prepare)
    return {"shell": None, "job": job.id, "command": command}


def found(state: State, job: str) -> discovery.Job:
    answer = state.jobs.get(job)
    if answer is None:
        raise ApiError(404, f"no job {job}")
    return answer


@router.get("/jobs/{job}", response_model=responses.JobLog)
async def job(state: GatewayState, job: str, after: int = 0) -> dict[str, Any]:
    """A job (an extras install, a blueprint scan): whether it finished, its progress, and its output from line `after` on."""
    return found(state, job).log(after)


@router.get("/docs/search", response_class=RedirectResponse, status_code=307)
async def docs_search(state: GatewayState, q: str = "") -> str:
    """Redirects to dimos's docs site with its search open on ?q= (open it in a new tab, or an iframe)."""
    url = await asyncio.to_thread(discovery.docs_search_url, state.dimos_dir, q)
    if url is None:
        raise ApiError(404, f"no docs site in {state.dimos_dir / 'mkdocs.yml'} (site_url)")
    return url
