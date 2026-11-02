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

from experimental.gateway import constants, logs, responses, store
from experimental.gateway.state import ApiError, GatewayState, State

router = APIRouter(prefix="/history")

RunId = Annotated[str, PathParam(alias="runId")]


async def read_log(
    state: State,
    run_id: str,
    after: int | None,
    level: str | None,
    query: str | None,
    limit: int | None,
) -> dict[str, Any]:
    file = await asyncio.to_thread(logs.find, state.dimos_dir, run_id)
    if file is None:
        raise ApiError(
            404, "no run has a log yet" if run_id == "latest" else f"no log for run {run_id}"
        )
    return await asyncio.to_thread(
        logs.read,
        file,
        run_id,
        after,
        limit or constants.DEFAULT_LOG_LIMIT,
        query or None,
        level or None,
    )


@router.get("", response_model=responses.History)
async def history(state: GatewayState) -> dict[str, Any]:
    return {"runs": await asyncio.to_thread(logs.history, state.dimos_dir)}


@router.get("/recordings", response_model=responses.Recordings)
async def recordings() -> dict[str, Any]:
    return {"path": str(store.recordings_dir())}


@router.get("/{runId}", response_model=responses.PastRunDetail)
async def past_run(state: GatewayState, run_id: RunId) -> dict[str, Any]:
    runs = await asyncio.to_thread(logs.history, state.dimos_dir)
    wanted = runs[:1] if run_id == "latest" else [run for run in runs if run["runId"] == run_id]
    found = wanted[0] if wanted else None
    file = await asyncio.to_thread(logs.find, state.dimos_dir, found["runId"]) if found else None
    if found is None or file is None:
        raise ApiError(404, f"no run {run_id}")
    return {**found, "logDir": str(file.parent)}


@router.get("/{runId}/logs", response_model=responses.RunLog)
async def past_log(
    state: GatewayState,
    run_id: RunId,
    after: int | None = None,
    level: str | None = None,
    q: str | None = None,
    limit: int | None = None,
) -> dict[str, Any]:
    return await read_log(state, run_id, after, level, q, limit)
