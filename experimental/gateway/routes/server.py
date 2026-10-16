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
import os
import sys
from typing import Any

from fastapi import APIRouter
from fastapi.responses import PlainTextResponse

from experimental.gateway import logs, responses, store
from experimental.gateway.state import GatewayState

router = APIRouter()


@router.get("/healthz", response_class=PlainTextResponse, include_in_schema=False)
async def healthz() -> str:
    return "ok"


@router.get("/info", response_model=responses.Info)
async def info(state: GatewayState) -> dict[str, Any]:
    from dimos.core.run_registry import REGISTRY_DIR

    try:
        modified: int | None = int(os.stat(sys.executable).st_mtime)
    except OSError:
        modified = None
    paths = {
        "dimosDir": str(state.dimos_dir),
        "runsDir": str(REGISTRY_DIR),
        "logsDirs": [str(d) for d in logs.logs_dirs(state.dimos_dir)],
        "recordingsDir": str(store.recordings_dir()),
    }
    server = {
        "exe": sys.executable,
        "exeModified": modified,
        "kind": "dimos",
        "startedAt": state.started_at,
        "zenohNamespace": state.bus.namespace,
    }
    return {
        **store.info(state.dimos_dir),
        "python": store.python_for(state.dimos_dir),
        "paths": paths,
        "server": server,
    }


@router.post("/server/stop", response_model=responses.Stopping)
async def stop_server(state: GatewayState) -> dict[str, Any]:
    """Stop this gateway (a running blueprint keeps running)."""
    state.uploads.shutdown()
    asyncio.get_running_loop().call_later(0.2, state.exit)
    return {"stopping": True}
