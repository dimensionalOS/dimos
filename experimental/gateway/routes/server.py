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
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any

from fastapi import APIRouter
from fastapi.responses import PlainTextResponse

from experimental.gateway import logs, store
from experimental.gateway.state import ApiError, GatewayState

router = APIRouter()

PROBE = "import json, sys, dimos; print(json.dumps({'version': sys.version.split()[0], 'file': dimos.__file__}))"
EXAMPLE = "import dimos; print(dimos.__file__)"


@router.get("/healthz", response_class=PlainTextResponse, include_in_schema=False)
async def healthz() -> str:
    return "ok"


@router.get("/dimos/info")
async def info(state: GatewayState) -> dict[str, Any]:
    return store.info(state.dimos_dir)


@router.get("/dimos/paths")
async def paths(state: GatewayState) -> dict[str, Any]:
    from dimos.core.run_registry import REGISTRY_DIR

    try:
        modified: int | None = int(os.stat(sys.executable).st_mtime)
    except OSError:
        modified = None
    return {
        "dimosDir": str(state.dimos_dir),
        "runsDir": str(REGISTRY_DIR),
        "logsDirs": [str(d) for d in logs.logs_dirs(state.dimos_dir)],
        "recordingsDir": str(store.recordings_dir()),
        "server": {
            "exe": sys.executable,
            "exeModified": modified,
            "kind": "dimos",
            "startedAt": state.started_at,
            "zenohNamespace": state.bus.namespace,
        },
    }


@router.post("/dimos/server/stop")
async def stop_server(state: GatewayState) -> dict[str, Any]:
    state.uploads.shutdown()
    asyncio.get_running_loop().call_later(0.2, state.exit)
    return {"stopping": True}


def probe(python: str, env: dict[str, str]) -> dict[str, Any] | None:
    clean = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")}
    try:
        done = subprocess.run(
            [python, "-c", PROBE],
            env={**clean, **env},
            cwd="/",
            capture_output=True,
            text=True,
            timeout=30,
        )
        answer: dict[str, Any] = json.loads(done.stdout.strip().splitlines()[-1])
        return answer if done.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired, ValueError, IndexError):
        return None


def find_python(dimos_dir: Path) -> dict[str, Any]:
    root = dimos_dir.resolve()
    for python in dict.fromkeys([str(store.venv_python(dimos_dir).absolute()), sys.executable]):
        if not os.access(python, os.X_OK):
            continue
        for env in ({}, {"PYTHONPATH": str(dimos_dir.absolute())}):
            found = probe(python, env)
            if found and Path(found["file"]).resolve().is_relative_to(root):
                prefix = "".join(f"{k}={shlex.quote(v)} " for k, v in env.items())
                return {
                    "python": python,
                    "command": [python],
                    "dimosDir": str(dimos_dir.absolute()),
                    "version": found["version"],
                    "dimosVersion": store.checkout_version(dimos_dir)[1],
                    "env": env,
                    "example": f"{prefix}{shlex.quote(python)} -c {shlex.quote(EXAMPLE)}",
                }
    raise ApiError(500, f"no python imports dimos from {dimos_dir}")


@router.get("/dimos/python")
async def python(state: GatewayState) -> dict[str, Any]:
    async def compute() -> dict[str, Any]:
        return await asyncio.to_thread(find_python, state.dimos_dir)

    answer: dict[str, Any] = await state.cache.get("python", 3600.0, compute)
    return answer
