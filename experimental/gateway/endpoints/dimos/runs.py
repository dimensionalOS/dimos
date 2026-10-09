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

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import config, models, runs
from experimental.gateway.utils.api_helpers import ApiHelpers
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    s = state
    helpers = ApiHelpers(state)

    @app.get(
        "/dimos/runs",
        response_model=models.RunList,
        **route_doc(
            "runs",
            "Running blueprints (run id, blueprint, pid, log_dir) and the launch this gateway started",
            "Every live run on this computer, newest first, whoever started it: dimos's run registry (pid alive), plus runs in other registries (another DIMOS_HOME or XDG_STATE_HOME: a test Desktop, an agent, a terminal) and `dimos run` processes not registered yet; each says where it came from (`registry`, `owner`, `command`, `ours`) and whether POST /dimos/runs/stop can stop it (`stoppable`, `whyNot`). A registry entry whose pid now belongs to a newer process is stale and left out. `seenOnBus`: a dimos run answering on the bus that no listed run accounts for (on another machine through the zenoh connection, or a process this gateway can't see), probed in the background every 10 s; it can't be stopped from here. Also this gateway's last launch with its phase. No side effects.",
            agent=True,
            answer="`{ runs: [{ run_id, pid, blueprint, started_at, log_dir, registry, owner, command, ours, stoppable, whyNot }], launch: Launch | null, seenOnBus: [{ where, peer, note }] }`",
        ),
    )
    async def run_list() -> dict[str, Any]:
        local = await asyncio.to_thread(runs.local_runs)
        return {
            "runs": local,
            "launch": await asyncio.to_thread(runs.current_launch),
            "seenOnBus": runs.seen_on_bus(local, s.zenoh_connect),
        }

    @app.post(
        "/dimos/runs",
        response_model=models.Launch,
        **route_doc(
            "runs",
            "Launch a blueprint (stops nothing; check /dimos/runs first)",
            "Starts `dimos [--key value ...] run <blueprint>` in the checkout, in its own session, with Desktop's saved GlobalConfig overrides, then the body's, then `--replay` if asked. Answers at once with phase `starting`; `launch` events (or GET /dimos/runs) follow it to running, stopped or failed. 400 when the checkout's dimos is outside Desktop's range (unless config.yaml `dimos.ignore_version_range`), the name is bad, an override (saved or given) isn't a GlobalConfig flag or valid value, while the last launch is still starting, running or stopping (any process of it left), and while any other dimos run is on this machine (one in dimos's run registry, or a coordinator answering on the bus: two runs share module RPC names, so one's start and stop calls reach the other's modules); 500 when dimos isn't installed or won't start.",
            errors=(400, 500),
            agent=True,
            mcp_tool="run_blueprint",
            answer="`Launch`: `{ blueprint, phase, startedAt, pid, output, runId, logDir, error, overrides, steps: [{ label, state: done|now|todo|failed, detail }], problems: [{ level, text, fix, line }] }`",
        ),
    )
    async def launch(request: models.LaunchRequest) -> dict[str, Any]:
        checkout = config.info(s.dimos_dir)
        if checkout.installed and (not checkout.in_range) and (not config.ignore_version_range()):
            raise ApiError(
                400,
                f"dimos {checkout.version or '?'} is outside the range Desktop supports ({checkout.range}); set dimos.ignore_version_range to launch anyway",
            )
        if not request.blueprint or request.blueprint.startswith("-"):
            raise ApiError(400, "bad blueprint name")
        launch_config = await helpers.launch_config_of(request)
        try:
            started = await asyncio.to_thread(
                runs.start, s.dimos_dir, request.blueprint, launch_config
            )
        except runs.StillRunningError as error:
            raise ApiError(400, str(error))
        except runs.RunError as error:
            raise ApiError(500, str(error))
        s.bus.launch(started)
        return started
