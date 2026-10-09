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
from experimental.gateway.utils import models, runs
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.post(
        "/dimos/runs/stop",
        response_model=models.StopResult,
        **route_doc(
            "runs",
            "Stop the blueprint this gateway launched (or runId)",
            "Sends the run's process group SIGINT, then SIGTERM after 20 s, then SIGKILL after 10 more, and answers once no process of it is left (its whole process group: workers, MuJoCo, rerun) and every port it listened on is free again (a leftover of the run's own still holding one gets SIGTERM, then SIGKILL). Stops `runId` (any live run on this computer that GET /dimos/runs lists as `stoppable`, whoever started it) or else this gateway's launch; the body is optional. 500 when there's nothing running to stop, it isn't this user's to stop, it won't stop, or a port it held is still taken.",
            errors=(400, 500),
            agent=True,
            mcp_tool="stop_blueprint",
            answer="`{ output }`",
        ),
    )
    async def stop(request: models.StopRequest | None = None) -> dict[str, Any]:
        try:
            return {
                "output": await runs.stop(
                    request.runId if request else None, lambda: s.bus.launch(runs.current_launch())
                )
            }
        except runs.RunError as error:
            raise ApiError(500, str(error))
        finally:
            s.bus.launch(await asyncio.to_thread(runs.current_launch))
