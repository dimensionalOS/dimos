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
from experimental.gateway.utils.api_helpers import ApiHelpers
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    s = state
    helpers = ApiHelpers(state)

    @app.post(
        "/dimos/runs/restart",
        response_model=models.Launch,
        **route_doc(
            "runs",
            "Stop the blueprint this gateway launched (if it still runs) and launch it again: its own values on top of the config saved now",
            "Takes the last launch's blueprint and its own (one-off) overrides (kept even after it stopped), stops it first if it's starting, running or stopping (as POST /dimos/runs/stop: launched again only once no process of the old run is left and every port it listened on is free), then launches it as POST /dimos/runs would, with Desktop's saved global and module config as saved now (a config change since applies). Takes no body. 400 when nothing was launched yet; 500 when it won't stop or won't start.",
            errors=(400, 500),
            agent=True,
            answer="`Launch` (as POST /dimos/runs)",
        ),
    )
    async def restart() -> dict[str, Any]:
        last = runs.last_launch_args()
        if last is None:
            raise ApiError(400, "the dimos gateway hasn't launched anything yet")
        (blueprint, last_config) = last
        launch_config = helpers.with_saved(blueprint, last_config.one_off, last_config.args)
        current = await asyncio.to_thread(runs.current_launch)
        try:
            if current and current["phase"] in ("starting", "running", "stopping"):
                await runs.stop(None, lambda: s.bus.launch(runs.current_launch()))
            started = await asyncio.to_thread(runs.start, s.dimos_dir, blueprint, launch_config)
        except runs.RunError as error:
            raise ApiError(500, str(error))
        s.bus.launch(started)
        return started
