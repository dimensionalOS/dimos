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
import re
from typing import Annotated

from fastapi import FastAPI, Query
from fastapi.responses import HTMLResponse

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.api_helpers import ApiHelpers
from experimental.gateway.utils.http import VIEW_DIR, ApiError


def register(app: FastAPI, state: ServerState) -> None:
    helpers = ApiHelpers(state)

    @app.get(
        "/dimos/blueprint_view",
        response_class=HTMLResponse,
        **route_doc(
            "blueprints",
            "A page showing a blueprint: its modules (rarest first) beside its module graph, each module's streams, skills, RPC methods and code",
            'An HTML page (plain JS and CSS, its files at /dimos/blueprint_view/{file}): dimOS Desktop\'s whole blueprint Details modal. A top bar (the blueprint and its phase from GET /dimos/runs; Relaunch: POST /dimos/runs/restart; Stop: POST /dimos/runs/stop; Configure: GET/PUT /dimos/blueprints/{name}/config and /dimos/global-config, recommended settings from GET /dimos/robots; Show code; Logs: GET /dimos/runs/{runId}/log), a side panel (Topic rates from GET /dimos/topics/rates; the modules, rarest first by GET /dimos/catalog) and the module graph drawn from the blueprint\'s wiring, live rates on its topics while it runs. It styles itself with Desktop\'s /theme.css and skin (Portal off Desktop). In an iframe it posts to its parent, on its own origin: {type:"dimos:chrome"} (it can draw the top bar; a parent that then shows only the page answers {type:"dimos:chrome-ok"}, and only then does the bar show), {type:"dimos:open-in-editor", file, line} (the parent answers {type:"dimos:open-in-editor-result", ok, text}) and {type:"dimos:close"} (its close button, or Escape). `view=logs` opens it on the Logs of `run` (else the blueprint\'s last run, even one that failed or stopped): Desktop\'s failed-run notification opens it so. 404 for a blueprint dimos doesn\'t list. The page itself has no side effects; its buttons do what the routes they call say.',
            errors=(400, 404),
            ok={"content": {"text/html": {"schema": {"type": "string"}}}},
            answer="an HTML page",
        ),
    )
    async def blueprint_view(
        name: Annotated[
            str,
            Query(
                description="blueprint name, e.g. unitree-go2-basic", examples=["unitree-go2-basic"]
            ),
        ],
        view: Annotated[
            str | None, Query(description="logs: open on the Logs view", examples=["logs"])
        ] = None,
        run: Annotated[
            str | None,
            Query(
                description="with view=logs: the run whose log to show (GET /dimos/runs/{runId}/log)",
                examples=["20260101-120000-unitree-go2"],
            ),
        ] = None,
    ) -> str:
        helpers.check_name(name)
        if view not in (None, "logs"):
            raise ApiError(400, f"view {view} isn't logs")
        if run is not None and (not re.fullmatch("[A-Za-z0-9_.-]+", run)):
            raise ApiError(400, f"not a run id: {run}")
        listed = await helpers.blueprint_list()
        if not any(entry["name"] == name for entry in listed["blueprints"]):
            raise ApiError(404, f"no such blueprint: {name}")
        return await asyncio.to_thread((VIEW_DIR / "index.html").read_text, "utf-8")
