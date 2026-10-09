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
from pathlib import Path
from typing import Annotated, Any

from fastapi import FastAPI, Path as PathParam, Query

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import logs, models, runs


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/runs/{runId}/log",
        response_model=models.LogPage,
        **route_doc(
            "logs",
            "A run's structured log (main.jsonl): records with level, logger, event",
            "Reads `<logs>/<runId>/main.jsonl` off disk (the checkout's logs/, then the library install's; the last 4 MB at most). Without `after`: the last `limit` matching records; with `after`: every matching record past that byte offset (tailing). An unknown run answers no records. No side effects.",
            errors=(400, 500),
            agent=True,
            answer="`{ runId, records: [{ timestamp, level, logger, event, extra, raw }], offset, loggers }`",
        ),
    )
    async def log(
        run_id: Annotated[
            str,
            PathParam(
                alias="runId",
                description="run id or latest",
                examples=["latest", "20260101-120000-unitree-go2"],
            ),
        ],
        after: Annotated[
            int | None,
            Query(description="byte offset from an earlier answer's `offset`: only newer records"),
        ] = None,
        level: Annotated[
            str | None,
            Query(description="minimum level: debug, info, warning, error", examples=["warning"]),
        ] = None,
        q: Annotated[str | None, Query(description="text to match")] = None,
        limit: Annotated[
            int | None,
            Query(description="at most this many records (default 1000; ignored with `after`)"),
        ] = None,
    ) -> dict[str, Any]:
        filter = logs.Filter(query=q or None, min_level=level or None)
        other = await asyncio.to_thread(runs.find_local_run, run_id) if run_id != "latest" else None
        if other and other.get("log_dir") and (not other.get("ours")):
            file = Path(str(other["log_dir"])) / "main.jsonl"
            return await asyncio.to_thread(
                logs.read_file, file, run_id, after, limit or 1000, filter
            )
        return await asyncio.to_thread(logs.read, s.dimos_dir, run_id, after, limit or 1000, filter)
