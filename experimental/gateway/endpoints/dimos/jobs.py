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

from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models


def register(app: FastAPI, state: ServerState) -> None:
    jobs = state.jobs
    assert jobs is not None

    @app.get(
        "/dimos/jobs",
        response_model=models.JobList,
        **route_doc(
            "jobs",
            "The dimos gateway's jobs (extras installs): running, and finished in the last 30 minutes",
            "No side effects.",
            answer="`{ jobs: [{ job, title, kind, done, ok, started_at, finished_at }] }`",
        ),
    )
    async def job_list() -> dict[str, Any]:
        return {"jobs": jobs.listing()}
