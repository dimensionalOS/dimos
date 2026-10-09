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

from typing import Annotated, Any

from fastapi import FastAPI, Path as PathParam, Query

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models
from experimental.gateway.utils.http import ApiError

JobParam = Annotated[str, PathParam(description="a job id", examples=["extras-1-1791000000"])]


def register(app: FastAPI, state: ServerState) -> None:
    jobs = state.jobs
    assert jobs is not None

    @app.get(
        "/dimos/jobs/{job}/log",
        response_model=models.JobLog,
        **route_doc(
            "jobs",
            "A job's output so far, and how it ended (`error`, and `failure`: its last lines)",
            "Lines from `after` on (default 0) and `next`, the `n` the next line will have: subscribe to `<ns>/dimos/jobs/<job>` first, then fetch this, then apply live lines with `n` >= `next`. 404 for an unknown (or expired) job.",
            errors=(404,),
            agent=True,
            answer="`{ job, title, kind, command, lines, next, done, ok, error, failure, started_at, finished_at }`",
        ),
    )
    async def job_log(
        job: JobParam,
        after: Annotated[int, Query(description="the first line's `n` to answer")] = 0,
    ) -> dict[str, Any]:
        try:
            return jobs.get(job).log(after)
        except KeyError as error:
            raise ApiError(404, error.args[0])
