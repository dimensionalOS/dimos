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

from fastapi import FastAPI, Path as PathParam

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models
from experimental.gateway.utils.http import ApiError

JobParam = Annotated[str, PathParam(description="a job id", examples=["extras-1-1791000000"])]


def register(app: FastAPI, state: ServerState) -> None:
    jobs = state.jobs
    assert jobs is not None

    @app.delete(
        "/dimos/jobs/{job}",
        response_model=models.JobLog,
        **route_doc(
            "jobs",
            "Cancel a running job",
            "Sends the job's process group SIGTERM; it ends with `ok: false`, `error: cancelled`. A finished job is answered as it is. 404 for an unknown job.",
            errors=(404,),
            answer="`JobLog`",
        ),
    )
    async def cancel_job(job: JobParam) -> dict[str, Any]:
        try:
            return jobs.cancel(job).log()
        except KeyError as error:
            raise ApiError(404, error.args[0])
