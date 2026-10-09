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
from experimental.gateway.utils.http import ApiError, UploadIdParam


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.post(
        "/dimos/uploads/{id}/retry",
        response_model=models.Upload,
        **route_doc(
            "uploads",
            "Queue a failed or cancelled upload again",
            "Moves a finished (done, failed or cancelled) upload to the back of the queue, reset, and answers it. 404 for an unknown id; 409 while it's still queued or uploading.",
            errors=(404, 409, 500),
            agent=True,
            answer="`Upload`, queued",
        ),
    )
    async def retry_upload(id: UploadIdParam) -> dict[str, Any]:
        try:
            return s.uploads.retry(id)
        except KeyError as error:
            raise ApiError(404, error.args[0])
        except ValueError as error:
            raise ApiError(409, str(error))
