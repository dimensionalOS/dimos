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

    @app.delete(
        "/dimos/uploads/{id}",
        response_model=models.Ok,
        **route_doc(
            "uploads",
            "Cancel a queued or running upload, or remove a finished one from the list",
            "A queued upload turns cancelled; a running one's worker is killed and it turns cancelled; a finished one is removed (an `upload-removed` event). 404 for an unknown id.",
            errors=(404, 500),
            agent=True,
            answer="`{ ok: true }`",
        ),
    )
    async def cancel_upload(id: UploadIdParam) -> dict[str, Any]:
        try:
            s.uploads.cancel(id)
        except KeyError as error:
            raise ApiError(404, error.args[0])
        return {"ok": True}
