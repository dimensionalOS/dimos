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
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/uploads",
        response_model=models.UploadList,
        **route_doc(
            "uploads",
            "The Dimensional cloud upload queue: each upload's state, progress, speed, time left and error",
            "The queue in order, and whether it waits for a cloud login. No side effects.",
            agent=True,
            answer="`{ uploads: [Upload], waitingForLogin }`",
        ),
    )
    async def upload_list() -> dict[str, Any]:
        return s.uploads.listing()

    @app.post(
        "/dimos/uploads",
        response_model=models.Upload,
        **route_doc(
            "uploads",
            "Upload a recording (.mcap or .db) to Dimensional cloud: it joins the queue (one at a time)",
            "Adds the recording to the end of the queue (saved, so it survives a restart) and answers its upload, queued; one already queued or uploading for that path is answered instead. Needs a cloud login: without one it waits. 400 when the path isn't an absolute path to an existing dimos recording (an .mcap, or a .db dimos recorded).",
            errors=(400, 500),
            agent=True,
            answer="`Upload`",
        ),
    )
    async def enqueue_upload(request: models.UploadRequest) -> dict[str, Any]:
        try:
            return s.uploads.enqueue(request.path, request.robotId, request.kind)
        except ValueError as error:
            raise ApiError(400, str(error))

    @app.delete(
        "/dimos/uploads",
        response_model=models.UploadList,
        **route_doc(
            "uploads",
            "Clear the finished uploads (done, failed, cancelled) from the list",
            "Removes every done, failed or cancelled upload from the list (what is in the cloud is still remembered, see /dimos/uploads/uploaded) and answers the queue.",
            answer="`{ uploads: [Upload], waitingForLogin }`, without the finished ones",
        ),
    )
    async def clear_uploads() -> dict[str, Any]:
        return s.uploads.clear_finished()
