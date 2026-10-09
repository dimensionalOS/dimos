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

from fastapi import FastAPI, Query

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/uploads/uploaded",
        response_model=models.UploadedByPath | models.Uploaded | None,
        **route_doc(
            "uploads",
            "Which recordings are already in Dimensional cloud (uploaded from this machine), by path, with a console link; ?path= for one",
            "Without `path`: `{byPath}` for every recording uploaded from here. With `path`: that one's entry, or null when it isn't uploaded. `changed` says the file differs from what was uploaded. No side effects.",
            errors=(400, 500),
            agent=True,
            answer="`{ byPath: { [path]: Uploaded } }`, or with `?path=` that one `Uploaded` or null",
        ),
    )
    async def uploaded(
        path: Annotated[str | None, Query(description="a recording's absolute path")] = None,
    ) -> Any:
        return s.uploads.uploaded() if path is None else s.uploads.uploaded_one(path)
