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
from typing import Annotated

from fastapi import FastAPI, Path as PathParam
from fastapi.responses import Response

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.http import VIEW_DIR, VIEW_FILE, ApiError


def register(app: FastAPI, state: ServerState) -> None:
    @app.get(
        "/dimos/blueprint_view/{file}",
        response_class=Response,
        **route_doc(
            "blueprints",
            "One of the blueprint view page's own files (its scripts and styles)",
            "Serves app.js, graph.js, layout.js, view.css or portal.css from experimental/gateway/blueprint_view/: only a plain .js or .css name in that folder (404 for anything else). No side effects.",
            errors=(404,),
            ok={
                "content": {
                    "text/javascript": {"schema": {"type": "string"}},
                    "text/css": {"schema": {"type": "string"}},
                }
            },
            answer="the file",
        ),
    )
    async def blueprint_view_file(
        file: Annotated[
            str, PathParam(description="the file's name, e.g. app.js", examples=["app.js"])
        ],
    ) -> Response:
        path = VIEW_DIR / file
        if not VIEW_FILE.fullmatch(file) or not path.is_file():
            raise ApiError(404, f"no such file: {file}")
        media = "text/css" if file.endswith(".css") else "text/javascript"
        return Response(
            await asyncio.to_thread(path.read_bytes),
            media_type=f"{media}; charset=utf-8",
            headers={"cache-control": "no-cache"},
        )
