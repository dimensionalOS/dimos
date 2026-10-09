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
from typing import Annotated, Any

from fastapi import FastAPI, Query

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/source",
        response_model=models.SourceFile,
        **route_doc(
            "blueprints",
            "A Python file of the dimos checkout, as text (a module's code)",
            "`file` is a path relative to the checkout (as GET /dimos/blueprints/{name} gives a module's `file`) or an absolute one inside it. Only .py files inside the checkout are read: 400 for anything else, 404 when there's no such file.",
            errors=(400, 404),
            answer="`{ file, text }`",
        ),
    )
    async def source(
        file: Annotated[
            str, Query(description="The file: relative to the checkout, or absolute inside it")
        ],
    ) -> Any:
        root = s.dimos_dir.resolve()
        path = (root / file).resolve()
        if path.suffix != ".py" or not path.is_relative_to(root):
            raise ApiError(400, f"not a .py file in the dimos checkout: {file}")
        if not path.is_file():
            raise ApiError(404, f"no such file: {file}")
        return {"file": file, "text": await asyncio.to_thread(path.read_text, "utf-8", "replace")}
