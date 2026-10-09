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
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import docs, models
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/docs/custom-robot",
        response_model=models.CustomRobotDoc,
        **route_doc(
            "docs",
            "dimos's guide to adding a robot of your own: markdown and HTML, with absolute links",
            "Found in the checkout's docs/ by file name and title (adding a custom/new robot, arm or platform); links point at the docs site (mkdocs.yml site_url), other repo files at GitHub. `html` is rendered here with markdown-it (raw HTML in the markdown is escaped), so a page can show it as is; `markdown` is there for a page that renders it itself. 404 when the checkout has no such page.",
            errors=(404,),
            agent=True,
            answer="`{ title, markdown, html, source_path, url, others }`",
        ),
    )
    async def custom_robot_doc() -> dict[str, Any]:
        answer = await asyncio.to_thread(docs.custom_robot, s.dimos_dir)
        if answer is None:
            raise ApiError(404, f"no guide to adding a robot in {s.dimos_dir / 'docs'}")
        return answer
