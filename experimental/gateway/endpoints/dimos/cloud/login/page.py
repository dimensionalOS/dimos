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

from typing import Annotated, Literal

from fastapi import FastAPI, Query
from fastapi.responses import HTMLResponse

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.http import ASSETS


def register(app: FastAPI, state: ServerState) -> None:
    @app.get(
        "/dimos/cloud/login/page",
        response_class=HTMLResponse,
        **route_doc(
            "cloud",
            'A small HTML page for an app\'s iframe that runs the cloud login and posts {type:"dimos-cloud-login", state, email} to its parent',
            "A page an app embeds (the console itself refuses to be framed): it starts the login, shows the URL and code, and posts the outcome to its parent window. No side effects until it's opened.",
            errors=(400,),
            ok={"content": {"text/html": {"schema": {"type": "string"}}}},
            answer="an HTML page",
        ),
    )
    async def cloud_login_page(
        theme: Annotated[
            Literal["light", "dark"] | None,
            Query(description="light or dark (default: the system's)"),
        ] = None,
    ) -> str:
        return (ASSETS / "login_page.html").read_text()
