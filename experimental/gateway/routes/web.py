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
import hashlib
from pathlib import Path
import re
from typing import Any, Literal

from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse, Response

from experimental.gateway.msgs import JS_FILE
from experimental.gateway.routes.blueprints import blueprint_list, check_name
from experimental.gateway.state import ApiError, GatewayState

router = APIRouter()

ASSETS = Path(__file__).parents[1] / "assets"
VIEW_DIR = ASSETS / "blueprint_view"
VIEW_FILE = re.compile(r"[a-z_]+\.(js|css)")


@router.get("/dimos/blueprint_view", response_class=HTMLResponse)
async def blueprint_view(
    state: GatewayState, name: str, view: str | None = None, run: str | None = None
) -> str:
    check_name(name)
    if view not in (None, "logs"):
        raise ApiError(400, f"view {view} isn't logs")
    if run is not None and not re.fullmatch("[A-Za-z0-9_.-]+", run):
        raise ApiError(400, f"not a run id: {run}")
    listed = await blueprint_list(state)
    if not any(entry["name"] == name for entry in listed["blueprints"]):
        raise ApiError(404, f"no such blueprint: {name}")
    return (VIEW_DIR / "index.html").read_text()


@router.get("/dimos/blueprint_view/{file}")
async def blueprint_view_file(file: str) -> Response:
    path = VIEW_DIR / file
    if not VIEW_FILE.fullmatch(file) or not path.is_file():
        raise ApiError(404, f"no such file: {file}")
    media = "text/css" if file.endswith(".css") else "text/javascript"
    return Response(
        path.read_bytes(),
        media_type=f"{media}; charset=utf-8",
        headers={"cache-control": "no-cache"},
    )


@router.get("/dimos/source")
async def source(state: GatewayState, file: str) -> dict[str, Any]:
    root = state.dimos_dir.resolve()
    path = (root / file).resolve()
    if path.suffix != ".py" or not path.is_relative_to(root):
        raise ApiError(400, f"not a .py file in the dimos checkout: {file}")
    if not path.is_file():
        raise ApiError(404, f"no such file: {file}")
    return {"file": file, "text": await asyncio.to_thread(path.read_text, "utf-8", "replace")}


@router.get("/dimos/topics/rates")
async def topic_rates(state: GatewayState) -> dict[str, Any]:
    if state.topics is None:
        return {"up": False, "error": "this gateway isn't listening to the bus", "topics": []}
    answer: dict[str, Any] = await asyncio.to_thread(state.topics.snapshot)
    return answer


@router.get("/dimos/msgs.js")
async def msgs_js(request: Request) -> Response:
    if not JS_FILE.is_file():
        raise ApiError(404, "no msgs.js in this install of dimos")
    body = JS_FILE.read_bytes()
    etag = f'"{hashlib.sha256(body).hexdigest()}"'
    headers = {"etag": etag, "cache-control": "no-cache"}
    if etag in (tag.strip() for tag in request.headers.get("if-none-match", "").split(",")):
        return Response(status_code=304, headers=headers)
    return Response(body, media_type="text/javascript; charset=utf-8", headers=headers)


@router.get("/dimos/cloud/login/page", response_class=HTMLResponse)
async def cloud_login_page(theme: Literal["light", "dark"] | None = None) -> str:
    return (ASSETS / "login_page.html").read_text()
