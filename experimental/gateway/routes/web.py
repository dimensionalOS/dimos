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
from pathlib import Path
import re
from typing import Any

from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse, Response

from experimental.gateway import msgs, responses, store, web_view
from experimental.gateway.routes.blueprints import blueprint_list, check_name
from experimental.gateway.state import ApiError, GatewayState

router = APIRouter()

ASSETS = Path(__file__).parents[1] / "assets"
# vite's hashed bundle names, e.g. index-rD2X8j_J.js
VIEW_FILE = re.compile(r"[A-Za-z0-9_-]+\.(js|css)")


@router.get("/blueprint_view", response_class=HTMLResponse)
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
    dist = await web_view.built_dist()
    if dist is None:
        raise ApiError(503, "the blueprint view isn't built (web/blueprint_view: deno task build)")
    return (dist / "index.html").read_text()


@router.get("/blueprint_view/{file}")
async def blueprint_view_file(file: str) -> Response:
    path = web_view.app_dir() / "dist" / "blueprint_view" / file
    if not VIEW_FILE.fullmatch(file) or not path.is_file():
        raise ApiError(404, f"no such file: {file}")
    media = "text/css" if file.endswith(".css") else "text/javascript"
    return Response(
        path.read_bytes(),
        media_type=f"{media}; charset=utf-8",
        headers={"cache-control": "public, max-age=31536000, immutable"},
    )


@router.get("/source", response_model=responses.SourceFile)
async def source(state: GatewayState, file: str) -> dict[str, Any]:
    root = state.dimos_dir.resolve()
    path = (root / file).resolve()
    if path.suffix != ".py" or not path.is_relative_to(root):
        raise ApiError(400, f"not a .py file in the dimos checkout: {file}")
    if not path.is_file():
        raise ApiError(404, f"no such file: {file}")
    return {"file": file, "text": await asyncio.to_thread(path.read_text, "utf-8", "replace")}


@router.get("/msgs.js")
async def msgs_js(request: Request) -> Response:
    try:
        path = await msgs.built_js(store.gateway_dir() / "msgs")
    except RuntimeError as error:
        raise ApiError(503, str(error)) from error
    etag = f'"{path.stem}"'  # the file name holds the hash of everything it's built from
    headers = {"etag": etag, "cache-control": "no-cache"}
    if etag in (tag.strip() for tag in request.headers.get("if-none-match", "").split(",")):
        return Response(status_code=304, headers=headers)
    return Response(path.read_bytes(), media_type="text/javascript; charset=utf-8", headers=headers)


@router.get("/cloud/login/page", response_class=HTMLResponse)
async def cloud_login_page() -> str:
    return (ASSETS / "login_page.html").read_text()
