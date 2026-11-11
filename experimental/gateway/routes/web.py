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

from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse, Response

from experimental.gateway import msgs, store

router = APIRouter()

ASSETS = Path(__file__).parents[1] / "assets"


@router.get("/msgs.js")
async def msgs_js(request: Request) -> Response:
    path = await msgs.built_js(store.gateway_dir() / "msgs")
    etag = f'"{path.stem}"'  # the file name holds the hash of everything it's built from
    headers = {"etag": etag, "cache-control": "no-cache"}
    if etag in (tag.strip() for tag in request.headers.get("if-none-match", "").split(",")):
        return Response(status_code=304, headers=headers)
    return Response(path.read_bytes(), media_type="text/javascript; charset=utf-8", headers=headers)


@router.get("/cloud/login/page", response_class=HTMLResponse)
async def cloud_login_page() -> str:
    return (ASSETS / "login_page.html").read_text()
