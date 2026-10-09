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

from fastapi import FastAPI, Request, Response

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.msgs.codegen import TS_FILE

USAGE = "Import it from a page: `import { decodeMessage, geometry_msgs } from \"../../dimos/msgs.js\"`. It exports decode(bytes) (by the frame's fingerprint), decodeChannel(channelOrKey, bytes) (the `#<pkg>.<Type>` of an LCM channel or the last segment of a zenoh key `dimos/<topic>/<pkg>.<Type>` wins over the fingerprint), decodeMessage(message) for a zenoh-gateway client Message ({key, bytes}), getTypeNames(), getMissingTypes() (dimos messages with no LCM schema), register(name, fingerprint, decode, encode?) for those or an app's own, and a namespace per package: geometry_msgs.Twist.decode(bytes), .encode(value), .zenohKey(topic), .lcmChannel(topic). Generated from dimos/msgs/ by `python -m experimental.gateway.msgs --write`. Answered with an ETag (the content's sha256) and `Cache-Control: no-cache`: If-None-Match with the current one answers 304. No side effects."


def _serve(path: Path, media: str, request: Request) -> Response:
    if not path.is_file():
        from experimental.gateway.utils.http import ApiError

        raise ApiError(404, f"no such file: {path.name} (not shipped with this install of dimos)")
    body = path.read_bytes()
    etag = f'"{hashlib.sha256(body).hexdigest()}"'
    headers = {"etag": etag, "cache-control": "no-cache"}
    if etag in (tag.strip() for tag in request.headers.get("if-none-match", "").split(",")):
        return Response(status_code=304, headers=headers)
    return Response(body, media_type=f"{media}; charset=utf-8", headers=headers)


def register(app: FastAPI, state: ServerState) -> None:
    @app.get(
        "/dimos/msgs.ts",
        response_class=Response,
        **route_doc(
            "discovery",
            "Every dimos message's LCM decoder and encoder, as TypeScript with each message's type",
            "The TypeScript source of GET /dimos/msgs.js, for Deno and TypeScript (an interface per message, `geometry_msgs.PoseStamped` both a type and its codec). 404 where dimos is installed from a wheel (only the .js ships). "
            + USAGE,
            errors=(404,),
            ok={"content": {"application/typescript": {"schema": {"type": "string"}}}},
            answer="the TypeScript module",
        ),
    )
    async def msgs_ts(request: Request) -> Response:
        return await asyncio.to_thread(_serve, TS_FILE, "application/typescript", request)
