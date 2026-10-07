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

"""The gateway listens on 127.0.0.1 without a login, so a browser on this machine can reach it: refuse a request whose
Host isn't a loopback name (DNS rebinding) or whose Origin is another site (a page posting to it, or opening a
websocket). Desktop's proxy sends `Host: localhost` and drops Origin after its own checks."""

from __future__ import annotations

from typing import Any
from urllib.parse import urlsplit

LOOPBACK_NAMES = {"127.0.0.1", "localhost", "::1"}


def host_name(authority: str) -> str:
    """`127.0.0.1:5557` → `127.0.0.1`, `[::1]:5557` → `::1`."""
    if authority.startswith("["):
        return authority[1:].split("]", 1)[0]
    return authority.rsplit(":", 1)[0] if authority.count(":") == 1 else authority


def allowed(host: str | None, origin: str | None) -> bool:
    """Host is a loopback name, and Origin is absent or this same origin (the gateway's own page)."""
    if host is None or host_name(host.lower()) not in LOOPBACK_NAMES:
        return False
    if origin is None:
        return True
    return urlsplit(origin.lower()).netloc == host.lower()


class LoopbackOnly:
    """ASGI middleware: `allowed` or 403 (HTTP and websockets)."""

    def __init__(self, app: Any) -> None:
        self.app = app

    async def __call__(self, scope: dict[str, Any], receive: Any, send: Any) -> None:
        if scope["type"] not in ("http", "websocket"):
            return await self.app(scope, receive, send)
        headers = {
            name.decode("latin-1"): value.decode("latin-1") for name, value in scope["headers"]
        }
        if allowed(headers.get("host"), headers.get("origin")):
            return await self.app(scope, receive, send)
        if scope["type"] == "websocket":
            return await send({"type": "websocket.close", "code": 1008})
        body = b'{"error":"the dimos gateway only answers this machine (Host 127.0.0.1/localhost, no other origin)"}'
        await send(
            {
                "type": "http.response.start",
                "status": 403,
                "headers": [
                    (b"content-type", b"application/json"),
                    (b"content-length", str(len(body)).encode()),
                ],
            }
        )
        await send({"type": "http.response.body", "body": body})
