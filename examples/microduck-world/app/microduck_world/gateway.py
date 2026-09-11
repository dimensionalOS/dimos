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

"""Private HTTPS bootstrap and UDP gateway; the DimOS relay stays on loopback."""

import argparse
import re
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from ipaddress import IPv4Address, ip_network
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit, urlunsplit

import httpx
import uvicorn
from microduck_world.udp_forwarder import UdpForwarder
from pydantic import BaseModel, ConfigDict, Field, field_validator
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import FileResponse, HTMLResponse, JSONResponse, Response
from starlette.routing import Route
from starlette.staticfiles import StaticFiles

STARTING_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="3">
<title>Microduck World is starting</title>
<style>
body { margin:0; min-height:100vh; display:grid; place-items:center;
       background:#14171a; color:#d7dde3; font:16px/1.6 system-ui,sans-serif; }
main { max-width:32rem; padding:2rem; }
h1 { font-size:1.5rem; } p { color:#8b949e; }
a { color:#58a6ff; }
</style></head><body><main>
<h1>Microduck World is starting</h1>
<p>Reconnecting shortly. You can leave this page open.</p>
<a href="/">Try now</a>
</main></body></html>"""


def unavailable(request: Request) -> Response:
    headers = {"cache-control": "no-store", "retry-after": "3"}
    if request.url.path == "/" and "text/html" in request.headers.get("accept", ""):
        return HTMLResponse(STARTING_PAGE, status_code=503, headers=headers)
    return JSONResponse({"status": "starting"}, status_code=503, headers=headers)


class GatewayConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    bind_host: IPv4Address
    port: int = Field(ge=1024, le=65535)
    public_origin: str
    upstream_url: str
    cert_file: Path
    key_file: Path
    max_peers: int = Field(default=64, ge=1, le=256)
    frontend_dir: Path | None = None
    world_assets_dir: Path | None = None

    @field_validator("bind_host")
    @classmethod
    def private_interface(cls, host: IPv4Address) -> IPv4Address:
        if not host.is_loopback and host not in ip_network("100.64.0.0/10"):
            raise ValueError("Bind only to a Tailscale IPv4 address or loopback")
        return host

    @field_validator("public_origin")
    @classmethod
    def https_origin(cls, value: str) -> str:
        parsed = urlsplit(value)
        if (
            parsed.scheme != "https"
            or not parsed.hostname
            or parsed.username
            or parsed.path not in ("", "/")
            or parsed.query
            or parsed.fragment
        ):
            raise ValueError("public_origin must be an HTTPS origin without a path")
        return value.rstrip("/")

    @field_validator("upstream_url")
    @classmethod
    def local_upstream(cls, value: str) -> str:
        parsed = urlsplit(value)
        if (
            parsed.scheme != "http"
            or parsed.hostname != "127.0.0.1"
            or parsed.username
            or parsed.path not in ("", "/")
            or parsed.query
            or parsed.fragment
        ):
            raise ValueError("upstream_url must be a loopback HTTP origin")
        return value.rstrip("/")


def rewrite_info(
    info: dict[str, Any], config: GatewayConfig, forwarder: UdpForwarder
) -> dict[str, Any]:
    """Keep the end-to-end certificate pin while publishing the private UDP endpoint."""
    upstream = urlsplit(info["wtUrl"])
    if upstream.scheme != "https" or upstream.hostname != "127.0.0.1" or not upstream.port:
        raise ValueError("Relay advertised a non-loopback QUIC endpoint")
    forwarder.set_target((upstream.hostname, upstream.port))
    public = urlsplit(config.public_origin)
    return {
        **info,
        "wtUrl": urlunsplit(("https", public.netloc, upstream.path, upstream.query, "")),
    }


def create_app(config: GatewayConfig) -> Starlette:
    forwarder = UdpForwarder(max_peers=config.max_peers)
    frontend = StaticFiles(directory=config.frontend_dir) if config.frontend_dir else None

    @asynccontextmanager
    async def lifespan(app: Starlette) -> AsyncIterator[None]:
        async with httpx.AsyncClient(timeout=10.0, trust_env=False) as client:
            app.state.client = client
            await forwarder.start(str(config.bind_host), config.port)
            try:
                yield
            finally:
                await forwarder.stop()

    async def serve(request: Request) -> Response:
        # Tailnet membership is the access boundary; a foreign website must
        # not read bootstrap data or use this as its own cross-origin relay.
        if request.headers.get("host") != urlsplit(config.public_origin).netloc:
            return Response("Unexpected host", status_code=421)
        origin = request.headers.get("origin")
        if origin is not None and origin != config.public_origin:
            return Response("Origin not allowed", status_code=403)
        if request.headers.get("sec-fetch-site") == "cross-site":
            return Response("Cross-site access not allowed", status_code=403)
        path = request.url.path
        if frontend is not None and (path == "/" or path.startswith("/client/")):
            file = "index.html" if path == "/" else path.removeprefix("/client/")
            result = await frontend.get_response(file, request.scope)
            result.headers["x-content-type-options"] = "nosniff"
            result.headers["cache-control"] = "no-cache"
            return result
        if path.startswith("/world-assets/") and config.world_assets_dir is not None:
            name = path.removeprefix("/world-assets/")
            if re.fullmatch(r"scene-[a-f0-9]{20}\.json", name) is None:
                return Response("Unknown world asset", status_code=404)
            file_path = config.world_assets_dir / name
            headers = {
                "cache-control": "public, max-age=31536000, immutable",
                "vary": "accept-encoding",
            }
            if "gzip" in request.headers.get("accept-encoding", ""):
                file_path = file_path.with_suffix(".json.gz")
                headers["content-encoding"] = "gzip"
            if not file_path.is_file():
                return Response("World asset not ready", status_code=404)
            headers["x-content-type-options"] = "nosniff"
            return FileResponse(file_path, media_type="application/json", headers=headers)
        session_info = re.fullmatch(r"/sessions/[a-f0-9-]{72}/api/info", path) is not None
        if (
            path not in ("/", "/healthz", "/api/stats", "/api/info", "/api/lobby", "/sdk.js")
            and not session_info
        ):
            return Response("Not found", status_code=404)
        if request.method == "POST" and path != "/api/lobby":
            return Response("Method not allowed", status_code=405)
        body = bytearray()
        if request.method == "POST":
            async for chunk in request.stream():
                body.extend(chunk)
                if len(body) > 1024:
                    return Response("Request too large", status_code=413)
        health = path == "/healthz"
        target = httpx.URL(config.upstream_url).copy_with(
            path="/api/stats" if health else request.url.path,
            query=request.url.query.encode(),
        )
        try:
            response = await request.app.state.client.request(
                request.method, target, content=bytes(body)
            )
        except httpx.RequestError:
            return unavailable(request)
        if health:
            try:
                stats = response.json()
                robots = stats.get("robots") if isinstance(stats, dict) else None
                ready = response.is_success and isinstance(robots, list) and len(robots) > 0
            except ValueError:
                ready = False
            return JSONResponse(
                {"status": "ready" if ready else "starting"},
                status_code=200 if ready else 503,
                headers={"cache-control": "no-store"},
            )
        if (
            (path == "/api/info" or session_info)
            and request.method == "GET"
            and response.is_success
        ):
            try:
                info = rewrite_info(response.json(), config, forwarder)
            except (ValueError, KeyError, TypeError):
                return Response("Invalid relay discovery response", status_code=502)
            return JSONResponse(info, headers={"cache-control": "no-store"})
        # Deliberately omit upstream wildcard CORS and content-encoding:
        # httpx has already decoded the response body.
        headers = {
            key: response.headers[key]
            for key in ("content-type", "cache-control")
            if key in response.headers
        }
        headers["x-content-type-options"] = "nosniff"
        return Response(response.content, status_code=response.status_code, headers=headers)

    return Starlette(
        routes=[Route("/{path:path}", serve, methods=["GET", "HEAD", "POST"])], lifespan=lifespan
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    config = GatewayConfig.model_validate_json(args.config.read_text())
    uvicorn.run(
        create_app(config),
        host=str(config.bind_host),
        port=config.port,
        ssl_certfile=str(config.cert_file),
        ssl_keyfile=str(config.key_file),
        access_log=False,
    )


if __name__ == "__main__":
    main()
