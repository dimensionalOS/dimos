# Copyright 2025 Dimensional Inc.
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

import asyncio
from datetime import datetime
import logging
import os
import ssl
import threading
import webbrowser

from aiohttp import web

from dimos.dashboard.support.html_generation import html_code_gen
from dimos.dashboard.support.utils import ensure_logger, env_bool, path_matches


def start_dashboard_server(config: dict, log: logging.Logger):
    auto_open = config["auto_open"]
    port = config["port"]
    dashboard_host = config["dashboard_host"]
    https_enabled = config["https_enabled"]
    https_key_path = config["https_key_path"]
    https_cert_path = config["https_cert_path"]
    protocol = config["protocol"]
    rrd_url = config["rrd_url"]

    if not rrd_url:
        raise RuntimeError("rrd_url must be provided to start the dashboard server")

    # NOTE: whatever name is picked for the frontend base path cannot be a zellij session name
    # we pick/generate the session names so its not that big of a deal to avoid collisions
    frontend_base_path = "/zviewer"
    api_base_path = f"{frontend_base_path}/api"

    def add_cors_headers(resp: web.StreamResponse) -> web.StreamResponse:
        resp.headers["Access-Control-Allow-Origin"] = "*"
        resp.headers["Access-Control-Allow-Methods"] = "GET,OPTIONS"
        resp.headers["Access-Control-Allow-Headers"] = "*"
        return resp

    async def handle_api(request: web.Request, subpath: str) -> web.StreamResponse:
        if request.method == "OPTIONS":
            return add_cors_headers(web.Response(status=204))

        if subpath.startswith("/"):
            subpath = subpath[1:]

        if subpath in ("health", "health/"):
            data = {
                "status": "ok",
                "timestamp": datetime.utcnow().isoformat() + "Z",
            }
            return add_cors_headers(web.json_response(data))

        return add_cors_headers(web.json_response({"error": "Not found"}, status=404))

    async def dispatch(request: web.Request) -> web.StreamResponse:
        path = request.rel_url.path

        if path in ("/", "", "/zviewer", "/zviewer/"):
            html_code = html_code_gen(
                rrd_url,
                zellij_enabled=False,
                zellij_token=None,
            )
            return web.Response(text=html_code, content_type="text/html")

        if path == "/health":
            return web.json_response(
                {
                    "status": "ok",
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "services": {
                        "frontend": f"{protocol}://{dashboard_host}:{port}/zviewer",
                        "api": f"{protocol}://{dashboard_host}:{port}{api_base_path}",
                        "rerun": rrd_url,
                    },
                }
            )

        if path_matches(api_base_path, path):
            subpath = path[len(api_base_path) :]
            return await handle_api(request, subpath)

        return web.Response(status=404, text="Not found")

    async def on_startup(app: web.Application):
        log.info("📋 Service Routes:")
        log.info("   🎛  Main Dashboard:        %s://%s:%s/", protocol, dashboard_host, port)
        rrd_display = rrd_url.replace("rerun+", "") if isinstance(rrd_url, str) else str(rrd_url)
        log.info(f"   📈 Rerun GRPC:            {rrd_display}")
        log.info(
            "   📱 Session Manager UI:    %s://%s:%s%s/",
            protocol,
            dashboard_host,
            port,
            frontend_base_path,
        )
        log.info(
            "   🔌 Backend API:           %s://%s:%s%s/",
            protocol,
            dashboard_host,
            port,
            api_base_path,
        )
        log.info("   ❤️  Health Check:          %s://%s:%s/health", protocol, dashboard_host, port)
        log.info("🚀 Ready to tunnel port %s!", port)
        if auto_open:
            target_url = f"{protocol}://{dashboard_host}:{port}{frontend_base_path}"

            async def _open_browser():
                try:
                    # Small delay so the server is ready before opening the browser
                    await asyncio.sleep(0.2)
                    await asyncio.get_running_loop().run_in_executor(
                        None, webbrowser.open, target_url
                    )
                except Exception as exc:  # pragma: no cover - environment dependent
                    log.warning("Failed to auto-open browser at %s: %s", target_url, exc)

            asyncio.create_task(_open_browser())

    async def on_cleanup(app: web.Application):
        return None

    def create_app() -> web.Application:
        app = web.Application()
        app.router.add_route("*", "/{path:.*}", dispatch)
        app.on_startup.append(on_startup)
        app.on_cleanup.append(on_cleanup)
        return app

    def build_ssl_context() -> ssl.SSLContext | None:
        if not https_enabled:
            return None

        if not https_key_path or not https_cert_path:
            raise RuntimeError("HTTPS enabled but HTTPS_KEY_PATH or HTTPS_CERT_PATH not set")

        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=https_cert_path, keyfile=https_key_path)
        return context

    ssl_context = build_ssl_context()
    app = create_app()
    try:
        web.run_app(
            app,
            host=dashboard_host,
            port=port,
            ssl_context=ssl_context,
            access_log=None,
            handle_signals=False,
        )
    except Exception as exc:  # pragma: no cover - runtime errors
        log.error("Failed to start dashboard server: %s", exc)
        raise


def start_dashboard_server_thread(
    *,
    auto_open: bool = False,
    port: int = int(os.environ.get("DASHBOARD_PORT", "4000")),
    dashboard_host: str = os.environ.get("DASHBOARD_HOST", "localhost"),
    https_enabled: bool = env_bool("HTTPS_ENABLED", False),
    https_key_path: str | None = os.environ.get("HTTPS_KEY_PATH"),
    https_cert_path: str | None = os.environ.get("HTTPS_CERT_PATH"),
    logger: logging.Logger | None = None,
    rrd_url: str | None = None,
    keep_alive: bool = False,
    **kwargs,
) -> threading.Thread:
    protocol = "https" if https_enabled else "http"
    thread = threading.Thread(
        target=start_dashboard_server,
        args=(
            dict(
                auto_open=auto_open,
                port=port,
                dashboard_host=dashboard_host,
                https_enabled=https_enabled,
                https_key_path=https_key_path,
                https_cert_path=https_cert_path,
                protocol=protocol,
                rrd_url=rrd_url,
            ),
            ensure_logger(logger, "dashboard"),
        ),
        daemon=not keep_alive,
        name="dashboard-server",
    )
    thread.start()
    return thread


if __name__ == "__main__":
    import rerun as rr
    import rerun.blueprint as rrb

    # import rerun.blueprint as rrb
    # there's basically 3 parts to rerun
    # 1. some kind of python init that does local message aggregation
    # 2. the actual (separate process) grpc message aggregator
    # 3. the viewer/renderer
    # init starts part 1 (needed before rr.log or rr.send_blueprint)
    # we manually start the gprc here (part 2)
    # we serve our own viewer via a webserver (part 3) which is why the init has spawn=False
    rr.init("rerun_main", spawn=False)
    # send an empty blueprint to get the initial state
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Tabs(
                rrb.Spatial3DView(
                    name="Spatial3D",
                    origin="/",
                    line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
                ),
            )
        )
    )
    print("starting server")
    t = start_dashboard_server_thread(
        rrd_url=rr.serve_grpc(),
    )
    try:
        while t.is_alive():
            t.join(timeout=0.5)
    except KeyboardInterrupt:
        print("Received interrupt; shutting down.")
