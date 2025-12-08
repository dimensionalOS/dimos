from datetime import datetime
from typing import Callable, Optional
import logging
import os
import ssl
import threading
import webbrowser

import asyncio
from aiohttp import web, ClientSession, WSMsgType

from dimos.dashboard.support.html_generation import html_code_gen
from dimos.dashboard.support.utils import env_bool, path_matches, build_target_url, ensure_logger
from dimos.dashboard.support.zellij_tooling import ZellijManager

HOP_BY_HOP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailers",
    "transfer-encoding",
    "upgrade",
}

def start_dashboard_server(config: dict, log: logging.Logger):
    auto_open = config["auto_open"]
    port = config["port"]
    dashboard_host = config["dashboard_host"]
    zellij_port = config["zellij_port"]
    zellij_url = config["zellij_url"]
    zellij_session_name = config["zellij_session_name"]
    https_enabled = config["https_enabled"]
    https_key_path = config["https_key_path"]
    https_cert_path = config["https_cert_path"]
    protocol = config["protocol"]
    rrd_url = config["rrd_url"]
    zellij_token = config["zellij_token"]
    terminals = config["terminals"]
    
    # NOTE: whatever name is picked for the frontend base path cannot be a zellij session name
    # we pick/generate the session names so its not that big of a deal to avoid collisions
    frontend_base_path = "/zviewer"
    api_base_path = f"{frontend_base_path}/api"
    
    zellij_token_holder = {"token": zellij_token}
    zellij_manager = ZellijManager(
        log=log,
        session_name=zellij_session_name,
        port=zellij_port,
        terminal_commands=terminals,
        token=zellij_token
    )
    
    async def proxy_http(
        request: web.Request,
        target_base: str,
        strip_prefix: Optional[str] = None,
        add_prefix: Optional[str] = None,
    ) -> web.StreamResponse:
        session: ClientSession = request.app["client"]
        target_url = build_target_url(request, target_base, strip_prefix, add_prefix)

        try:
            data = await request.read()
            headers = {
                k: v for k, v in request.headers.items() if k.lower() not in HOP_BY_HOP_HEADERS
            }

            async with session.request(
                request.method,
                target_url,
                headers=headers,
                data=data if data else None,
                allow_redirects=False,
            ) as resp:
                resp_headers = {
                    k: v
                    for k, v in resp.headers.items()
                    if k.lower() not in HOP_BY_HOP_HEADERS
                }
                body = await resp.read()
                return web.Response(status=resp.status, headers=resp_headers, body=body)
        except Exception as exc:  # pragma: no cover - network errors
            log.error("Proxy error to %s: %s", target_url, exc)
            return web.Response(status=502, text="Upstream unavailable")

    async def proxy_websocket(
        request: web.Request,
        target_base: str,
        strip_prefix: Optional[str] = None,
        add_prefix: Optional[str] = None,
    ) -> web.StreamResponse:
        session: ClientSession = request.app["client"]
        target_url = build_target_url(request, target_base, strip_prefix, add_prefix)
        target_url = target_url.with_scheme("wss" if target_url.scheme == "https" else "ws")

        ws_server = web.WebSocketResponse()
        await ws_server.prepare(request)

        headers = {
            k: v for k, v in request.headers.items() if k.lower() not in HOP_BY_HOP_HEADERS
        }

        try:
            async with session.ws_connect(target_url, headers=headers) as ws_client:
                async def relay(ws_from, ws_to):
                    async for msg in ws_from:
                        if msg.type == WSMsgType.TEXT:
                            await ws_to.send_str(msg.data)
                        elif msg.type == WSMsgType.BINARY:
                            await ws_to.send_bytes(msg.data)
                        elif msg.type == WSMsgType.CLOSE:
                            await ws_to.close()
                            break
                        elif msg.type == WSMsgType.ERROR:
                            break

                tasks = [
                    asyncio.create_task(relay(ws_server, ws_client)),
                    asyncio.create_task(relay(ws_client, ws_server)),
                ]
                done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in pending:
                    task.cancel()
                await asyncio.gather(*pending, return_exceptions=True)
        except Exception as exc:  # pragma: no cover - network errors
            log.error("WebSocket proxy error to %s: %s", target_url, exc)
        finally:
            await ws_server.close()

        return ws_server

    

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

        if subpath in ("sessions", "sessions/"):
            try:
                data = await zellij_manager.run_zellij_list_sessions()
                return add_cors_headers(web.json_response(data))
            except Exception as exc:
                log.error("Error fetching zellij sessions: %s", exc)
                data = {"success": False, "error": str(exc)}
                return add_cors_headers(web.json_response(data, status=500))

        return add_cors_headers(web.json_response({"error": "Not found"}, status=404))

    async def dispatch(request: web.Request) -> web.StreamResponse:
        path = request.rel_url.path
        is_ws = request.headers.get("upgrade", "").lower() == "websocket"

        if path in ("/", "", "/zviewer", "/zviewer/"):
            html_code = html_code_gen(rrd_url, zellij_enabled=zellij_manager.enabled, zellij_token=zellij_token_holder["token"], session_name=zellij_manager.session_name)
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
                        "zellij": zellij_url,
                    },
                }
            )

        if path_matches(api_base_path, path):
            if is_ws:
                return web.Response(status=400, text="WebSocket not supported on API")
            subpath = path[len(api_base_path) :]
            return await handle_api(request, subpath)

        proxy_fn = proxy_websocket if is_ws else proxy_http
        return await proxy_fn(request, zellij_url)

    async def on_startup(app: web.Application):
        app["client"] = ClientSession()
        if zellij_manager.enabled:
            app["zellij_process"] = await zellij_manager.start_zellij_server(zellij_token_holder)
            log.info("🚀 Starting Zellij Session Viewer Reverse Proxy (Python)")
            log.info("🎯 Reverse Proxy Server running on %s://%s:%s", protocol, dashboard_host, port)
        
        log.info("📋 Service Routes:")
        if zellij_manager.enabled:
            log.info("   🖥️  Zellij Web Client:     %s://%s:%s/", protocol, dashboard_host, port)
        log.info(f"   📈 Rerun URL:             {rrd_url}")
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
        log.info("   ❤️  Health Check:         %s://%s:%s/health", protocol, dashboard_host, port)
        log.info("🚀 Ready to tunnel port %s!", port)
        if auto_open:
            target_url = f"{protocol}://{dashboard_host}:{port}{frontend_base_path}"

            async def _open_browser():
                try:
                    # Small delay so the server is ready before opening the browser
                    await asyncio.sleep(0.2)
                    await asyncio.get_running_loop().run_in_executor(None, webbrowser.open, target_url)
                except Exception as exc:  # pragma: no cover - environment dependent
                    log.warning("Failed to auto-open browser at %s: %s", target_url, exc)

            asyncio.create_task(_open_browser())

    async def on_cleanup(app: web.Application):
        client: ClientSession = app["client"]
        await client.close()
        proc: Optional[asyncio.subprocess.Process] = app.get("zellij_process", None)
        if proc and proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=2)
            except asyncio.TimeoutError:
                proc.kill()

    def create_app() -> web.Application:
        app = web.Application()
        app.router.add_route("*", "/{path:.*}", dispatch)
        app.on_startup.append(on_startup)
        app.on_cleanup.append(on_cleanup)
        return app

    def build_ssl_context() -> Optional[ssl.SSLContext]:
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
    terminal_commands: Optional[dict[str, str]] = None,
    https_enabled: bool = env_bool("HTTPS_ENABLED", False),
    zellij_host: str = os.environ.get("ZELLIJ_HOST", "127.0.0.1"),
    zellij_port: int = int(os.environ.get("ZELLIJ_PORT", "8083")),
    zellij_token: Optional[str] = os.environ.get("ZELLIJ_TOKEN"),
    zellij_url: Optional[str] = None,
    zellij_session_name: Optional[str] = "dimos-dashboard",
    https_key_path: Optional[str] = os.environ.get("HTTPS_KEY_PATH"),
    https_cert_path: Optional[str] = os.environ.get("HTTPS_CERT_PATH"),
    logger: Optional[logging.Logger] = None,
    rrd_url: Optional[str] = None,
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
                zellij_port=zellij_port,
                zellij_url=zellij_url or f"{protocol}://{zellij_host}:{zellij_port}",
                zellij_session_name=zellij_session_name,
                https_enabled=https_enabled,
                https_key_path=https_key_path,
                https_cert_path=https_cert_path,
                protocol=protocol,
                rrd_url=rrd_url,
                zellij_token=zellij_token,
                terminals=terminal_commands,
            ),
            ensure_logger(logger, "dashboard")
        ),
        daemon=True,
        name="proxy-server",
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
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(
                name="Spatial3D",
                origin="/",
                line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
            ),
        )
    ))
    print("starting server")
    t = start_dashboard_server_thread(
        zellij_session_name="dimos-dashboard",
        rrd_url=rr.serve_grpc(),
        terminals={
            "agent-spy": "dimos agentspy",
            "lcm-spy": "dimos lcmspy",
            # "skill-spy": "dimos skillspy",
        },
    )
    try:
        while t.is_alive():
            t.join(timeout=0.5)
    except KeyboardInterrupt:
        print("Received interrupt; shutting down.")
