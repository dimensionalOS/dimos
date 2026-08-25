from __future__ import annotations

import asyncio
from collections.abc import AsyncGenerator, Callable
import concurrent.futures
import hmac
import ipaddress
import json
import os
import time
from typing import TYPE_CHECKING, Any
import uuid

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from starlette.middleware.base import RequestResponseEndpoint
from starlette.requests import Request
from starlette.responses import Response, StreamingResponse
import uvicorn

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CapabilityRegistry
from dimos.agents.mcp import tool_stream
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.core.transport_factory import make_transport
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig
    from dimos.core.module import SkillInfo

logger = setup_logger()

_SSE_KEEPALIVE_INTERVAL = 20.0  # seconds

# How long a `tools/call` waits for a capability held by a short, self-completing
# (instant) skill before refusing. Well under the MCP client's 120s HTTP timeout.
# Background holders run until stopped, so they are never waited on (see
# `_can_wait` in `_handle_tools_call`).
DEFAULT_CAP_ACQUIRE_TIMEOUT = 30.0  # seconds

# No legitimate JSON-RPC request comes close to this; anything larger is anomalous.
MAX_MCP_BODY_BYTES = 1024 * 1024  # 1 MB


def is_loopback_host(host: str | None) -> bool:
    """True for loopback IPs and `localhost`; False for anything else (incl. unknown)."""
    if not host:
        return False
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return host == "localhost"


def _load_global_config() -> GlobalConfig:
    # Imported lazily so unit tests that only need the FastAPI `app` don't pay
    # for the full settings import chain at module import time.
    from dimos.core.global_config import global_config

    return global_config


def _check_bind_allowed(host: str, token: str | None) -> None:
    """Fail-closed startup guard (I2): binding a non-loopback address without an
    auth token must crash at startup, not be discovered later by a port scan."""
    if not is_loopback_host(host) and not token:
        raise RuntimeError(
            f"Refusing to start MCP server: listen_host '{host}' is not loopback "
            "but mcp_auth_token (DIMOS_MCP_AUTH_TOKEN) is not set. "
            "Set a token or bind to 127.0.0.1."
        )


app = FastAPI()


@app.middleware("http")
async def _mcp_auth_middleware(request: Request, call_next: RequestResponseEndpoint) -> Response:
    """Bearer-token gate for `/mcp` (POST and the GET/SSE channel alike).

    Loopback clients are trusted without a token. Non-loopback clients must
    present `mcp_auth_token` as `Authorization: Bearer <token>`; the 401 body
    is identical for missing and wrong tokens so it doesn't leak whether a
    token is configured at all (I1). Registered before CORSMiddleware below so
    CORS stays outermost and browser preflights are answered by it.
    """
    if request.url.path != "/mcp":
        return await call_next(request)
    client_host = request.client.host if request.client is not None else None
    if is_loopback_host(client_host):
        return await call_next(request)
    expected = _load_global_config().mcp_auth_token
    got = request.headers.get("authorization", "").removeprefix("Bearer ")
    if not expected or not hmac.compare_digest(got.encode(), expected.encode()):
        return JSONResponse({"error": "unauthorized"}, status_code=401)
    return await call_next(request)


app.add_middleware(
    CORSMiddleware,
    # Empty by default = all cross-origin browser requests denied (I4);
    # curl/CLI are unaffected since CORS is only enforced by browsers.
    allow_origins=_load_global_config().mcp_allowed_origins,
    allow_methods=["POST", "GET"],
    allow_headers=["authorization", "content-type"],
)
app.state.skills = []
app.state.skills_by_name = {}
app.state.rpc_calls = {}
app.state.sse_queues = []
app.state.event_loop = None
app.state.cap_registry = CapabilityRegistry()
app.state.cap_acquire_timeout = DEFAULT_CAP_ACQUIRE_TIMEOUT


def _jsonrpc_result(req_id: Any, result: Any) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": req_id, "result": result}


def _jsonrpc_result_text(req_id: Any, text: str) -> dict[str, Any]:
    return _jsonrpc_result(req_id, {"content": [{"type": "text", "text": text}]})


def _jsonrpc_error(req_id: Any, code: int, message: str) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}}


def _handle_initialize(req_id: Any) -> dict[str, Any]:
    return _jsonrpc_result(
        req_id,
        {
            "protocolVersion": "2025-11-25",
            "capabilities": {"tools": {}, "logging": {}},
            "serverInfo": {"name": "dimensional", "version": "1.0.0"},
        },
    )


def _handle_tools_list(req_id: Any, skills: list[SkillInfo]) -> dict[str, Any]:
    tools = []
    for s in skills:
        schema = json.loads(s.args_schema)
        description = schema.pop("description", None)
        schema.pop("title", None)
        tool: dict[str, Any] = {"name": s.func_name, "inputSchema": schema}
        if description:
            tool["description"] = description
        if s.uses or s.lifecycle != "instant":
            tool["_meta"] = {
                "dimos/uses": list(s.uses),
                "dimos/lifecycle": s.lifecycle,
            }
        tools.append(tool)
    return _jsonrpc_result(req_id, {"tools": tools})


async def _handle_tools_call(
    req_id: Any,
    params: dict[str, Any],
    rpc_calls: dict[str, Any],
    *,
    client_host: str | None = None,
) -> dict[str, Any]:
    name = params.get("name", "")
    args: dict[str, Any] = params.get("arguments") or {}
    meta = params.get("_meta") or {}
    progress_token = meta.get("progressToken")
 
    # TODO(L2): remove once RiskLevel.EXTERNAL lands
    # agent_send injec ts arbitrary text into /human_input — i.e. lets any caller
    # impersonate the human operator. Until skills carry risk levels, restrict
    # it to loopback callers. client_host=None means an in-process caller (unit
    # tests, direct handle_request use), which is treated as loopback.
    if name == "agent_send" and client_host is not None and not is_loopback_host(client_host):
        logger.warning("agent_send refused (non-loopback caller)", client_host=client_host)
        return _jsonrpc_error(req_id, -32600, "agent_send is restricted to loopback clients")

    rpc_call = rpc_calls.get(name)
    if rpc_call is None:
        logger.warning("MCP tool not found", tool=name)
        return _jsonrpc_result_text(req_id, f"Tool not found: {name}")

    skill_info = app.state.skills_by_name.get(name)
    uses: list[str] = list(skill_info.uses) if skill_info is not None else []
    lifecycle = skill_info.lifecycle if skill_info is not None else "instant"
    cap_registry: CapabilityRegistry = app.state.cap_registry

    # A per-invocation token scopes the capability hold, so a stale invocation's
    # teardown can't release a hold that a newer same-tool invocation took over.
    acquire_token = uuid.uuid4().hex

    if uses:

        def _can_wait(holder: str) -> bool:
            # Wait only on instant holders; they release when they return.
            # Background holders run until explicitly stopped, so refuse instead
            # of blocking until the timeout.
            info = app.state.skills_by_name.get(holder)
            return (info.lifecycle if info is not None else "instant") != "background"

        # Run the (possibly blocking) acquire off the event loop so waiting for a
        # busy capability doesn't stall the server.
        conflict = await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: cap_registry.acquire(
                uses,
                tool_name=name,
                token=acquire_token,
                timeout=app.state.cap_acquire_timeout,
                can_wait=_can_wait,
            ),
        )
        if conflict is not None:
            cap, holder = conflict
            logger.info(
                "MCP tool refused (capability busy)",
                tool=name,
                cap=cap,
                holder=holder,
                snapshot=cap_registry.snapshot(),
            )
            # A background holder has a stop tool to call; an instant holder is
            # waited on above, so reaching here means it outlasted the timeout.
            holder_info = app.state.skills_by_name.get(holder)
            holder_lifecycle = holder_info.lifecycle if holder_info is not None else "instant"
            if holder_lifecycle == "background":
                advice = "Call the appropriate stop tool first, then retry."
            else:
                advice = "It is taking longer than expected; wait a moment and then retry."
            return _jsonrpc_result_text(
                req_id,
                f"Cannot start '{name}': capability '{cap}' is held by '{holder}'. {advice}",
            )

    logger.info("MCP tool call", tool=name, args=args, progress_token=progress_token)
    t0 = time.monotonic()

    # _mcp_context is a reserved kwarg consumed by the `@skill` wrapper; it never
    # reaches the user-visible skill signature. The acquire token rides along so
    # a background skill's ToolStream can stamp it on its stop frame for release.
    call_kwargs = dict(args)
    mcp_context: dict[str, Any] = {}
    if progress_token is not None:
        mcp_context["progress_token"] = progress_token
    if uses:
        mcp_context["acquire_token"] = acquire_token
    if mcp_context:
        call_kwargs["_mcp_context"] = mcp_context

    # Track whether we still hold the caps so we can release on failure even
    # for background skills. On success the background skill keeps them until
    # its tool-stream closes.
    caps_held = bool(uses)
    try:
        try:
            result = await asyncio.get_event_loop().run_in_executor(
                None, lambda: rpc_call(**call_kwargs)
            )
        except Exception as e:
            logger.exception("MCP tool error", tool=name, duration=f"{time.monotonic() - t0:.3f}s")
            return _jsonrpc_result_text(req_id, f"Error running tool '{name}': {e}")
        if lifecycle == "background":
            # Hand ownership of the caps off to the tool-stream lifecycle.
            caps_held = False
    finally:
        if caps_held:
            cap_registry.release_by_token(acquire_token)

    duration = f"{time.monotonic() - t0:.3f}s"
    response = str(result)[:200]

    if hasattr(result, "agent_encode"):
        logger.info("MCP tool done", tool=name, duration=duration, response=response)
        return _jsonrpc_result(req_id, {"content": result.agent_encode()})

    logger.info("MCP tool done", tool=name, duration=duration, response=response)
    return _jsonrpc_result_text(req_id, str(result))


async def handle_request(
    request: dict[str, Any],
    skills: list[SkillInfo],
    rpc_calls: dict[str, Any],
    *,
    client_host: str | None = None,
) -> dict[str, Any] | None:
    """Handle a single MCP JSON-RPC request.

    Returns None for JSON-RPC notifications (no ``id``), which must not
    receive a response. ``client_host`` is the TCP peer of the HTTP request;
    None means an in-process caller and is treated as loopback.
    """
    method = request.get("method", "")
    params = request.get("params", {}) or {}
    req_id = request.get("id")

    # JSON-RPC notifications have no "id" -- the server must not reply.
    if "id" not in request:
        return None

    if method == "initialize":
        return _handle_initialize(req_id)
    if method == "tools/list":
        return _handle_tools_list(req_id, skills)
    if method == "tools/call":
        return await _handle_tools_call(req_id, params, rpc_calls, client_host=client_host)
    return _jsonrpc_error(req_id, -32601, f"Unknown: {method}")


@app.post("/mcp")
async def mcp_endpoint(request: Request) -> Response:
    content_length = request.headers.get("content-length")
    if (
        content_length is not None
        and content_length.isdigit()
        and int(content_length) > MAX_MCP_BODY_BYTES
    ):
        return JSONResponse({"error": "payload too large"}, status_code=413)
    raw = await request.body()
    if len(raw) > MAX_MCP_BODY_BYTES:  # chunked uploads carry no Content-Length
        return JSONResponse({"error": "payload too large"}, status_code=413)
    try:
        body = json.loads(raw)
    except Exception:
        logger.exception("POST /mcp JSON parse failed")
        return JSONResponse(
            {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}},
            status_code=400,
        )
    client_host = request.client.host if request.client is not None else None
    result = await handle_request(
        body, request.app.state.skills, request.app.state.rpc_calls, client_host=client_host
    )
    if result is None:
        return Response(status_code=204)
    return JSONResponse(result)


def _sse_frame(data: dict[str, Any]) -> str:
    """Format a JSON-RPC message as an SSE ``event: message`` frame."""
    return f"event: message\ndata: {json.dumps(data)}\n\n"


def _fan_out_to_sse_queues(msg: dict[str, Any]) -> None:
    """LCM subscriber callback: forward a tool-stream frame to every active SSE client.

    Also releases capabilities held by a background skill when its tool-stream
    closes (signaled by a ``dimos/tool_stopped`` frame).
    """
    if msg.get("method") == tool_stream.TOOL_STREAM_STOPPED_METHOD:
        params = msg.get("params") or {}
        token = params.get("token")
        if token:
            released = app.state.cap_registry.release_by_token(token)
            if released:
                logger.info(
                    "Capabilities released on tool-stream stop",
                    holder=params.get("tool_name"),
                    token=token,
                    released=released,
                )
    loop = app.state.event_loop
    if loop is None:
        return
    for queue in list(app.state.sse_queues):
        try:
            asyncio.run_coroutine_threadsafe(queue.put(msg), loop)
        except RuntimeError:
            pass


@app.get("/mcp")
async def mcp_sse_endpoint() -> StreamingResponse:
    """Persistent server-to-client SSE channel for MCP notifications.

    This is the Streamable-HTTP transport's out-of-band channel for
    server-initiated messages.  Every tool-stream update is fanned out here,
    so the subscription is live for the full client session and independent
    of any particular ``tools/call`` request.
    """
    queue: asyncio.Queue[dict[str, Any] | None] = asyncio.Queue()
    # Remember the loop so the LCM subscriber (running on an LCM thread)
    # can schedule queue.put via run_coroutine_threadsafe.
    app.state.event_loop = asyncio.get_running_loop()
    app.state.sse_queues.append(queue)

    async def event_generator() -> AsyncGenerator[str, None]:
        try:
            # Initial comment flushes the response headers and unblocks
            # any synchronous client that's waiting on iter_lines().
            yield ": connected\n\n"
            while True:
                try:
                    msg = await asyncio.wait_for(queue.get(), timeout=_SSE_KEEPALIVE_INTERVAL)
                except asyncio.TimeoutError:
                    yield ": keepalive\n\n"
                    continue
                if msg is None:
                    return
                yield _sse_frame(msg)
        except asyncio.CancelledError:
            pass
        finally:
            try:
                app.state.sse_queues.remove(queue)
            except ValueError:
                pass

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "Connection": "keep-alive"},
    )


class McpServer(Module):
    _uvicorn_server: uvicorn.Server | None = None
    _serve_future: concurrent.futures.Future[None] | None = None
    _tool_stream_cleanup: Callable[[], None] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._start_server()
        self._tool_stream_cleanup = tool_stream.subscribe(_fan_out_to_sse_queues)

    @rpc
    def stop(self) -> None:
        if self._tool_stream_cleanup is not None:
            self._tool_stream_cleanup()
            self._tool_stream_cleanup = None
        for queue in list(app.state.sse_queues):
            try:
                queue.put_nowait(None)
            except Exception:
                pass
        app.state.sse_queues.clear()
        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True
            loop = self._loop
            if loop is not None and self._serve_future is not None:
                self._serve_future.result(timeout=5.0)
            self._uvicorn_server = None
            self._serve_future = None
        super().stop()

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        # TODO: this is a bit hacky, also not thread-safe
        assert self.rpc is not None
        app.state.skills = [
            skill_info for module in modules for skill_info in (module.get_skills() or [])
        ]
        app.state.skills_by_name = {s.func_name: s for s in app.state.skills}
        app.state.rpc_calls = {
            skill_info.func_name: RpcCall(
                None, self.rpc, skill_info.func_name, skill_info.class_name, []
            )
            for skill_info in app.state.skills
        }

    @skill
    def server_status(self) -> str:
        """Get MCP server status: main process PID, deployed modules, and skill count."""
        from dimos.core.run_registry import get_most_recent

        skills: list[SkillInfo] = app.state.skills
        modules = list(dict.fromkeys(s.class_name for s in skills))
        entry = get_most_recent()
        pid = entry.pid if entry else os.getpid()
        return json.dumps(
            {
                "pid": pid,
                "modules": modules,
                "skills": [s.func_name for s in skills],
            }
        )

    @skill
    def list_modules(self) -> str:
        """List deployed modules and their skills."""
        skills: list[SkillInfo] = app.state.skills
        modules: dict[str, list[str]] = {}
        for s in skills:
            modules.setdefault(s.class_name, []).append(s.func_name)
        return json.dumps({"modules": modules})

    @skill
    def agent_send(self, message: str) -> str:
        """Send a message to the running DimOS agent over the active transport."""
        if not message:
            raise ValueError("Message cannot be empty")
        transport = make_transport("/human_input")
        try:
            transport.start()
            transport.publish(message)
            return f"Message sent to agent: {message[:100]}"
        finally:
            transport.stop()

    def _start_server(self, port: int | None = None) -> None:
        from dimos.core.global_config import global_config

        _port = port if port is not None else global_config.mcp_port
        _host = global_config.listen_host
        _check_bind_allowed(_host, global_config.mcp_auth_token)
        config = uvicorn.Config(app, host=_host, port=_port, log_level="warning")
        self._uvicorn_server = uvicorn.Server(config)

        def _serve() -> None:
            assert self._uvicorn_server is not None
            asyncio.run(self._uvicorn_server.serve())

        self._serve_future = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="mcp-uvicorn"
        ).submit(_serve)
        logger.info("MCP server started", host=_host, port=_port)
