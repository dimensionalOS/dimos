# Copyright 2025-2026 Dimensional Inc.
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
from collections.abc import AsyncGenerator, Callable
import concurrent.futures
import json
import os
import time
from typing import TYPE_CHECKING, Any

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from starlette.requests import Request
from starlette.responses import Response, StreamingResponse
import uvicorn

from dimos.agents.annotation import skill
from dimos.agents.mcp import tool_stream
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.module import SkillInfo

logger = setup_logger()


_SSE_KEEPALIVE_INTERVAL = 20.0  # seconds

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["POST", "GET"],
    allow_headers=["*"],
)
app.state.skills = []
app.state.rpc_calls = {}
app.state.sse_queues = []
app.state.event_loop = None


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
        tools.append(tool)

    return _jsonrpc_result(req_id, {"tools": tools})


async def _handle_tools_call(
    req_id: Any, params: dict[str, Any], rpc_calls: dict[str, Any]
) -> dict[str, Any]:
    name = params.get("name", "")
    args: dict[str, Any] = params.get("arguments") or {}
    meta = params.get("_meta") or {}
    progress_token = meta.get("progressToken")

    rpc_call = rpc_calls.get(name)
    if rpc_call is None:
        logger.warning("MCP tool not found", tool=name)
        return _jsonrpc_result_text(req_id, f"Tool not found: {name}")

    logger.info("MCP tool call", tool=name, args=args, progress_token=progress_token)
    t0 = time.monotonic()

    # _mcp_context is a reserved kwarg consumed by the `@skill` wrapper;
    # it never reaches the user-visible skill signature.
    call_kwargs = dict(args)
    if progress_token is not None:
        call_kwargs["_mcp_context"] = {"progress_token": progress_token}

    try:
        result = await asyncio.get_event_loop().run_in_executor(
            None, lambda: rpc_call(**call_kwargs)
        )
    except Exception as e:
        logger.exception("MCP tool error", tool=name, duration=f"{time.monotonic() - t0:.3f}s")
        return _jsonrpc_result_text(req_id, f"Error running tool '{name}': {e}")

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
) -> dict[str, Any] | None:
    """Handle a single MCP JSON-RPC request.

    Returns None for JSON-RPC notifications (no ``id``), which must not
    receive a response.
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
        return await _handle_tools_call(req_id, params, rpc_calls)
    return _jsonrpc_error(req_id, -32601, f"Unknown: {method}")


@app.post("/mcp")
async def mcp_endpoint(request: Request) -> Response:
    raw = await request.body()
    try:
        body = json.loads(raw)
    except Exception:
        logger.exception("POST /mcp JSON parse failed")
        return JSONResponse(
            {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}},
            status_code=400,
        )

    result = await handle_request(body, request.app.state.skills, request.app.state.rpc_calls)

    if result is None:
        return Response(status_code=204)
    return JSONResponse(result)


def _sse_frame(data: dict[str, Any]) -> str:
    """Format a JSON-RPC message as an SSE ``event: message`` frame."""
    return f"event: message\ndata: {json.dumps(data)}\n\n"


def _fan_out_to_sse_queues(msg: dict[str, Any]) -> None:
    """LCM subscriber callback: forward a tool-stream frame to every active SSE client."""
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
        """Send a message to the running DimOS agent via LCM."""
        if not message:
            raise ValueError("Message cannot be empty")

        from dimos.core.transport import pLCMTransport

        transport: pLCMTransport[str] = pLCMTransport("/human_input")
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
        config = uvicorn.Config(app, host=_host, port=_port, log_level="warning", access_log=False)
        server = uvicorn.Server(config)
        self._uvicorn_server = server
        loop = self._loop
        assert loop is not None
        self._serve_future = asyncio.run_coroutine_threadsafe(server.serve(), loop)
