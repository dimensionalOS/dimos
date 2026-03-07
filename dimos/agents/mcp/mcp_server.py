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
import json
from typing import TYPE_CHECKING, Any

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from starlette.responses import Response
import uvicorn

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


from starlette.requests import Request  # noqa: TC002

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient

if TYPE_CHECKING:
    import concurrent.futures

    from dimos.core.module import SkillInfo


app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["POST"],
    allow_headers=["*"],
)
app.state.skills = []
app.state.rpc_calls = {}


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
            "capabilities": {"tools": {}},
            "serverInfo": {"name": "dimensional", "version": "1.0.0"},
        },
    )


def _handle_tools_list(req_id: Any, skills: list[SkillInfo]) -> dict[str, Any]:
    tools = []

    for skill in skills:
        schema = json.loads(skill.args_schema)
        description = schema.pop("description", None)
        schema.pop("title", None)
        tool = {"name": skill.func_name, "inputSchema": schema}
        if description:
            tool["description"] = description
        tools.append(tool)

    return _jsonrpc_result(req_id, {"tools": tools})


async def _handle_tools_call(
    req_id: Any, params: dict[str, Any], rpc_calls: dict[str, Any]
) -> dict[str, Any]:
    name = params.get("name", "")
    args: dict[str, Any] = params.get("arguments") or {}

    rpc_call = rpc_calls.get(name)
    if rpc_call is None:
        return _jsonrpc_result_text(req_id, f"Tool not found: {name}")

    try:
        result = await asyncio.get_event_loop().run_in_executor(None, lambda: rpc_call(**args))
    except Exception as e:
        logger.exception("Error running tool", tool_name=name, exc_info=True)
        return _jsonrpc_result_text(req_id, f"Error running tool '{name}': {e}")

    if result is None:
        return _jsonrpc_result_text(req_id, "It has started. You will be updated later.")

    if hasattr(result, "agent_encode"):
        return _jsonrpc_result(req_id, {"content": result.agent_encode()})

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

    # JSON-RPC notifications have no "id" – the server must not reply.
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


class McpServer(Module):
    def __init__(self) -> None:
        super().__init__()
        self._uvicorn_server: uvicorn.Server | None = None
        self._serve_future: concurrent.futures.Future[None] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._start_server()

    @rpc
    def stop(self) -> None:
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
        assert self.rpc is not None
        app.state.skills = [skill for module in modules for skill in (module.get_skills() or [])]
        app.state.rpc_calls = {
            skill.func_name: RpcCall(None, self.rpc, skill.func_name, skill.class_name, [])
            for skill in app.state.skills
        }

    def _start_server(self, port: int | None = None) -> None:
        from dimos.core.global_config import global_config

        _port = port if port is not None else global_config.mcp_port
        _host = global_config.mcp_host
        config = uvicorn.Config(app, host=_host, port=_port, log_level="info")
        server = uvicorn.Server(config)
        self._uvicorn_server = server
        loop = self._loop
        assert loop is not None
        self._serve_future = asyncio.run_coroutine_threadsafe(server.serve(), loop)
