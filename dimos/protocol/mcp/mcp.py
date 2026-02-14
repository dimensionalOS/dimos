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

from dimos.core import Module, rpc
from dimos.core.rpc_client import RpcCall, RPCClient

if TYPE_CHECKING:
    from dimos.core.module import SkillInfo


class MCPModule(Module):
    _skills: list[SkillInfo]
    _rpc_calls: dict[str, RpcCall]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._skills = []
        self._rpc_calls = {}
        self._server: asyncio.AbstractServer | None = None
        self._server_future: object | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._start_server()

    @rpc
    def stop(self) -> None:
        if self._server:
            self._server.close()
            loop = self._loop
            assert loop is not None
            asyncio.run_coroutine_threadsafe(self._server.wait_closed(), loop).result()
            self._server = None
        if self._server_future and hasattr(self._server_future, "cancel"):
            self._server_future.cancel()
        super().stop()

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        assert self.rpc is not None
        self._skills = [skill for module in modules for skill in (module.get_skills() or [])]
        self._rpc_calls = {
            skill.func_name: RpcCall(None, self.rpc, skill.func_name, skill.class_name, [])
            for skill in self._skills
        }

    def _start_server(self, port: int = 9990) -> None:
        async def handle_client(reader, writer) -> None:  # type: ignore[no-untyped-def]
            while True:
                if not (data := await reader.readline()):
                    break
                response = await self._handle_request(json.loads(data.decode()))
                writer.write(json.dumps(response).encode() + b"\n")
                await writer.drain()
            writer.close()

        async def start_server() -> None:
            self._server = await asyncio.start_server(handle_client, "0.0.0.0", port)
            await self._server.serve_forever()

        loop = self._loop
        assert loop is not None
        self._server_future = asyncio.run_coroutine_threadsafe(start_server(), loop)

    async def _handle_request(self, request: dict[str, Any]) -> dict[str, Any]:
        method = request.get("method", "")
        params = request.get("params", {}) or {}
        req_id = request.get("id")
        if method == "initialize":
            init_result = {
                "protocolVersion": "2024-11-05",
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "dimensional", "version": "1.0.0"},
            }
            return {"jsonrpc": "2.0", "id": req_id, "result": init_result}
        if method == "tools/list":
            tools = []
            for skill in self._skills:
                schema = json.loads(skill.args_schema)
                tools.append(
                    {
                        "name": skill.func_name,
                        "description": schema.get("description", ""),
                        "inputSchema": schema,
                    }
                )
            return {"jsonrpc": "2.0", "id": req_id, "result": {"tools": tools}}
        if method == "tools/call":
            name = params.get("name")
            args = params.get("arguments") or {}
            if not isinstance(name, str):
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "error": {"code": -32602, "message": "Missing or invalid tool name"},
                }
            if not isinstance(args, dict):
                args = {}
            rpc_call = self._rpc_calls.get(name)
            if rpc_call is None:
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {"content": [{"type": "text", "text": "Skill not found"}]},
                }
            try:
                result = await asyncio.get_event_loop().run_in_executor(
                    None, lambda: rpc_call(**args)
                )
                text = str(result) if result is not None else "Completed"
            except Exception as e:
                text = f"Error: {e}"
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {"content": [{"type": "text", "text": text}]},
            }
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": -32601, "message": f"Unknown: {method}"},
        }
