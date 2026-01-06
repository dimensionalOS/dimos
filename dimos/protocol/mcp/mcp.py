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
import uuid

from dimos.core import Module, rpc
from dimos.protocol.skill.coordinator import SkillCoordinator, SkillStateEnum

if TYPE_CHECKING:
    from dimos.protocol.skill.coordinator import SkillState


class MCPModule(Module):
    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.coordinator = SkillCoordinator()
        self._server: asyncio.AbstractServer | None = None
        self._server_future: asyncio.Future | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.coordinator.start()
        self._start_server()

    @rpc
    def stop(self) -> None:
        if self._server:
            asyncio.run_coroutine_threadsafe(self._stop_server(), self._loop).result()
        if self._server_future and not self._server_future.done():
            self._server_future.cancel()
        self.coordinator.stop()
        super().stop()

    @rpc
    def register_skills(self, container) -> None:  # type: ignore[no-untyped-def]
        self.coordinator.register_skills(container)

    async def _stop_server(self) -> None:
        if not self._server:
            return
        self._server.close()
        await self._server.wait_closed()
        self._server = None

    def _start_server(self, port: int = 9990) -> None:
        async def handle_client(reader, writer) -> None:  # type: ignore[no-untyped-def]
            while True:
                data = await reader.readline()
                if not data:
                    break
                request = json.loads(data.decode())
                response = await self._handle_request(request)
                writer.write(json.dumps(response).encode() + b"\n")
                await writer.drain()
            writer.close()

        async def start_server() -> None:
            server = await asyncio.start_server(handle_client, "0.0.0.0", port)
            self._server = server
            await server.serve_forever()

        self._server_future = asyncio.run_coroutine_threadsafe(start_server(), self._loop)

    async def _handle_request(self, request: dict[str, Any]) -> dict[str, Any]:
        method = request.get("method", "")
        params = request.get("params", {}) or {}
        req_id = request.get("id")
        if method == "initialize":
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "dimensional", "version": "1.0.0"},
                },
            }
        if method == "tools/list":
            tools = [
                {
                    "name": c.name,
                    "description": c.schema.get("function", {}).get("description", ""),
                    "inputSchema": c.schema.get("function", {}).get("parameters", {}),
                }
                for c in self.coordinator.skills().values()
                if not c.hide_skill
            ]
            return {"jsonrpc": "2.0", "id": req_id, "result": {"tools": tools}}
        if method == "tools/call":
            name, args = params.get("name"), params.get("arguments") or {}
            call_id = str(uuid.uuid4())
            self.coordinator.call_skill(call_id, name, args)
            result: SkillState | None = self.coordinator._skill_state.get(call_id)
            try:
                await asyncio.wait_for(self.coordinator.wait_for_updates(), timeout=5.0)
            except asyncio.TimeoutError:
                pass
            if result is None:
                text = "Skill not found"
            elif result.state == SkillStateEnum.completed:
                text = str(result.content()) if result.content() else "Completed"
            elif result.state == SkillStateEnum.error:
                text = f"Error: {result.content()}"
            else:
                text = f"Started ({result.state.name})"
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
