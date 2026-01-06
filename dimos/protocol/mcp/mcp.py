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
from typing import Any
import uuid

from dimos.core import Module, rpc
from dimos.protocol.skill.coordinator import SkillStateEnum


class MCPModule(Module):
    rpc_calls = (
        "Agent.list_skills",
        "Agent.call_skill",
        "Agent.wait_for_skill_updates",
        "Agent.get_skill_state",
    )

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._list_skills = self._call_skill = self._wait_for_updates = self._get_skill_state = None

    @rpc
    def start(self) -> None:
        super().start()
        (
            self._list_skills,
            self._call_skill,
            self._wait_for_updates,
            self._get_skill_state,
        ) = self.get_rpc_calls(*self.rpc_calls)
        self._start_server()

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
            await server.serve_forever()

        asyncio.run_coroutine_threadsafe(start_server(), self._loop)

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
            return {"jsonrpc": "2.0", "id": req_id, "result": {"tools": self._list_skills()}}
        if method == "tools/call":
            name, args = params.get("name"), params.get("arguments") or {}
            call_id = str(uuid.uuid4())
            self._call_skill(call_id, name, args)
            try:
                await asyncio.wait_for(asyncio.to_thread(self._wait_for_updates, 5.0), timeout=6.0)
            except asyncio.TimeoutError:
                pass
            result = self._get_skill_state(call_id)
            if not result:
                text = "Skill not found"
            elif result["state"] == SkillStateEnum.completed.name:
                text = str(result["content"]) if result["content"] else "Completed"
            elif result["state"] == SkillStateEnum.error.name:
                text = f"Error: {result['content']}"
            else:
                text = f"Started ({result['state']})"
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
