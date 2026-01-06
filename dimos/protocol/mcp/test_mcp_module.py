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
from pathlib import Path

from dimos.protocol.mcp.mcp import MCPModule
from dimos.protocol.skill.coordinator import SkillStateEnum


def test_unitree_blueprint_has_mcp() -> None:
    contents = Path("dimos/robot/unitree_webrtc/unitree_go2_blueprints.py").read_text()
    assert "agentic_mcp" in contents
    assert "MCPModule.blueprint()" in contents


def test_mcp_module_request_flow() -> None:
    class DummySkill:
        def __init__(self) -> None:
            self.name = "add"
            self.hide_skill = False
            self.schema = {"function": {"description": "", "parameters": {"type": "object"}}}

    class DummyState:
        def __init__(self, content: int) -> None:
            self.state = SkillStateEnum.completed
            self._content = content

        def content(self) -> int:
            return self._content

    class DummyCoordinator:
        def __init__(self) -> None:
            self._skill_state: dict[str, DummyState] = {}

        def skills(self) -> dict[str, DummySkill]:
            return {"add": DummySkill()}

        def call_skill(self, call_id: str, _name: str, args: dict[str, int]) -> None:
            self._skill_state[call_id] = DummyState(args["x"] + args["y"])

        async def wait_for_updates(self) -> bool:
            return True

    mcp = MCPModule.__new__(MCPModule)
    mcp.coordinator = DummyCoordinator()

    response = asyncio.run(mcp._handle_request({"method": "tools/list", "id": 1}))
    assert response["result"]["tools"][0]["name"] == "add"

    response = asyncio.run(
        mcp._handle_request(
            {
                "method": "tools/call",
                "id": 2,
                "params": {"name": "add", "arguments": {"x": 2, "y": 3}},
            }
        )
    )
    assert response["result"]["content"][0]["text"] == "5"
