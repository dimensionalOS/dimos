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
import os
from pathlib import Path
import socket
import subprocess
import sys

import pytest

from dimos.protocol.mcp.mcp import MCPModule
from dimos.protocol.skill.coordinator import SkillStateEnum
from dimos.protocol.skill.skill import SkillContainer, skill


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


def test_mcp_module_handles_hidden_and_errors() -> None:
    class DummySkill:
        def __init__(self, name: str, hide_skill: bool) -> None:
            self.name = name
            self.hide_skill = hide_skill
            self.schema = {"function": {"description": "", "parameters": {"type": "object"}}}

    class DummyState:
        def __init__(self, state: SkillStateEnum, content: str | None) -> None:
            self.state = state
            self._content = content

        def content(self) -> str | None:
            return self._content

    class DummyCoordinator:
        def __init__(self) -> None:
            self._skill_state: dict[str, DummyState] = {}
            self._skills = {
                "visible": DummySkill("visible", False),
                "hidden": DummySkill("hidden", True),
                "fail": DummySkill("fail", False),
            }

        def skills(self) -> dict[str, DummySkill]:
            return self._skills

        def call_skill(self, call_id: str, name: str, _args: dict[str, int]) -> None:
            if name == "fail":
                self._skill_state[call_id] = DummyState(SkillStateEnum.error, "boom")
            elif name in self._skills:
                self._skill_state[call_id] = DummyState(SkillStateEnum.running, None)

        async def wait_for_updates(self) -> bool:
            return True

    mcp = MCPModule.__new__(MCPModule)
    mcp.coordinator = DummyCoordinator()

    response = asyncio.run(mcp._handle_request({"method": "tools/list", "id": 1}))
    tool_names = {tool["name"] for tool in response["result"]["tools"]}
    assert "visible" in tool_names
    assert "hidden" not in tool_names

    response = asyncio.run(
        mcp._handle_request(
            {"method": "tools/call", "id": 2, "params": {"name": "fail", "arguments": {}}}
        )
    )
    assert "Error:" in response["result"]["content"][0]["text"]


@pytest.mark.integration
def test_mcp_end_to_end_lcm_bridge() -> None:
    try:
        import lcm  # type: ignore[import-untyped]

        lcm.LCM()
    except Exception as exc:
        if os.environ.get("CI"):
            pytest.fail(f"LCM unavailable for MCP end-to-end test: {exc}")
        pytest.skip("LCM unavailable for MCP end-to-end test.")

    try:
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).close()
    except PermissionError:
        if os.environ.get("CI"):
            pytest.fail("Socket creation not permitted in CI environment.")
        pytest.skip("Socket creation not permitted in this environment.")

    class TestSkills(SkillContainer):
        @skill()
        def add(self, x: int, y: int) -> int:
            return x + y

    mcp = MCPModule()
    mcp.start()

    try:
        mcp.register_skills(TestSkills())

        env = {"MCP_HOST": "127.0.0.1", "MCP_PORT": "9990"}
        proc = subprocess.Popen(
            [sys.executable, "-m", "dimos.protocol.mcp"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={**os.environ, **env},
            text=True,
        )
        try:
            request = {"jsonrpc": "2.0", "id": 1, "method": "tools/list"}
            proc.stdin.write(json.dumps(request) + "\n")
            proc.stdin.flush()
            stdout = proc.stdout.readline()
            assert '"tools"' in stdout
            assert '"add"' in stdout
        finally:
            proc.terminate()
            proc.wait(timeout=5)

        proc = subprocess.Popen(
            [sys.executable, "-m", "dimos.protocol.mcp"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={**os.environ, **env},
            text=True,
        )
        try:
            request = {
                "jsonrpc": "2.0",
                "id": 2,
                "method": "tools/call",
                "params": {"name": "add", "arguments": {"x": 2, "y": 3}},
            }
            proc.stdin.write(json.dumps(request) + "\n")
            proc.stdin.flush()
            stdout = proc.stdout.readline()
            assert "5" in stdout
        finally:
            proc.terminate()
            proc.wait(timeout=5)
    finally:
        mcp.stop()
