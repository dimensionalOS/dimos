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
from pathlib import Path
from unittest.mock import MagicMock

from dimos.core.module import SkillInfo
from dimos.protocol.mcp.mcp import MCPModule


def _make_mcp(skills: list[SkillInfo], call_results: dict[str, object]) -> MCPModule:
    """Create an MCPModule with pre-populated skills and mock RPC calls."""
    mcp = MCPModule.__new__(MCPModule)
    mcp._skills = skills
    mcp._rpc_calls = {}
    for skill in skills:
        mock_call = MagicMock()
        if skill.func_name in call_results:
            mock_call.return_value = call_results[skill.func_name]
        else:
            mock_call.return_value = None
        mcp._rpc_calls[skill.func_name] = mock_call
    return mcp


def test_unitree_blueprint_has_mcp() -> None:
    contents = Path(
        "dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_mcp.py"
    ).read_text()
    assert "agentic_mcp" in contents
    assert "MCPModule.blueprint()" in contents


def test_mcp_module_request_flow() -> None:
    schema = json.dumps(
        {
            "type": "object",
            "description": "Add two numbers",
            "properties": {"x": {"type": "integer"}, "y": {"type": "integer"}},
            "required": ["x", "y"],
        }
    )
    skills = [SkillInfo(class_name="TestSkills", func_name="add", args_schema=schema)]

    mcp = _make_mcp(skills, {"add": 5})

    response = asyncio.run(mcp._handle_request({"method": "tools/list", "id": 1}))
    assert response["result"]["tools"][0]["name"] == "add"
    assert response["result"]["tools"][0]["description"] == "Add two numbers"

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


def test_mcp_module_handles_errors() -> None:
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [
        SkillInfo(class_name="TestSkills", func_name="ok_skill", args_schema=schema),
        SkillInfo(class_name="TestSkills", func_name="fail_skill", args_schema=schema),
    ]

    mcp = _make_mcp(skills, {"ok_skill": "done"})
    mcp._rpc_calls["fail_skill"] = MagicMock(side_effect=RuntimeError("boom"))

    # All skills listed
    response = asyncio.run(mcp._handle_request({"method": "tools/list", "id": 1}))
    tool_names = {tool["name"] for tool in response["result"]["tools"]}
    assert "ok_skill" in tool_names
    assert "fail_skill" in tool_names

    # Error skill returns error text
    response = asyncio.run(
        mcp._handle_request(
            {"method": "tools/call", "id": 2, "params": {"name": "fail_skill", "arguments": {}}}
        )
    )
    assert "Error:" in response["result"]["content"][0]["text"]
    assert "boom" in response["result"]["content"][0]["text"]

    # Unknown skill returns not found
    response = asyncio.run(
        mcp._handle_request(
            {"method": "tools/call", "id": 3, "params": {"name": "no_such", "arguments": {}}}
        )
    )
    assert "not found" in response["result"]["content"][0]["text"].lower()


def test_mcp_module_initialize_and_unknown() -> None:
    mcp = _make_mcp([], {})

    response = asyncio.run(mcp._handle_request({"method": "initialize", "id": 1}))
    assert response["result"]["serverInfo"]["name"] == "dimensional"

    response = asyncio.run(mcp._handle_request({"method": "unknown/method", "id": 2}))
    assert response["error"]["code"] == -32601


def test_mcp_module_invalid_tool_name() -> None:
    mcp = _make_mcp([], {})

    response = asyncio.run(
        mcp._handle_request(
            {"method": "tools/call", "id": 1, "params": {"name": 123, "arguments": {}}}
        )
    )
    assert response["error"]["code"] == -32602
