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
from unittest.mock import MagicMock

from dimos.agents.mcp.mcp_server import handle_request
from dimos.core.module import SkillInfo


def _make_rpc_calls(
    skills: list[SkillInfo], call_results: dict[str, object]
) -> dict[str, MagicMock]:
    """Create mock RPC calls for the given skills."""
    rpc_calls: dict[str, MagicMock] = {}
    for skill in skills:
        mock_call = MagicMock()
        if skill.func_name in call_results:
            mock_call.return_value = call_results[skill.func_name]
        else:
            mock_call.return_value = None
        rpc_calls[skill.func_name] = mock_call
    return rpc_calls


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
    rpc_calls = _make_rpc_calls(skills, {"add": 5})

    response = asyncio.run(handle_request({"method": "tools/list", "id": 1}, skills, rpc_calls))
    assert response["result"]["tools"][0]["name"] == "add"
    assert response["result"]["tools"][0]["description"] == "Add two numbers"

    response = asyncio.run(
        handle_request(
            {
                "method": "tools/call",
                "id": 2,
                "params": {"name": "add", "arguments": {"x": 2, "y": 3}},
            },
            skills,
            rpc_calls,
        )
    )
    assert response["result"]["content"][0]["text"] == "5"
    rpc_calls["add"].assert_called_once_with(x=2, y=3)


def test_mcp_module_injects_progress_token_as_mcp_context() -> None:
    """When the client sends `_meta.progressToken`, the RPC call receives it as
    an `_mcp_context` kwarg so the `@skill` wrapper can stash it in the
    thread-local that `ToolStream` reads on construction."""
    schema = json.dumps({"type": "object", "properties": {"x": {"type": "integer"}}})
    skills = [SkillInfo(class_name="TestSkills", func_name="echo", args_schema=schema)]
    rpc_calls = _make_rpc_calls(skills, {"echo": "ok"})

    asyncio.run(
        handle_request(
            {
                "method": "tools/call",
                "id": 7,
                "params": {
                    "name": "echo",
                    "arguments": {"x": 42},
                    "_meta": {"progressToken": "pt-srv-test"},
                },
            },
            skills,
            rpc_calls,
        )
    )

    rpc_calls["echo"].assert_called_once_with(x=42, _mcp_context={"progress_token": "pt-srv-test"})


def test_mcp_module_without_progress_token_does_not_inject_context() -> None:
    """Absence of `progressToken` preserves the legacy call shape. The skill
    sees only the arguments it declared."""
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="TestSkills", func_name="ping", args_schema=schema)]
    rpc_calls = _make_rpc_calls(skills, {"ping": "pong"})

    asyncio.run(
        handle_request(
            {
                "method": "tools/call",
                "id": 8,
                "params": {"name": "ping", "arguments": {}},
            },
            skills,
            rpc_calls,
        )
    )

    rpc_calls["ping"].assert_called_once_with()


def test_mcp_module_handles_errors() -> None:
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [
        SkillInfo(class_name="TestSkills", func_name="ok_skill", args_schema=schema),
        SkillInfo(class_name="TestSkills", func_name="fail_skill", args_schema=schema),
    ]

    rpc_calls = _make_rpc_calls(skills, {"ok_skill": "done"})
    rpc_calls["fail_skill"] = MagicMock(side_effect=RuntimeError("boom"))

    # All skills listed
    response = asyncio.run(handle_request({"method": "tools/list", "id": 1}, skills, rpc_calls))
    tool_names = {tool["name"] for tool in response["result"]["tools"]}
    assert "ok_skill" in tool_names
    assert "fail_skill" in tool_names

    # Error skill returns error text
    response = asyncio.run(
        handle_request(
            {"method": "tools/call", "id": 2, "params": {"name": "fail_skill", "arguments": {}}},
            skills,
            rpc_calls,
        )
    )
    assert "Error running tool" in response["result"]["content"][0]["text"]
    assert "boom" in response["result"]["content"][0]["text"]

    # Unknown skill returns not found
    response = asyncio.run(
        handle_request(
            {"method": "tools/call", "id": 3, "params": {"name": "no_such", "arguments": {}}},
            skills,
            rpc_calls,
        )
    )
    assert "not found" in response["result"]["content"][0]["text"].lower()


def test_mcp_module_initialize_and_unknown() -> None:
    response = asyncio.run(handle_request({"method": "initialize", "id": 1}, [], {}))
    assert response["result"]["serverInfo"]["name"] == "dimensional"

    response = asyncio.run(handle_request({"method": "unknown/method", "id": 2}, [], {}))
    assert response["error"]["code"] == -32601
