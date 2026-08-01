# Copyright 2026 Dimensional Inc.
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

import json

import pytest

from dimos.agents.mcp.mcp_server import _handle_tools_list
from dimos.benchmark.agent_eval.pi_adapter import (
    PI_TOOL_NAMES,
    CodePolicyCallLog,
    PythonExecBroker,
    ToolInventoryError,
    credential_binding_sha256,
    inspect_python_exec_inventory,
    wait_for_python_exec,
)
from dimos.core.module import SkillInfo

ATTEMPT_ID = "attempt_" + "a" * 32
PI_SESSION_ID = "pi_session_" + "b" * 32
POLICY_SESSION_ID = "code_policy_session_" + "c" * 32


def _python_exec_tool() -> dict[str, object]:
    return {
        "name": "python_exec",
        "description": (
            "Execute one synchronous Python program in the persistent policy session.\n\n"
            "The trusted, unsandboxed session preloads `app` for deployed DimOS RPCs."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "code": {"title": "Code", "type": "string"},
                "timeout_s": {
                    "default": 110.0,
                    "title": "Timeout S",
                    "type": "number",
                },
            },
            "required": ["code"],
        },
    }


class FakeMcp:
    def __init__(
        self,
        tools: list[dict[str, object]] | None = None,
        result: dict[str, object] | None = None,
    ) -> None:
        self.tools = tools or [_python_exec_tool()]
        self.result = result or {"content": [{"type": "text", "text": "done"}]}
        self.calls: list[tuple[str, dict[str, object]]] = []
        self.ready = True

    def wait_for_ready(self, timeout: float) -> bool:
        assert timeout > 0
        return self.ready

    def list_tools(self) -> list[dict[str, object]]:
        return self.tools

    def call_tool(self, name: str, arguments: dict[str, object] | None = None) -> dict[str, object]:
        self.calls.append((name, arguments or {}))
        return self.result


def test_inventory_retains_additional_tools_but_admits_only_python_exec() -> None:
    tools = [
        _python_exec_tool(),
        {
            "name": "move",
            "description": "Direct robot motion",
            "inputSchema": {"type": "object", "properties": {}},
        },
    ]

    receipt = inspect_python_exec_inventory("http://localhost/mcp", tools)

    assert [tool["name"] for tool in receipt.observed_tools] == ["python_exec", "move"]
    assert PI_TOOL_NAMES == ("python_exec",)


@pytest.mark.parametrize(
    "tools",
    [
        [],
        [_python_exec_tool(), _python_exec_tool()],
        [{**_python_exec_tool(), "description": "changed"}],
        [
            {
                **_python_exec_tool(),
                "inputSchema": {
                    "type": "object",
                    "properties": {"code": {"type": "string"}},
                    "required": ["code"],
                },
            }
        ],
    ],
)
def test_inventory_rejects_missing_duplicate_or_changed_tool(
    tools: list[dict[str, object]],
) -> None:
    with pytest.raises(ToolInventoryError):
        inspect_python_exec_inventory("http://localhost/mcp", tools)


def test_readiness_timeout_is_infrastructure_failure() -> None:
    mcp = FakeMcp()
    mcp.ready = False

    with pytest.raises(TimeoutError):
        wait_for_python_exec("http://localhost/mcp", mcp, 0.1)


def test_broker_forwards_one_tool_and_records_both_sessions(tmp_path) -> None:
    mcp = FakeMcp()
    path = tmp_path / "code-policy-calls.jsonl"
    with CodePolicyCallLog(path) as call_log:
        broker = PythonExecBroker(
            attempt_id=ATTEMPT_ID,
            pi_session_id=PI_SESSION_ID,
            code_policy_session_id=POLICY_SESSION_ID,
            mcp=mcp,
            call_log=call_log,
        )
        result = broker.request("python_exec", {"code": "print('hello')"})

    record = json.loads(path.read_text())
    assert result == mcp.result
    assert mcp.calls == [("python_exec", {"code": "print('hello')", "timeout_s": 110.0})]
    assert record["attempt_id"] == ATTEMPT_ID
    assert record["pi_session_id"] == PI_SESSION_ID
    assert record["code_policy_session_id"] == POLICY_SESSION_ID
    assert record["ok"] is True


def test_broker_rejects_every_other_tool_without_forwarding(tmp_path) -> None:
    mcp = FakeMcp()
    with CodePolicyCallLog(tmp_path / "calls.jsonl") as call_log:
        broker = PythonExecBroker(
            attempt_id=ATTEMPT_ID,
            pi_session_id=PI_SESSION_ID,
            code_policy_session_id=POLICY_SESSION_ID,
            mcp=mcp,
            call_log=call_log,
        )
        with pytest.raises(PermissionError):
            broker.request("move", {"x": 1})
    assert mcp.calls == []


def test_credentials_do_not_enter_records_or_diagnostics(tmp_path) -> None:
    secret = "sk-super-secret-value"
    digest = credential_binding_sha256("environment", "OPENAI_API_KEY", secret)
    mcp = FakeMcp(result={"content": [{"type": "text", "text": "safe"}]})
    path = tmp_path / "calls.jsonl"
    with CodePolicyCallLog(path) as call_log:
        broker = PythonExecBroker(
            attempt_id=ATTEMPT_ID,
            pi_session_id=PI_SESSION_ID,
            code_policy_session_id=POLICY_SESSION_ID,
            mcp=mcp,
            call_log=call_log,
        )
        broker.request("python_exec", {"code": "1 + 1"})

    retained = path.read_text() + digest
    assert secret not in retained
    assert digest == credential_binding_sha256("environment", "OPENAI_API_KEY", secret)


def test_inventory_rejects_non_json_tool() -> None:
    skill = SkillInfo(
        class_name="Bad",
        func_name="python_exec",
        args_schema=json.dumps({"type": "object"}),
    )
    tool = _handle_tools_list(1, [skill])["result"]["tools"][0]
    tool["not_json"] = object()
    with pytest.raises(ValueError, match="strict JSON"):
        inspect_python_exec_inventory("http://localhost/mcp", [tool])
