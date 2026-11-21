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
import threading
from unittest.mock import MagicMock

import pytest
from starlette.datastructures import State

from dimos.agents.capabilities import CapabilityRegistry
from dimos.agents.mcp.mcp_server import app, handle_request
from dimos.agents.skill_result import SkillResult
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
    assert response is not None
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
    assert response is not None
    assert response["result"]["content"][0]["text"] == "5"
    rpc_calls["add"].assert_called_once_with(x=2, y=3, _mcp_context={"meta": {}})


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

    rpc_calls["echo"].assert_called_once_with(
        x=42,
        _mcp_context={"meta": {"progressToken": "pt-srv-test"}, "progress_token": "pt-srv-test"},
    )


def test_mcp_module_without_meta_passes_empty_meta() -> None:
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

    rpc_calls["ping"].assert_called_once_with(_mcp_context={"meta": {}})


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
    assert response is not None
    tool_names = {tool["name"] for tool in response["result"]["tools"]}
    assert "ok_skill" in tool_names
    assert "fail_skill" in tool_names

    # A raised skill error is a failed tool call carrying the exception text.
    response = asyncio.run(
        handle_request(
            {"method": "tools/call", "id": 2, "params": {"name": "fail_skill", "arguments": {}}},
            skills,
            rpc_calls,
        )
    )
    assert response is not None
    assert response["result"]["isError"] is True
    text = response["result"]["content"][0]["text"]
    assert "Error running tool 'fail_skill'" in text
    assert "RuntimeError: boom" in text

    # A skill that returns is not flagged.
    response = asyncio.run(
        handle_request(
            {"method": "tools/call", "id": 4, "params": {"name": "ok_skill", "arguments": {}}},
            skills,
            rpc_calls,
        )
    )
    assert response is not None
    assert "isError" not in response["result"]
    assert response["result"]["content"][0]["text"] == "done"

    # Unknown skill returns not found
    response = asyncio.run(
        handle_request(
            {"method": "tools/call", "id": 3, "params": {"name": "no_such", "arguments": {}}},
            skills,
            rpc_calls,
        )
    )
    assert response is not None
    assert response["result"]["isError"] is True
    assert "not found" in response["result"]["content"][0]["text"].lower()


def test_skill_result_without_strict_json_answers_with_text_only() -> None:
    """Strict JSON has no Infinity, so the endpoint could not render it as structuredContent."""
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="TestSkills", func_name="measure", args_schema=schema)]

    def measure(**_: object) -> SkillResult:
        return SkillResult("distance inf", metadata={"distance": float("inf")})

    response = asyncio.run(
        handle_request(
            {"method": "tools/call", "id": 5, "params": {"name": "measure", "arguments": {}}},
            skills,
            {"measure": measure},
        )
    )
    assert response is not None
    assert response["result"] == {
        "content": [
            {
                "type": "text",
                "text": '{"message": "distance inf", "duration_ms": 0.0, "metadata": {"distance": Infinity}}',
            }
        ]
    }


def test_mcp_module_initialize_and_unknown() -> None:
    response = asyncio.run(handle_request({"method": "initialize", "id": 1}, [], {}))
    assert response is not None
    assert response["result"]["serverInfo"]["name"] == "dimensional"

    response = asyncio.run(handle_request({"method": "unknown/method", "id": 2}, [], {}))
    assert response is not None
    assert response["error"]["code"] == -32601


def test_mcp_module_injects_acquire_token_for_capability_skill() -> None:
    """A capability-using skill receives a per-invocation `acquire_token` in its
    `_mcp_context`, and the registry records the hold under the tool name. The
    token lets the skill's stop frame release exactly this invocation's hold."""
    schema = json.dumps({"type": "object", "properties": {}})
    mover = SkillInfo(
        class_name="TestSkills",
        func_name="mover",
        args_schema=schema,
        uses=("movement",),
        lifecycle="background",
    )
    rpc_calls = _make_rpc_calls([mover], {"mover": "moving"})

    # `_handle_tools_call` reads skill metadata and the registry off `app.state`,
    # not the `skills` arg; set/restore them so other tests aren't affected.
    saved_skills = app.state.skills_by_name
    saved_registry = app.state.cap_registry
    app.state.skills_by_name = {"mover": mover}
    app.state.cap_registry = CapabilityRegistry()
    try:
        asyncio.run(
            handle_request(
                {"method": "tools/call", "id": 9, "params": {"name": "mover", "arguments": {}}},
                [mover],
                rpc_calls,
            )
        )
        ctx = rpc_calls["mover"].call_args.kwargs["_mcp_context"]
        assert isinstance(ctx["acquire_token"], str) and ctx["acquire_token"]
        # A background skill hands its hold off to the tool-stream lifecycle, so
        # the hold persists after the call, keyed by the tool name.
        assert app.state.cap_registry.snapshot() == {"movement": "mover"}
    finally:
        app.state.skills_by_name = saved_skills
        app.state.cap_registry = saved_registry


@pytest.mark.parametrize(
    ("lifecycle", "advice"), [("background", "stop tool"), ("instant", "wait")]
)
async def test_conflict_uses_the_running_holders_lifecycle_after_discovery_changes(
    lifecycle: str, advice: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    holder = SkillInfo("Patrol", "patrol", "{}", uses=("movement",), lifecycle=lifecycle)
    requester = SkillInfo("Navigator", "navigate", "{}", uses=("movement",))
    rpc_calls = _make_rpc_calls([holder, requester], {})
    registry = CapabilityRegistry()
    monkeypatch.setattr(app, "state", State())
    app.state.skills_by_name = {"patrol": holder, "navigate": requester}
    app.state.cap_registry = registry
    app.state.cap_acquire_timeout = 10.0 if lifecycle == "background" else 0.01
    if lifecycle == "background":
        await handle_request(
            {"method": "tools/call", "id": 1, "params": {"name": "patrol"}},
            [holder, requester],
            rpc_calls,
        )
    else:
        # Simulate an instant invocation that has not yet returned.
        registry.acquire(["movement"], tool_name="patrol", token="held")

    # A second provider makes patrol ambiguous, so only navigate is callable.
    app.state.skills_by_name = {"navigate": requester}
    del rpc_calls["patrol"]
    response = await asyncio.wait_for(
        handle_request(
            {"method": "tools/call", "id": 2, "params": {"name": "navigate"}},
            [requester],
            rpc_calls,
        ),
        timeout=3.0,
    )
    assert response is not None
    text = response["result"]["content"][0]["text"]
    assert "held by 'patrol'" in text
    assert advice in text
    rpc_calls["navigate"].assert_not_called()


def test_instant_holder_conflict_waits_then_runs() -> None:
    """A call blocked by an *instant* holder waits for the hold to clear and then
    runs, instead of being refused. This is what lets two same-capability instant
    tools requested 'at the same time' both succeed (serialized)."""
    schema = json.dumps({"type": "object", "properties": {}})
    holder = SkillInfo(
        class_name="TestSkills",
        func_name="weigh_payload",
        args_schema=schema,
        uses=("payload",),
        lifecycle="instant",
    )
    requester = SkillInfo(
        class_name="TestSkills",
        func_name="secure_payload",
        args_schema=schema,
        uses=("payload",),
        lifecycle="instant",
    )
    rpc_calls = _make_rpc_calls([requester], {"secure_payload": "secured"})

    saved_skills = app.state.skills_by_name
    saved_registry = app.state.cap_registry
    saved_timeout = app.state.cap_acquire_timeout
    app.state.skills_by_name = {"weigh_payload": holder, "secure_payload": requester}
    registry = CapabilityRegistry()
    registry.acquire(["payload"], tool_name="weigh_payload", token="held")
    app.state.cap_registry = registry
    app.state.cap_acquire_timeout = 2.0
    # Free the holder shortly after the requester starts waiting on it.
    releaser = threading.Timer(0.1, registry.release_by_token, args=("held",))
    releaser.start()
    try:
        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 1,
                    "params": {"name": "secure_payload", "arguments": {}},
                },
                [requester],
                rpc_calls,
            )
        )
        # It waited for the hold to clear, then actually ran the tool.
        assert response is not None
        assert response["result"]["content"][0]["text"] == "secured"
        rpc_calls["secure_payload"].assert_called_once()
    finally:
        releaser.join()
        app.state.skills_by_name = saved_skills
        app.state.cap_registry = saved_registry
        app.state.cap_acquire_timeout = saved_timeout
