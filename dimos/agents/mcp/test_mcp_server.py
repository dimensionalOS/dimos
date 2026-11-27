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

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import json
import sys
import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
import requests

from dimos.agents.mcp.mcp_adapter import MCP_PROTOCOL_VERSION, McpAdapter
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.mcp.tool_stream import make_notification
from dimos.agents.skill_result import SkillResult
from dimos.core.module import SkillInfo
from dimos.utils.testing.waiting import wait_until

ServeSkills = Callable[[list[SkillInfo], dict[str, Callable[..., Any]]], McpAdapter]

# The `_meta` that McpAdapter sends with every request, and so what a skill is given.
META = {
    "io.modelcontextprotocol/protocolVersion": MCP_PROTOCOL_VERSION,
    "io.modelcontextprotocol/clientCapabilities": {},
}


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


def test_mcp_module_request_flow(serve_skills: ServeSkills) -> None:
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
    adapter = serve_skills(skills, rpc_calls)

    response = adapter.call("tools/list")
    assert response["result"]["tools"][0]["name"] == "add"
    assert response["result"]["tools"][0]["description"] == "Add two numbers"

    response = adapter.call("tools/call", {"name": "add", "arguments": {"x": 2, "y": 3}})
    assert response["result"]["content"][0]["text"] == "5"
    rpc_calls["add"].assert_called_once_with(x=2, y=3, _mcp_context={"meta": META})


def test_tools_list_advertises_the_schema_of_each_skill(serve_skills: ServeSkills) -> None:
    pose = {
        "properties": {"x": {"title": "X", "type": "number"}},
        "required": ["x"],
        "title": "Pose",
        "type": "object",
    }
    node = {
        "properties": {"children": {"items": {"$ref": "#/$defs/Node"}, "type": "array"}},
        "title": "Node",
        "type": "object",
    }
    schema = json.dumps(
        {
            "$defs": {"Pose": pose, "Node": node},
            "description": "Go somewhere.",
            "properties": {
                "goal": {"$ref": "#/$defs/Pose", "description": "Where to go."},
                "via": {
                    "anyOf": [
                        {"items": {"$ref": "#/$defs/Pose"}, "type": "array"},
                        {"type": "null"},
                    ],
                    "default": None,
                    "title": "Via",
                },
                "speed": {"default": 0.5, "title": "Speed", "type": "number"},
                "tree": {"$ref": "#/$defs/Node"},
            },
            "required": ["goal", "tree"],
            "title": "go_to",
            "type": "object",
        }
    )
    skills = [SkillInfo(class_name="TestSkills", func_name="go_to", args_schema=schema)]
    adapter = serve_skills(skills, _make_rpc_calls(skills, {}))

    assert adapter.list_tools() == [
        {
            "name": "go_to",
            "description": "Go somewhere.",
            "inputSchema": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "goal": {**pose, "description": "Where to go."},
                    "via": {
                        "anyOf": [{"items": pose, "type": "array"}, {"type": "null"}],
                        "default": None,
                        "title": "Via",
                    },
                    "speed": {"default": 0.5, "title": "Speed", "type": "number"},
                    # A model that contains itself is open from the second level down.
                    "tree": {
                        "properties": {"children": {"items": {}, "type": "array"}},
                        "title": "Node",
                        "type": "object",
                    },
                },
                "required": ["goal", "tree"],
            },
        }
    ]


def test_mcp_module_injects_progress_token_as_mcp_context(serve_skills: ServeSkills) -> None:
    """When the client sends `_meta.progressToken`, the RPC call receives it as
    an `_mcp_context` kwarg so the `@skill` wrapper can stash it in the
    thread-local that `ToolStream` reads on construction."""
    schema = json.dumps({"type": "object", "properties": {"x": {"type": "integer"}}})
    skills = [SkillInfo(class_name="TestSkills", func_name="echo", args_schema=schema)]
    rpc_calls = _make_rpc_calls(skills, {"echo": "ok"})

    serve_skills(skills, rpc_calls).call(
        "tools/call",
        {
            "name": "echo",
            "arguments": {"x": 42},
            "_meta": {"progressToken": "pt-srv-test"},
        },
    )

    rpc_calls["echo"].assert_called_once_with(
        x=42,
        _mcp_context={
            "meta": {**META, "progressToken": "pt-srv-test"},
            "progress_token": "pt-srv-test",
        },
    )


def test_mcp_module_without_arguments_passes_only_the_context(serve_skills: ServeSkills) -> None:
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="TestSkills", func_name="ping", args_schema=schema)]
    rpc_calls = _make_rpc_calls(skills, {"ping": "pong"})

    serve_skills(skills, rpc_calls).call("tools/call", {"name": "ping", "arguments": {}})

    rpc_calls["ping"].assert_called_once_with(_mcp_context={"meta": META})


def test_mcp_module_handles_errors(serve_skills: ServeSkills) -> None:
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [
        SkillInfo(class_name="TestSkills", func_name="ok_skill", args_schema=schema),
        SkillInfo(class_name="TestSkills", func_name="fail_skill", args_schema=schema),
    ]

    rpc_calls = _make_rpc_calls(skills, {"ok_skill": "done"})
    rpc_calls["fail_skill"] = MagicMock(side_effect=RuntimeError("boom"))
    adapter = serve_skills(skills, rpc_calls)

    # All skills listed
    response = adapter.call("tools/list")
    tool_names = {tool["name"] for tool in response["result"]["tools"]}
    assert "ok_skill" in tool_names
    assert "fail_skill" in tool_names

    # A raised skill error is a failed tool call carrying the exception text.
    response = adapter.call("tools/call", {"name": "fail_skill", "arguments": {}})
    assert response["result"]["isError"] is True
    text = response["result"]["content"][0]["text"]
    assert "Error running tool 'fail_skill'" in text
    assert "RuntimeError: boom" in text

    # A skill that returns is not flagged.
    response = adapter.call("tools/call", {"name": "ok_skill", "arguments": {}})
    assert "isError" not in response["result"]
    assert response["result"]["content"][0]["text"] == "done"

    # An argument the skill does not take is a failed tool call too, and never reaches it.
    response = adapter.call("tools/call", {"name": "ok_skill", "arguments": {"extra": 1}})
    assert response["result"]["isError"] is True
    assert response["result"]["content"][0]["text"] == "Unexpected arguments: extra"
    rpc_calls["ok_skill"].assert_called_once()

    # An unknown skill is a protocol error
    response = adapter.call("tools/call", {"name": "no_such", "arguments": {}})
    assert response["error"] == {"code": -32602, "message": "Unknown tool: no_such"}


def test_skill_result_without_strict_json_answers_with_text_only(
    serve_skills: ServeSkills,
) -> None:
    """Strict JSON has no Infinity, so the endpoint could not render it as structuredContent."""
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="TestSkills", func_name="measure", args_schema=schema)]

    def measure(**_: object) -> SkillResult:
        return SkillResult("distance inf", metadata={"distance": float("inf")})

    response = serve_skills(skills, {"measure": measure}).call(
        "tools/call", {"name": "measure", "arguments": {}}
    )
    assert response["result"]["content"] == [
        {
            "type": "text",
            "text": '{"message": "distance inf", "duration_ms": 0.0, "metadata": {"distance": Infinity}}',
        }
    ]
    assert "structuredContent" not in response["result"]


def test_skill_result_answers_with_structured_content(serve_skills: ServeSkills) -> None:
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="TestSkills", func_name="measure", args_schema=schema)]
    rpc_calls = {"measure": lambda **_: SkillResult("distance 2", metadata={"distance": 2.0})}

    response = serve_skills(skills, rpc_calls).call(
        "tools/call", {"name": "measure", "arguments": {}}
    )

    structured = {"message": "distance 2", "duration_ms": 0.0, "metadata": {"distance": 2.0}}
    assert response["result"]["content"] == [{"type": "text", "text": json.dumps(structured)}]
    assert response["result"]["structuredContent"] == structured


def test_an_image_result_is_mcp_image_content(serve_skills: ServeSkills) -> None:
    """`Image.agent_encode()` gives a data URL, which is not a kind of MCP content."""
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="TestSkills", func_name="look", args_schema=schema)]
    encoded = [{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,/9j/4A=="}}]
    rpc_calls = {"look": lambda **_: SimpleNamespace(agent_encode=lambda: encoded)}

    response = serve_skills(skills, rpc_calls).call("tools/call", {"name": "look", "arguments": {}})

    assert response["result"]["content"] == [
        {"type": "image", "data": "/9j/4A==", "mimeType": "image/jpeg"}
    ]


def test_mcp_module_discover_and_unknown(serve_skills: ServeSkills) -> None:
    adapter = serve_skills([], {})

    response = adapter.call("server/discover")
    assert response["result"]["supportedVersions"] == [MCP_PROTOCOL_VERSION]
    server_info = response["result"]["_meta"]["io.modelcontextprotocol/serverInfo"]
    assert server_info["name"] == "dimensional"

    response = adapter.call("unknown/method")
    assert response["error"]["code"] == -32601


def test_a_skill_name_in_two_modules_is_one_tool_that_calls_the_later_module(
    lone_mcp_server: McpServer,
) -> None:
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="Arm", func_name="home", args_schema=schema)]
    modules = [
        SimpleNamespace(get_skills=lambda: skills, remote_name="left/arm"),
        SimpleNamespace(get_skills=lambda: skills, remote_name="right/arm"),
    ]

    with patch(
        "dimos.agents.mcp.mcp_server.RpcCall",
        side_effect=lambda _cls, _rpc, name, remote_name, _unsubs: lambda **_: remote_name,
    ):
        lone_mcp_server.on_system_modules(modules)
    adapter = McpAdapter(lone_mcp_server.mcp_url())

    assert [tool["name"] for tool in adapter.list_tools()] == ["home"]
    assert adapter.call_tool_text("home") == "right/arm"


def test_the_tool_list_follows_the_modules(serve_skills: ServeSkills) -> None:
    schema = json.dumps({"type": "object", "properties": {}})
    first = [SkillInfo(class_name="TestSkills", func_name="first", args_schema=schema)]
    second = [SkillInfo(class_name="TestSkills", func_name="second", args_schema=schema)]

    adapter = serve_skills(first, _make_rpc_calls(first, {}))
    assert [tool["name"] for tool in adapter.list_tools()] == ["first"]

    serve_skills(second, _make_rpc_calls(second, {}))
    assert [tool["name"] for tool in adapter.list_tools()] == ["second"]


def test_mcp_module_injects_acquire_token_for_capability_skill(
    lone_mcp_server: McpServer, serve_skills: ServeSkills
) -> None:
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

    serve_skills([mover], rpc_calls).call("tools/call", {"name": "mover", "arguments": {}})

    ctx = rpc_calls["mover"].call_args.kwargs["_mcp_context"]
    assert isinstance(ctx["acquire_token"], str) and ctx["acquire_token"]
    # A background skill hands its hold off to the tool-stream lifecycle, so
    # the hold persists after the call, keyed by the tool name.
    assert lone_mcp_server._cap_registry.snapshot() == {"movement": "mover"}


def test_refusal_message_distinguishes_holder_lifecycle(
    lone_mcp_server: McpServer, serve_skills: ServeSkills
) -> None:
    """The conflict message tells the LLM to call a stop tool for a background
    holder, but to wait for an instant holder (which has no stop tool)."""
    schema = json.dumps({"type": "object", "properties": {}})
    requester = SkillInfo(
        class_name="TestSkills",
        func_name="follow_person",
        args_schema=schema,
        uses=("movement",),
        lifecycle="background",
    )
    # An instant holder is waited on; keep that wait short so the test that
    # exercises the timeout path stays fast.
    lone_mcp_server._cap_acquire_timeout = 0.05

    def _refusal_text(holder: SkillInfo) -> str:
        rpc_calls = _make_rpc_calls([holder, requester], {"follow_person": "ok"})
        adapter = serve_skills([holder, requester], rpc_calls)
        registry = lone_mcp_server._cap_registry
        registry.acquire(["movement"], tool_name=holder.func_name, token="held")
        try:
            response = adapter.call("tools/call", {"name": "follow_person", "arguments": {}})
        finally:
            registry.release_by_token("held")
        # The requester is refused, so its RPC never runs.
        rpc_calls["follow_person"].assert_not_called()
        text: str = response["result"]["content"][0]["text"]
        return text

    background_text = _refusal_text(
        SkillInfo(
            class_name="TestSkills",
            func_name="start_patrol",
            args_schema=schema,
            uses=("movement",),
            lifecycle="background",
        )
    )
    assert "held by 'start_patrol'" in background_text
    assert "stop tool" in background_text

    instant_text = _refusal_text(
        SkillInfo(
            class_name="TestSkills",
            func_name="turn_in_place",
            args_schema=schema,
            uses=("movement",),
            lifecycle="instant",
        )
    )
    assert "held by 'turn_in_place'" in instant_text
    assert "wait" in instant_text.lower()
    assert "stop tool" not in instant_text


def test_instant_holder_conflict_waits_then_runs(
    lone_mcp_server: McpServer, serve_skills: ServeSkills
) -> None:
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
    rpc_calls = _make_rpc_calls([holder, requester], {"secure_payload": "secured"})
    adapter = serve_skills([holder, requester], rpc_calls)

    registry = lone_mcp_server._cap_registry
    registry.acquire(["payload"], tool_name="weigh_payload", token="held")
    lone_mcp_server._cap_acquire_timeout = 2.0
    # Free the holder shortly after the requester starts waiting on it.
    releaser = threading.Timer(0.1, registry.release_by_token, args=("held",))
    releaser.start()
    try:
        response = adapter.call("tools/call", {"name": "secure_payload", "arguments": {}})
        # It waited for the hold to clear, then actually ran the tool.
        assert response["result"]["content"][0]["text"] == "secured"
        rpc_calls["secure_payload"].assert_called_once()
    finally:
        releaser.join()


def test_the_event_stream_carries_tool_stream_frames_until_its_client_leaves(
    lone_mcp_server: McpServer,
) -> None:
    frame = make_notification("patrol", "Reached the door.")

    with patch("dimos.agents.mcp.mcp_server._SSE_KEEPALIVE_INTERVAL", 0.05):
        with requests.get(lone_mcp_server.mcp_url(), stream=True, timeout=5) as response:
            assert response.headers["Content-Type"] == "text/event-stream"
            lone_mcp_server._on_tool_stream(frame)
            data = next(line for line in response.iter_lines() if line.startswith(b"data: "))
        assert json.loads(data[6:]) == frame
        # Nothing tells the server that the client went, so it has to notice.
        wait_until(lambda: not lone_mcp_server._event_queues, timeout=5, interval=0.01)


def test_stop_ends_the_event_streams(mcp_port: int) -> None:
    if sys.version_info < (3, 11):
        pytest.skip("McpServer needs Python 3.11")
    server = McpServer(port=mcp_port)
    server.start()
    try:
        response = requests.get(server.mcp_url(), stream=True, timeout=5)
    finally:
        server.stop()

    # A stream cut off rather than ended would raise here.
    with response:
        assert not any(response.iter_lines())


def test_stop_does_not_wait_for_a_skill_that_is_still_running(mcp_port: int) -> None:
    if sys.version_info < (3, 11):
        pytest.skip("McpServer needs Python 3.11")
    schema = json.dumps({"type": "object", "properties": {}})
    skills = [SkillInfo(class_name="TestSkills", func_name="patrol", args_schema=schema)]
    running = threading.Event()
    finish = threading.Event()

    def patrol(**_: object) -> str:
        running.set()
        finish.wait()
        return "done"

    module = SimpleNamespace(get_skills=lambda: skills, remote_name="TestSkills")
    with (
        patch("dimos.agents.mcp.mcp_server._SHUTDOWN_TIMEOUT", 0.1),
        patch("dimos.agents.mcp.mcp_server.RpcCall", return_value=patrol),
        ThreadPoolExecutor(max_workers=2) as executor,
    ):
        server = McpServer(port=mcp_port)
        server.start()
        try:
            server.on_system_modules([module])
            call = executor.submit(McpAdapter(server.mcp_url()).call_tool, "patrol")
            started = running.wait(timeout=10)
            executor.submit(server.stop).result(timeout=10)
            assert started
            with pytest.raises(requests.ConnectionError):
                call.result(timeout=10)
        finally:
            finish.set()


def test_python_310_is_refused_before_anything_starts() -> None:
    with patch("dimos.agents.mcp.mcp_server.sys", SimpleNamespace(version_info=(3, 10, 18))):
        with pytest.raises(ImportError, match=r"Python 3\.11 or newer"):
            McpServer.check_dependencies()
