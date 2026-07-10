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

import json
from queue import Empty, Queue
from threading import Event
import time
from unittest.mock import MagicMock, patch

import httpx
from langchain_core.messages import HumanMessage
from langchain_core.messages.base import BaseMessage
import pytest

from dimos.agents.annotation import skill
from dimos.agents.mcp.mcp_client import McpClient, McpClientConfig
from dimos.core.module import SkillInfo
from dimos.utils.sequential_ids import SequentialIds


def _mock_post(url: str, **kwargs: object) -> MagicMock:
    """Return a fake httpx response based on the JSON-RPC method."""
    body = kwargs.get("json") or (kwargs.get("content") and json.loads(kwargs["content"]))
    assert isinstance(body, dict)
    method = body["method"]
    req_id = body["id"]

    result: object
    if method == "initialize":
        result = {
            "protocolVersion": "2024-11-05",
            "capabilities": {"tools": {}},
            "serverInfo": {"name": "dimensional", "version": "1.0.0"},
        }
    elif method == "tools/list":
        result = {
            "tools": [
                {
                    "name": "add",
                    "description": "Add two numbers",
                    "inputSchema": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "integer"},
                            "y": {"type": "integer"},
                        },
                        "required": ["x", "y"],
                    },
                },
                {
                    "name": "greet",
                    "description": "Say hello",
                    "inputSchema": {
                        "type": "object",
                        "properties": {
                            "name": {"type": "string"},
                        },
                    },
                },
            ]
        }
    elif method == "tools/call":
        name = body["params"]["name"]
        args = body["params"].get("arguments", {})
        if name == "add":
            text = str(args.get("x", 0) + args.get("y", 0))
        elif name == "greet":
            text = f"Hello, {args.get('name', 'world')}!"
        else:
            text = "Skill not found"
        result = {"content": [{"type": "text", "text": text}]}
    else:
        resp = MagicMock()
        resp.status_code = 200
        resp.raise_for_status = MagicMock()
        resp.json.return_value = {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": -32601, "message": f"Unknown: {method}"},
        }
        return resp

    resp = MagicMock()
    resp.status_code = 200
    resp.raise_for_status = MagicMock()
    resp.json.return_value = {"jsonrpc": "2.0", "id": req_id, "result": result}
    return resp


@pytest.fixture
def mcp_client() -> McpClient:
    """Build an McpClient wired to the mock MCP post handler."""
    mock_http = MagicMock()
    mock_http.post.side_effect = _mock_post

    with patch("dimos.agents.mcp.mcp_client.httpx.Client", return_value=mock_http):
        client = McpClient.__new__(McpClient)

    client._http_client = mock_http
    client._seq_ids = SequentialIds()
    client._tool_registry = {}
    client._direct_tool_payload = None
    client._direct_rpc_calls = {}
    client._last_user_task = ""
    client.config = MagicMock()
    client.config.mcp_server_url = "http://localhost:9990/mcp"
    return client


def test_mcp_client_does_not_inherit_proxy_environment() -> None:
    client: McpClient | None = None
    with patch("dimos.agents.mcp.mcp_client.httpx.Client") as client_factory:
        try:
            client = McpClient()
        finally:
            if client is not None:
                client.stop()

    client_factory.assert_called_once_with(timeout=120.0, trust_env=False)


def test_default_mcp_server_url_uses_ipv4_loopback() -> None:
    assert McpClientConfig().mcp_server_url == "http://127.0.0.1:9990/mcp"


def test_on_system_modules_initializes_agent_without_blocking_startup() -> None:
    client = McpClient()
    fetch_started = Event()

    def slow_fetch_tools() -> list[object]:
        fetch_started.set()
        time.sleep(0.25)
        return []

    def fake_create_agent(**kwargs: object) -> MagicMock:
        return MagicMock()

    client._fetch_tools = slow_fetch_tools  # type: ignore[method-assign]

    try:
        with patch("langchain.agents.create_agent", side_effect=fake_create_agent):
            started_at = time.monotonic()
            client.on_system_modules([])
            elapsed = time.monotonic() - started_at

        assert elapsed < 0.1
        assert fetch_started.wait(1.0)
    finally:
        client.stop()


def test_on_system_modules_defers_direct_tool_registration_outside_rpc_path() -> None:
    client = McpClient()
    registration_started = Event()
    release_registration = Event()

    def slow_register_tools(modules: list[object]) -> None:
        registration_started.set()
        release_registration.wait(1.0)

    client._register_direct_tools = slow_register_tools  # type: ignore[method-assign]
    client._initialize_agent_graph = MagicMock()  # type: ignore[method-assign]

    try:
        with patch("dimos.agents.mcp.mcp_client._AGENT_INIT_START_DELAY_SECONDS", 0.0):
            started_at = time.monotonic()
            client.on_system_modules([MagicMock()])
            elapsed = time.monotonic() - started_at

        assert elapsed < 0.1
        assert registration_started.wait(1.0)
    finally:
        release_registration.set()
        client.stop()


def test_fetch_tools_from_mcp_server(mcp_client: McpClient) -> None:
    tools = mcp_client._fetch_tools()

    assert len(tools) == 2
    assert tools[0].name == "add"
    assert tools[1].name == "greet"


def test_try_fetch_tools_retries_transient_initialize_timeout(mcp_client: McpClient) -> None:
    initialize_attempts = 0

    def flaky_request(
        method: str,
        params: dict[str, object] | None = None,
        timeout: float | None = None,
    ) -> dict[str, object]:
        nonlocal initialize_attempts
        if method == "initialize":
            initialize_attempts += 1
            if initialize_attempts == 1:
                raise httpx.TimeoutException("server not accepting requests yet")
            return {"protocolVersion": "2024-11-05"}
        if method == "tools/list":
            return {"tools": []}
        raise AssertionError(f"Unexpected MCP request: {method}")

    mcp_client._mcp_request = flaky_request  # type: ignore[method-assign]

    result = mcp_client._try_fetch_tools(timeout=1.0, interval=0.01)

    assert result == {"tools": []}
    assert initialize_attempts == 2


def test_register_direct_tools_from_system_modules(mcp_client: McpClient) -> None:
    class DirectSkills:
        @skill
        def ping(self, name: str) -> str:
            """Ping by name."""
            return name

    class DirectProxy:
        remote_name = "DirectSkills"
        actor_class = DirectSkills

        def get_skills(self) -> list[SkillInfo]:
            raise AssertionError("direct registration should prefer actor_class")

    mcp_client.rpc = MagicMock()

    mcp_client._register_direct_tools([DirectProxy()])

    assert mcp_client._direct_tool_payload is not None
    assert [tool["name"] for tool in mcp_client._direct_tool_payload["tools"]] == ["ping"]
    assert set(mcp_client._direct_rpc_calls) == {"ping"}


def test_fetch_tools_uses_direct_registry_without_http(mcp_client: McpClient) -> None:
    direct_call = MagicMock(return_value="pong")
    mcp_client._direct_tool_payload = {
        "tools": [
            {
                "name": "ping",
                "description": "Ping by name.",
                "inputSchema": {
                    "type": "object",
                    "properties": {"name": {"type": "string"}},
                    "required": ["name"],
                },
            }
        ]
    }
    mcp_client._direct_rpc_calls = {"ping": direct_call}
    mcp_client._mcp_request = MagicMock(side_effect=AssertionError("HTTP should not be used"))

    tools = mcp_client._fetch_tools()

    assert [tool.name for tool in tools] == ["ping"]
    assert tools[0].func(name="agent") == "pong"
    direct_call.assert_called_once_with(
        name="agent",
        _mcp_context={"progress_token": direct_call.call_args.kwargs["_mcp_context"]["progress_token"]},
    )


def test_tool_invocation_via_mcp(mcp_client: McpClient) -> None:
    tools = mcp_client._fetch_tools()
    add_tool = next(t for t in tools if t.name == "add")
    greet_tool = next(t for t in tools if t.name == "greet")

    assert add_tool.func(x=2, y=3) == "5"
    assert greet_tool.func(name="Alice") == "Hello, Alice!"


def test_mcp_request_error_propagation(mcp_client: McpClient) -> None:
    def error_post(url: str, **kwargs: object) -> MagicMock:
        resp = MagicMock()
        resp.status_code = 200
        resp.raise_for_status = MagicMock()
        resp.json.return_value = {
            "jsonrpc": "2.0",
            "id": 1,
            "error": {"code": -32601, "message": "Unknown: bad/method"},
        }
        return resp

    mcp_client._http_client.post.side_effect = error_post

    try:
        mcp_client._mcp_request("bad/method")
        raise AssertionError("Expected RuntimeError")
    except RuntimeError as e:
        assert "Unknown: bad/method" in str(e)


def test_tool_stream_notification_becomes_human_message(mcp_client: McpClient) -> None:
    """A `notifications/message` delivered over LCM becomes a HumanMessage."""
    mcp_client._message_queue = Queue()

    notification = {
        "jsonrpc": "2.0",
        "method": "notifications/message",
        "params": {
            "level": "info",
            "logger": "follow_person",
            "data": "Person follow stopped: lost track.",
        },
    }
    mcp_client._on_tool_stream_message(notification)

    msg: BaseMessage = mcp_client._message_queue.get_nowait()
    assert isinstance(msg, HumanMessage)
    assert "[tool:follow_person]" in str(msg.content)
    assert "Person follow stopped: lost track." in str(msg.content)


def test_tool_stream_ignores_unrelated_frames(mcp_client: McpClient) -> None:
    """Unknown methods and empty bodies are dropped on the floor."""

    mcp_client._message_queue = Queue()

    mcp_client._on_tool_stream_message({"jsonrpc": "2.0", "method": "notifications/other"})
    mcp_client._on_tool_stream_message(
        {"jsonrpc": "2.0", "method": "notifications/message", "params": {"data": ""}}
    )
    mcp_client._on_tool_stream_message(
        {"jsonrpc": "2.0", "method": "notifications/progress", "params": {"message": ""}}
    )

    with pytest.raises(Empty):
        mcp_client._message_queue.get_nowait()


def test_tool_stream_progress_frame_becomes_human_message(mcp_client: McpClient) -> None:
    """A `notifications/progress` frame is routed as a HumanMessage."""

    mcp_client._message_queue = Queue()

    progress_frame = {
        "jsonrpc": "2.0",
        "method": "notifications/progress",
        "params": {
            "progressToken": "pt-abc",
            "progress": 1,
            "message": "Found a person",
            "_meta": {"tool_name": "follow_person"},
        },
    }
    mcp_client._on_tool_stream_message(progress_frame)

    msg: BaseMessage = mcp_client._message_queue.get_nowait()
    assert isinstance(msg, HumanMessage)
    assert str(msg.content) == "[tool:follow_person] Found a person"


def test_mcp_tool_call_sends_progress_token(mcp_client: McpClient) -> None:
    """Every `tools/call` request carries a `_meta.progressToken`."""
    captured: dict[str, object] = {}

    def fake_request(method: str, params: dict[str, object] | None = None) -> dict[str, object]:
        captured["method"] = method
        captured["params"] = params
        return {"content": [{"type": "text", "text": "ok"}]}

    mcp_client._mcp_request = fake_request
    mcp_client._mcp_tool_call("add", {"x": 1, "y": 2})

    assert captured["method"] == "tools/call"
    params = captured["params"]
    assert isinstance(params, dict)
    assert params["name"] == "add"
    assert params["arguments"] == {"x": 1, "y": 2}
    meta = params["_meta"]
    assert isinstance(meta, dict)
    token = meta["progressToken"]
    assert isinstance(token, str) and len(token) > 0


def test_mcp_tool_call_records_structured_skill_outcome(mcp_client: McpClient) -> None:
    """When the outcome store is available, agent tool calls are recorded."""
    calls: list[dict[str, object]] = []

    def fake_request(method: str, params: dict[str, object] | None = None) -> dict[str, object]:
        assert params is not None
        calls.append(params)
        if params["name"] == "navigate_with_text":
            return {
                "content": [
                    {
                        "type": "text",
                        "text": json.dumps(
                            {
                                "success": False,
                                "message": "No matching location found",
                                "error_code": "EXECUTION_FAILED",
                                "duration_ms": 12.0,
                            }
                        ),
                    }
                ]
            }
        if params["name"] == "record_skill_outcome":
            return {"content": [{"type": "text", "text": "recorded"}]}
        raise AssertionError(f"Unexpected tool call: {params['name']}")

    mcp_client._mcp_request = fake_request
    mcp_client._tool_registry = {
        "navigate_with_text": {},
        "record_skill_outcome": {},
    }

    result = mcp_client._mcp_tool_call("navigate_with_text", {"query": "kitchen"})

    assert result["content"][0]["type"] == "text"
    assert [call["name"] for call in calls] == ["navigate_with_text", "record_skill_outcome"]
    record_args = calls[1]["arguments"]
    assert isinstance(record_args, dict)
    assert record_args["skill_name"] == "navigate_with_text"
    assert record_args["success"] is False
    assert record_args["domain"] == "navigation"
    assert record_args["error_code"] == "EXECUTION_FAILED"
    assert record_args["risk"] == "medium"


def test_mcp_tool_call_skips_layer_3_internal_outcome_recording(mcp_client: McpClient) -> None:
    """Layer 3 support tools should not pollute the skill-outcome store."""
    calls: list[dict[str, object]] = []

    def fake_request(method: str, params: dict[str, object] | None = None) -> dict[str, object]:
        assert params is not None
        calls.append(params)
        return {"content": [{"type": "text", "text": "context"}]}

    mcp_client._mcp_request = fake_request
    mcp_client._tool_registry = {
        "get_context": {},
        "record_skill_outcome": {},
    }

    mcp_client._mcp_tool_call("get_context", {"task": "go to the kitchen"})

    assert [call["name"] for call in calls] == ["get_context"]
