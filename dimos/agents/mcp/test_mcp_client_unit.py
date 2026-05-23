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
from queue import Empty, Queue
import time
from typing import Any
from unittest.mock import MagicMock, patch

from langchain_core.messages import HumanMessage
from langchain_core.messages.base import BaseMessage
import pytest

from dimos.agents.mcp.mcp_client import McpClient
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
                    "lane": "motion",
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
    client._lane_locks = {}
    client.config = MagicMock()
    client.config.mcp_server_url = "http://localhost:9990/mcp"
    return client


def test_lane_stored_in_tool_registry(mcp_client: McpClient) -> None:
    mcp_client._fetch_tools()

    assert mcp_client._tool_registry["add"].get("lane") == "motion"
    assert mcp_client._tool_registry["greet"].get("lane") is None


def test_fetch_tools_from_mcp_server(mcp_client: McpClient) -> None:
    tools = mcp_client._fetch_tools()

    assert len(tools) == 2
    assert tools[0].name == "add"
    assert tools[1].name == "greet"


def test_tool_invocation_via_mcp(mcp_client: McpClient) -> None:
    tools = mcp_client._fetch_tools()
    add_tool = next(t for t in tools if t.name == "add")
    greet_tool = next(t for t in tools if t.name == "greet")

    assert asyncio.run(add_tool.coroutine(x=2, y=3)) == "5"  # type: ignore[arg-type, misc]
    assert asyncio.run(greet_tool.coroutine(name="Alice")) == "Hello, Alice!"  # type: ignore[arg-type, misc]


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


def test_same_lane_tools_execute_serially(mcp_client: McpClient) -> None:
    """Tools sharing a lane never overlap — the second waits for the first."""
    events: list[str] = []

    def slow_call(name: str, kwargs: dict[str, Any]) -> dict[str, Any]:
        events.append(f"{name}:start")
        time.sleep(0.05)
        events.append(f"{name}:end")
        return {"content": [{"type": "text", "text": "ok"}]}

    mcp_client._mcp_tool_call = slow_call  # type: ignore[assignment]
    mcp_client._tool_registry = {
        "a": {"name": "a", "lane": "motion"},
        "b": {"name": "b", "lane": "motion"},
    }
    tool_a = mcp_client._mcp_tool_to_langchain(
        {
            "name": "a",
            "description": "",
            "inputSchema": {"type": "object", "properties": {}},
            "lane": "motion",
        }
    )
    tool_b = mcp_client._mcp_tool_to_langchain(
        {
            "name": "b",
            "description": "",
            "inputSchema": {"type": "object", "properties": {}},
            "lane": "motion",
        }
    )

    async def run() -> None:
        mcp_client._lane_locks.clear()
        await asyncio.gather(tool_a.coroutine(), tool_b.coroutine())  # type: ignore[misc]

    asyncio.run(run())

    a_start, a_end = events.index("a:start"), events.index("a:end")
    b_start, b_end = events.index("b:start"), events.index("b:end")
    assert a_end < b_start or b_end < a_start


def test_different_lane_tools_run_in_parallel(mcp_client: McpClient) -> None:
    """Tools in different lanes overlap — neither waits for the other."""
    timestamps: dict[str, float] = {}

    def slow_call(name: str, kwargs: dict[str, Any]) -> dict[str, Any]:
        timestamps[f"{name}:start"] = time.monotonic()
        time.sleep(0.05)
        timestamps[f"{name}:end"] = time.monotonic()
        return {"content": [{"type": "text", "text": "ok"}]}

    mcp_client._mcp_tool_call = slow_call  # type: ignore[assignment]
    mcp_client._tool_registry = {
        "cam": {"name": "cam", "lane": "camera"},
        "nav": {"name": "nav", "lane": "motion"},
    }
    tool_cam = mcp_client._mcp_tool_to_langchain(
        {
            "name": "cam",
            "description": "",
            "inputSchema": {"type": "object", "properties": {}},
            "lane": "camera",
        }
    )
    tool_nav = mcp_client._mcp_tool_to_langchain(
        {
            "name": "nav",
            "description": "",
            "inputSchema": {"type": "object", "properties": {}},
            "lane": "motion",
        }
    )

    async def run() -> None:
        mcp_client._lane_locks.clear()
        await asyncio.gather(tool_cam.coroutine(), tool_nav.coroutine())  # type: ignore[misc]

    asyncio.run(run())

    # Intervals overlap: each tool starts before the other finishes
    assert timestamps["cam:start"] < timestamps["nav:end"]
    assert timestamps["nav:start"] < timestamps["cam:end"]
