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
from unittest.mock import MagicMock, patch

from langchain_core.messages import HumanMessage, ToolMessage
from langchain_core.messages.base import BaseMessage
from langgraph.types import Command
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
                {
                    "name": "take_picture",
                    "description": "Take a picture",
                    "inputSchema": {"type": "object", "properties": {}},
                },
            ]
        }
    elif method == "tools/call":
        name = body["params"]["name"]
        args = body["params"].get("arguments", {})
        if name == "add":
            result = {"content": [{"type": "text", "text": str(args.get("x", 0) + args.get("y", 0))}]}
        elif name == "greet":
            result = {
                "content": [
                    {"type": "text", "text": f"Hello, {args.get('name', 'world')}!"}
                ]
            }
        elif name == "take_picture":
            # Simulates `dimos.msgs.sensor_msgs.Image.agent_encode()` output.
            result = {
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": "data:image/jpeg;base64,FAKEPAYLOAD"},
                    }
                ]
            }
        else:
            result = {"content": [{"type": "text", "text": "Skill not found"}]}
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
    client.config = MagicMock()
    client.config.mcp_server_url = "http://localhost:9990/mcp"
    return client


def test_fetch_tools_from_mcp_server(mcp_client: McpClient) -> None:
    tools = mcp_client._fetch_tools()

    assert [t.name for t in tools] == ["add", "greet", "take_picture"]


def test_tool_invocation_via_mcp(mcp_client: McpClient) -> None:
    tools = mcp_client._fetch_tools()
    add_tool = next(t for t in tools if t.name == "add")
    greet_tool = next(t for t in tools if t.name == "greet")

    # tool_call_id is an InjectedToolCallId argument; the LangGraph tool node
    # supplies it at runtime, but here we call .func directly so we pass it
    # explicitly.
    assert add_tool.func(tool_call_id="tc-1", x=2, y=3) == "5"
    assert greet_tool.func(tool_call_id="tc-2", name="Alice") == "Hello, Alice!"


def test_image_tool_returns_langgraph_command(mcp_client: McpClient) -> None:
    """Non-text MCP content rides back as a `Command` that appends a
    ``ToolMessage`` + image-bearing ``HumanMessage`` to the agent state.

    Replaces the previous side-channel (`add_message` after a UUID
    placeholder), which forced an extra agent turn to deliver the image.
    """
    tools = mcp_client._fetch_tools()
    picture_tool = next(t for t in tools if t.name == "take_picture")

    out = picture_tool.func(tool_call_id="tc-image")

    assert isinstance(out, Command)
    messages = out.update["messages"]
    assert len(messages) == 2

    tool_msg, human_msg = messages
    assert isinstance(tool_msg, ToolMessage)
    assert tool_msg.tool_call_id == "tc-image"

    assert isinstance(human_msg, HumanMessage)
    # Tagged so the state reducer can pair this HumanMessage with the
    # corresponding ToolMessage when several tool calls run in parallel.
    assert human_msg.additional_kwargs.get("tool_call_id") == "tc-image"
    blocks = human_msg.content
    assert isinstance(blocks, list)
    # First block is the intro text; the rest carry the image_url payload.
    assert blocks[0]["type"] == "text"
    assert any(
        b.get("type") == "image_url" and "FAKEPAYLOAD" in b["image_url"]["url"]
        for b in blocks[1:]
    )


def test_structured_tool_invocation_injects_tool_call_id(mcp_client: McpClient) -> None:
    """End-to-end: invoking via the ToolCall path lets the wrapper grab
    `tool_call_id` even though `args_schema` is a JSON-Schema dict — the
    behaviour Langchain only ships for Pydantic schemas out of the box.
    """
    tools = mcp_client._fetch_tools()
    picture_tool = next(t for t in tools if t.name == "take_picture")

    result = picture_tool.invoke(
        {
            "name": "take_picture",
            "args": {},
            "id": "tc-via-invoke",
            "type": "tool_call",
        }
    )

    assert isinstance(result, Command)
    messages = result.update["messages"]
    assert messages[0].tool_call_id == "tc-via-invoke"


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
