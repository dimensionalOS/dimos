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
from unittest.mock import MagicMock, patch

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_core.messages.base import BaseMessage
from langgraph.types import Command
import pytest

from dimos.agents.mcp.mcp_client import (
    McpClient,
    _fix_parallel_tool_batches,
    _reorder_tool_responses,
)
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
                {
                    "name": "narrate_picture",
                    "description": "Take a picture and describe what's in it",
                    "inputSchema": {"type": "object", "properties": {}},
                },
            ]
        }
    elif method == "tools/call":
        name = body["params"]["name"]
        args = body["params"].get("arguments", {})
        if name == "add":
            result = {
                "content": [{"type": "text", "text": str(args.get("x", 0) + args.get("y", 0))}]
            }
        elif name == "greet":
            result = {"content": [{"type": "text", "text": f"Hello, {args.get('name', 'world')}!"}]}
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
        elif name == "narrate_picture":
            # Tool that returns both prose AND an image (e.g. a VLM
            # describing what it sees). Exercises the `summary = text`
            # branch of the Command-building path — the fallback
            # "{name} returned N artefact(s)" sentinel must NOT be used
            # when the tool already provided real text.
            result = {
                "content": [
                    {"type": "text", "text": "I see a chair and a window."},
                    {
                        "type": "image_url",
                        "image_url": {"url": "data:image/jpeg;base64,FAKEPAYLOAD"},
                    },
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

    assert [t.name for t in tools] == ["add", "greet", "take_picture", "narrate_picture"]


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
        b.get("type") == "image_url" and "FAKEPAYLOAD" in b["image_url"]["url"] for b in blocks[1:]
    )


def test_image_tool_with_text_uses_real_text_as_tool_message(mcp_client: McpClient) -> None:
    """When a tool returns BOTH text and image content, the ToolMessage
    carries the tool's actual narration — not the
    "{name} returned N artefact(s)" fallback sentinel. The image still
    rides back on the follow-up HumanMessage as usual.
    """
    tools = mcp_client._fetch_tools()
    narrate_tool = next(t for t in tools if t.name == "narrate_picture")

    out = narrate_tool.func(tool_call_id="tc-narrate")

    assert isinstance(out, Command)
    tool_msg, human_msg = out.update["messages"]

    assert isinstance(tool_msg, ToolMessage)
    assert tool_msg.content == "I see a chair and a window."
    assert "artefact" not in str(tool_msg.content)

    assert isinstance(human_msg, HumanMessage)
    blocks = human_msg.content
    assert isinstance(blocks, list)
    assert any(
        b.get("type") == "image_url" and "FAKEPAYLOAD" in b["image_url"]["url"] for b in blocks[1:]
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


def test_structured_tool_ainvoke_injects_tool_call_id(mcp_client: McpClient) -> None:
    """Async mirror of `test_structured_tool_invocation_injects_tool_call_id`:
    langgraph's tool node may dispatch via `ainvoke` (e.g. under `astream`),
    so the async path must inject `tool_call_id` the same way the sync path
    does. A langchain release that changes ainvoke's call convention should
    fail this test rather than silently drop the id.
    """
    tools = mcp_client._fetch_tools()
    picture_tool = next(t for t in tools if t.name == "take_picture")

    result = asyncio.run(
        picture_tool.ainvoke(
            {
                "name": "take_picture",
                "args": {},
                "id": "tc-via-ainvoke",
                "type": "tool_call",
            }
        )
    )

    assert isinstance(result, Command)
    messages = result.update["messages"]
    assert messages[0].tool_call_id == "tc-via-ainvoke"


def test_structured_tool_invocation_without_toolcall_raises(mcp_client: McpClient) -> None:
    """Bare-dict invocation (no ToolCall envelope) must fail loud, so a
    future langchain change that bypasses our `invoke` override is caught
    by tests instead of silently dropping `tool_call_id`.
    """
    tools = mcp_client._fetch_tools()
    picture_tool = next(t for t in tools if t.name == "take_picture")

    with pytest.raises(ValueError, match="ToolCall"):
        picture_tool.invoke({"name": "take_picture", "args": {}})


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


def _ai_with_parallel_calls(call_ids: list[str]) -> AIMessage:
    return AIMessage(
        content="",
        tool_calls=[{"name": f"t{i}", "args": {}, "id": cid} for i, cid in enumerate(call_ids)],
    )


def _image_human(tool_call_id: str) -> HumanMessage:
    return HumanMessage(
        content=[{"type": "text", "text": "img"}],
        additional_kwargs={"tool_call_id": tool_call_id},
    )


def test_fix_parallel_tool_batches_reorders_interleaved_responses() -> None:
    """[AI(parallel), Tool₁, Human₁, Tool₂, Human₂] should become
    [AI, Tool₁, Tool₂, Human₁, Human₂] so OpenAI doesn't reject the next
    turn for non-contiguous parallel tool responses."""
    messages: list[BaseMessage] = [
        _ai_with_parallel_calls(["a", "b"]),
        ToolMessage(content="summary-a", tool_call_id="a"),
        _image_human("a"),
        ToolMessage(content="summary-b", tool_call_id="b"),
        _image_human("b"),
    ]

    out = _fix_parallel_tool_batches(messages)

    assert isinstance(out[0], AIMessage)
    assert isinstance(out[1], ToolMessage) and out[1].tool_call_id == "a"
    assert isinstance(out[2], ToolMessage) and out[2].tool_call_id == "b"
    assert isinstance(out[3], HumanMessage)
    assert out[3].additional_kwargs["tool_call_id"] == "a"
    assert isinstance(out[4], HumanMessage)
    assert out[4].additional_kwargs["tool_call_id"] == "b"


def test_fix_parallel_tool_batches_leaves_single_tool_call_alone() -> None:
    """Single tool calls already satisfy the contiguity rule — don't touch them."""
    messages: list[BaseMessage] = [
        AIMessage(
            content="",
            tool_calls=[{"name": "t", "args": {}, "id": "solo"}],
        ),
        ToolMessage(content="summary", tool_call_id="solo"),
        _image_human("solo"),
    ]

    out = _fix_parallel_tool_batches(messages)
    assert out == messages


def test_fix_parallel_tool_batches_leaves_already_ordered_alone() -> None:
    """[AI, Tool₁, Tool₂, Human₁, Human₂] is already valid; don't reshuffle it."""
    messages: list[BaseMessage] = [
        _ai_with_parallel_calls(["a", "b"]),
        ToolMessage(content="sa", tool_call_id="a"),
        ToolMessage(content="sb", tool_call_id="b"),
        _image_human("a"),
        _image_human("b"),
    ]

    out = _fix_parallel_tool_batches(messages)
    assert out == messages


def test_fix_parallel_tool_batches_skips_untagged_human_messages() -> None:
    """A plain HumanMessage with no `tool_call_id` tag terminates the run —
    we won't reorder past it because we can't safely attribute it to a
    parallel call."""
    plain_human = HumanMessage(content="just talking")
    messages: list[BaseMessage] = [
        _ai_with_parallel_calls(["a", "b"]),
        ToolMessage(content="sa", tool_call_id="a"),
        plain_human,
        ToolMessage(content="sb", tool_call_id="b"),
    ]

    out = _fix_parallel_tool_batches(messages)
    # Untouched: we stopped scanning at the plain human, so no rewrite.
    assert out == messages


def test_reorder_tool_responses_merges_then_fixes() -> None:
    """The reducer runs add_messages first, then applies the fix — so an
    incoming Command-style append of [Tool, Human] for the second parallel
    call lands contiguously after the first batch."""
    left: list[BaseMessage] = [
        _ai_with_parallel_calls(["a", "b"]),
        ToolMessage(content="sa", tool_call_id="a"),
        _image_human("a"),
    ]
    right: list[BaseMessage] = [
        ToolMessage(content="sb", tool_call_id="b"),
        _image_human("b"),
    ]

    out = _reorder_tool_responses(left, right)

    tool_ids = [m.tool_call_id for m in out if isinstance(m, ToolMessage)]
    human_ids = [
        m.additional_kwargs.get("tool_call_id") for m in out if isinstance(m, HumanMessage)
    ]
    assert tool_ids == ["a", "b"]
    assert human_ids == ["a", "b"]
    # And critically: both ToolMessages come before either HumanMessage.
    first_human = next(i for i, m in enumerate(out) if isinstance(m, HumanMessage))
    last_tool = max(i for i, m in enumerate(out) if isinstance(m, ToolMessage))
    assert last_tool < first_human


def test_process_message_normalizes_history_after_parallel_tool_batch(
    mcp_client: McpClient,
) -> None:
    """Regression: stream_mode="updates" yields node outputs in completion
    order, so when two parallel image-returning tools each emit a
    [ToolMessage, HumanMessage] Command, self._history ends up interleaved
    as [Tool₁, Human₁, Tool₂, Human₂]. The graph's reducer reorders its
    own channel state, but langgraph does NOT re-run reducers when an
    initial state dict is fed back into stream() on the next turn — so
    without an explicit fix-up here, OpenAI rejects the next user turn
    for non-contiguous parallel ToolMessages.
    """
    mcp_client._history = []
    mcp_client._message_queue = Queue()
    mcp_client.agent_idle = MagicMock()
    mcp_client.agent = MagicMock()

    ai = _ai_with_parallel_calls(["a", "b"])
    tool_a = ToolMessage(content="sa", tool_call_id="a")
    human_a = _image_human("a")
    tool_b = ToolMessage(content="sb", tool_call_id="b")
    human_b = _image_human("b")

    # Mirror what langgraph's `stream_mode="updates"` looks like when two
    # parallel ToolNode invocations finish out-of-order with respect to
    # their image follow-ups: each Command landed [Tool, Human] in the
    # raw node output stream.
    fake_graph = MagicMock()
    fake_graph.stream.return_value = iter(
        [
            {"agent": {"messages": [ai]}},
            {"tools": {"messages": [tool_a, human_a]}},
            {"tools": {"messages": [tool_b, human_b]}},
        ]
    )

    user_msg = HumanMessage(content="look around")
    mcp_client._process_message(fake_graph, user_msg)

    # Critical post-condition: both ToolMessages are contiguous, and only
    # then come the image HumanMessages. Without the fix-up, _history
    # would be [user, ai, tool_a, human_a, tool_b, human_b].
    tool_positions = [i for i, m in enumerate(mcp_client._history) if isinstance(m, ToolMessage)]
    human_positions = [
        i
        for i, m in enumerate(mcp_client._history)
        if isinstance(m, HumanMessage) and m.additional_kwargs.get("tool_call_id")
    ]
    assert tool_positions == sorted(tool_positions)
    assert max(tool_positions) < min(human_positions)


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
