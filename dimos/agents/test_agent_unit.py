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

from collections.abc import Callable, Iterator
from pathlib import Path
from queue import Empty
from threading import RLock, Thread
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

from langchain.agents import create_agent
from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_core.messages.base import BaseMessage
import pytest
import requests

from dimos.agents.agent import Agent
from dimos.agents.mcp.constants import TOOL_ORIGIN_KEY, TURN_FAILED_KEY
from dimos.agents.mcp.mcp_adapter import McpError
from dimos.agents.testing.mock_model import MockModel
from dimos.utils.testing.waiting import wait_until


def _mock_payload(body: dict[str, object]) -> dict[str, object]:
    """Return the JSON-RPC response dict for a request body, keyed by method."""
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
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": -32601, "message": f"Unknown: {method}"},
        }

    return {"jsonrpc": "2.0", "id": req_id, "result": result}


class _FakeServer:
    """Answers McpAdapter's requests.post through `payload`."""

    def __init__(self) -> None:
        self.payload: Callable[[dict[str, object]], dict[str, object]] = _mock_payload
        self.sent: list[dict[str, object]] = []

    def post(self, url: str, *, json: dict[str, object], timeout: float) -> SimpleNamespace:
        self.sent.append(json)
        reply = self.payload(json)
        return SimpleNamespace(status_code=200, raise_for_status=lambda: None, json=lambda: reply)


@pytest.fixture
def server(monkeypatch: pytest.MonkeyPatch) -> _FakeServer:
    fake = _FakeServer()
    monkeypatch.setattr("dimos.agents.mcp.mcp_adapter.requests.post", fake.post)
    return fake


def _tools(agent: Agent) -> list[Any]:
    mcp = agent._resolve_mcp()
    assert mcp is not None
    return agent._fetch_tools(mcp)


def _tools_after_listing(agent: Agent, server: _FakeServer) -> Any:
    """The `add` tool, listed before `server.payload` starts failing."""
    failing = server.payload
    server.payload = _mock_payload
    add_tool = next(t for t in _tools(agent) if t.name == "add")
    server.payload = failing
    return add_tool


@pytest.mark.flaky(reruns=1)
def test_mcp_endpoint_is_configured_else_the_blueprints_server_else_none() -> None:
    blueprint_server = SimpleNamespace(mcp_url=lambda: "http://127.0.0.1:9992/mcp")
    agent = Agent()
    try:
        assert agent._resolve_mcp() is None
        agent._mcp_server = blueprint_server
        mcp = agent._resolve_mcp()
        assert mcp is not None and mcp.url == "http://127.0.0.1:9992/mcp"
    finally:
        agent.stop()
    agent = Agent(mcp_server_url="http://configured/mcp")
    try:
        agent._mcp_server = blueprint_server
        mcp = agent._resolve_mcp()
        assert mcp is not None and mcp.url == "http://configured/mcp"
    finally:
        agent.stop()


def test_agent_without_an_mcp_server_runs_with_no_tools() -> None:
    agent = Agent(model="gpt-4o")
    agent._thread = MagicMock()
    agent._thread.is_alive.return_value = True
    try:
        with (
            patch("langchain.agents.create_agent") as create,
            patch("dimos.agents.agent.init_model"),
        ):
            agent.on_system_modules([])
        assert create.call_args.kwargs["tools"] == []

        agent.dispatch_continuation({"tool": "add"}, {})
        assert "not found" in str(agent._message_queue.get_nowait().content)
    finally:
        agent.stop()


@pytest.fixture
def agent(server: _FakeServer) -> Iterator[Agent]:
    """An Agent whose MCP requests are answered by `server`."""
    agent = Agent(mcp_server_url="http://localhost:9990/mcp")
    try:
        yield agent
    finally:
        agent.stop()


def test_fetch_tools_from_mcp_server(agent: Agent) -> None:
    tools = _tools(agent)

    assert len(tools) == 2
    assert tools[0].name == "add"
    assert tools[1].name == "greet"


def test_tool_invocation_via_mcp(agent: Agent) -> None:
    tools = _tools(agent)
    add_tool = next(t for t in tools if t.name == "add")
    greet_tool = next(t for t in tools if t.name == "greet")

    assert add_tool.func(x=2, y=3) == "5"
    assert greet_tool.func(name="Alice") == "Hello, Alice!"


def test_image_result_follows_as_a_tool_tagged_user_turn(agent: Agent, server: _FakeServer) -> None:
    """An image cannot ride the tool response, so it follows as a HumanMessage
    naming its tool, or chat UIs would show it as typed input."""
    image = {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,AAAA"}}

    def image_payload(body: dict[str, object]) -> dict[str, object]:
        if body["method"] != "tools/call":
            return _mock_payload(body)
        return {"jsonrpc": "2.0", "id": body["id"], "result": {"content": [image]}}

    server.payload = image_payload
    greet_tool = next(t for t in _tools(agent) if t.name == "greet")

    reply = greet_tool.func(name="Alice")
    msg: BaseMessage = agent._message_queue.get_nowait()

    assert reply.startswith("Tool call started with UUID: ")
    uuid_ = reply.removeprefix("Tool call started with UUID: ").split(".")[0]
    assert isinstance(msg, HumanMessage)
    assert msg.additional_kwargs[TOOL_ORIGIN_KEY] == "greet"
    assert msg.content == [
        {"type": "text", "text": f"This is the artefact for the 'greet' tool with UUID:={uuid_}."},
        image,
    ]


def test_mcp_error_reaches_the_caller(agent: Agent, server: _FakeServer) -> None:
    server.payload = lambda body: {
        "jsonrpc": "2.0",
        "id": 1,
        "error": {"code": -32601, "message": "Unknown: bad/method"},
    }
    add_tool = _tools_after_listing(agent, server)

    with pytest.raises(McpError, match="Unknown: bad/method"):
        add_tool.func(x=1, y=2)


def test_tool_stream_notification_becomes_human_message(agent: Agent) -> None:
    """A `notifications/message` delivered over LCM becomes a HumanMessage."""
    notification = {
        "jsonrpc": "2.0",
        "method": "notifications/message",
        "params": {
            "level": "info",
            "logger": "follow_person",
            "data": "Person follow stopped: lost track.",
        },
    }
    agent._on_tool_stream_message(notification)

    msg: BaseMessage = agent._message_queue.get_nowait()
    assert isinstance(msg, HumanMessage)
    assert "[tool:follow_person]" in str(msg.content)
    assert "Person follow stopped: lost track." in str(msg.content)


def test_tool_stream_ignores_unrelated_frames(agent: Agent) -> None:
    """Unknown methods and empty bodies are dropped on the floor."""

    agent._on_tool_stream_message({"jsonrpc": "2.0", "method": "notifications/other"})
    agent._on_tool_stream_message(
        {"jsonrpc": "2.0", "method": "notifications/message", "params": {"data": ""}}
    )
    agent._on_tool_stream_message(
        {"jsonrpc": "2.0", "method": "notifications/progress", "params": {"message": ""}}
    )

    with pytest.raises(Empty):
        agent._message_queue.get_nowait()


def test_tool_stream_progress_frame_becomes_human_message(agent: Agent) -> None:
    """A `notifications/progress` frame is routed as a HumanMessage."""

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
    agent._on_tool_stream_message(progress_frame)

    msg: BaseMessage = agent._message_queue.get_nowait()
    assert isinstance(msg, HumanMessage)
    assert str(msg.content) == "[tool:follow_person] Found a person"


def test_tool_call_sends_a_progress_token(agent: Agent, server: _FakeServer) -> None:
    """Every `tools/call` request carries a `_meta.progressToken`."""
    add_tool = next(t for t in _tools(agent) if t.name == "add")

    add_tool.func(x=1, y=2)

    params = server.sent[-1]["params"]
    assert server.sent[-1]["method"] == "tools/call"
    assert isinstance(params, dict)
    assert params["name"] == "add"
    assert params["arguments"] == {"x": 1, "y": 2}
    token = params["_meta"]["progressToken"]
    assert isinstance(token, str) and token


@pytest.fixture
def configured_agent(agent: Agent, monkeypatch: pytest.MonkeyPatch) -> Agent:
    """Prepare an agent for testing model initialization."""
    agent.config.model_fixture = None
    agent.config.system_prompt = "System prompt"
    monkeypatch.setattr(agent, "_fetch_tools", MagicMock(return_value=[]))
    agent._lock = RLock()
    agent._thread = MagicMock()
    agent._thread.is_alive.return_value = True
    return agent


def test_on_system_modules_uses_responses_api_model(
    configured_agent: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Production agents use the Responses API required for Luna tool calls."""
    from langchain_openai import ChatOpenAI

    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    configured_agent.config.model = "gpt-5.6-luna"

    with patch("langchain.agents.create_agent") as create_agent:
        configured_agent.on_system_modules([])

    model = create_agent.call_args.kwargs["model"]
    assert isinstance(model, ChatOpenAI)
    assert model.model_name == "gpt-5.6-luna"
    assert model.use_responses_api is True
    assert model.reasoning == {"effort": "medium", "summary": "auto"}


@pytest.mark.parametrize("model_name", ["gpt-4o", "ollama:qwen3:8b", "huggingface:Qwen/Qwen3-8B"])
def test_on_system_modules_resolves_non_reasoning_models(
    configured_agent: Agent, model_name: str
) -> None:
    """Models without Responses reasoning support use provider resolution."""
    configured_agent.config.model = model_name
    resolved_model = MagicMock()

    with (
        patch("langchain.agents.create_agent"),
        patch("langchain.chat_models.init_chat_model", return_value=resolved_model) as init,
    ):
        configured_agent.on_system_modules([])

    init.assert_called_once_with(model=model_name)


def test_set_trace_dir_rebuilds_the_model_with_capture(
    configured_agent: Agent,
) -> None:
    """Evals repoint raw LLM capture per case; the model must be rebuilt so
    the HTTP hook writes under the new directory. Before the agent exists
    the path is only stored for the first build."""
    configured_agent.config.model = "gpt-4o"
    resolved = MagicMock()

    with (
        patch("langchain.agents.create_agent") as create_agent,
        patch("dimos.agents.agent.init_model", return_value=resolved) as init,
    ):
        configured_agent.set_trace_dir("/eval/case/raw")  # no agent yet: stored only
        assert init.call_count == 0

        configured_agent.on_system_modules([])
        assert init.call_args.kwargs["trace_dir"] == Path("/eval/case/raw")

        configured_agent.set_trace_dir(None)
        assert init.call_args.kwargs["trace_dir"] is None
        assert create_agent.call_count == 2


def _call(name: str, call_id: str, **args: Any) -> dict[str, Any]:
    return {"name": name, "args": args, "id": call_id, "type": "tool_call"}


class _FailsOnceGraph:
    """Fake agent graph that records the messages each turn is given in `seen`.

    The first turn streams `first_turn`, then times out; later turns answer "done".
    """

    def __init__(self, first_turn: list[BaseMessage]) -> None:
        self.first_turn = first_turn
        self.seen: list[list[BaseMessage]] = []

    def stream(
        self, state: dict[str, list[BaseMessage]], stream_mode: str
    ) -> Iterator[dict[str, Any]]:
        self.seen.append(list(state["messages"]))
        if len(self.seen) == 1:
            for message in self.first_turn:
                yield {"node": {"messages": [message]}}
            raise TimeoutError("LLM API timed out")
        yield {"model": {"messages": [AIMessage(content="done")]}}


@pytest.fixture
def agent_thread(agent: Agent) -> Iterator[Thread]:
    """Run Agent's message loop, which the `agent` fixture doesn't start."""
    thread = Thread(target=agent._thread_loop, daemon=True)
    thread.start()
    try:
        yield thread
    finally:
        agent._stop_event.set()
        thread.join(timeout=2)


def _send_first_and_second(
    agent: Agent, agent_thread: Thread, graph: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Run `graph` on the messages "first" and "second", and wait for "done"."""
    monkeypatch.setattr(agent, "_state_graph", graph)
    agent._message_queue.put(HumanMessage("first"))
    agent._message_queue.put(HumanMessage("second"))

    def done_or_thread_dead() -> bool:
        done = bool(agent._history) and agent._history[-1].content == "done"
        return done or not agent_thread.is_alive()

    wait_until(
        done_or_thread_dead,
        timeout=5,
        interval=0.01,
        message="the second message was never processed",
    )
    assert agent_thread.is_alive(), "the agent thread died"


def test_a_failed_turn_does_not_stop_the_agent_thread(
    agent: Agent, agent_thread: Thread, monkeypatch: pytest.MonkeyPatch
) -> None:
    call = AIMessage(content="", tool_calls=[_call("add", "c1")])
    graph = _FailsOnceGraph([call])

    _send_first_and_second(agent, agent_thread, graph, monkeypatch)

    second_turn = graph.seen[1]
    first, kept_call, result, note, second, done = agent._history
    assert (first.content, kept_call) == ("first", call)
    assert isinstance(result, ToolMessage)
    assert (result.tool_call_id, result.status) == ("c1", "error")
    assert result.content == (
        "No result: the agent turn failed with TimeoutError: LLM API timed out. "
        "The call may or may not have run."
    )
    assert note.content == "This turn failed with TimeoutError: LLM API timed out"
    assert note.additional_kwargs == {TURN_FAILED_KEY: True}
    # Append-only: the next turn sees everything.
    assert second_turn == agent._history[:5]
    assert (second.content, done.content) == ("second", "done")


def test_only_unanswered_tool_calls_get_a_failure_result(
    agent: Agent, agent_thread: Thread, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A call that returned keeps its real result, so the model knows it ran."""
    both = AIMessage(content="", tool_calls=[_call("add", "c1"), _call("greet", "c2")])
    real_result = ToolMessage(content="3", tool_call_id="c1")
    graph = _FailsOnceGraph([both, real_result])

    _send_first_and_second(agent, agent_thread, graph, monkeypatch)

    first, kept_both, kept_result, failure_result, note = graph.seen[1][:5]
    assert (kept_both, kept_result) == (both, real_result)
    assert isinstance(failure_result, ToolMessage)
    assert (failure_result.tool_call_id, failure_result.status) == ("c2", "error")
    assert note.content == "This turn failed with TimeoutError: LLM API timed out"


def test_a_tool_call_the_mcp_server_never_answers_does_not_stop_the_agent(
    agent: Agent,
    server: _FakeServer,
    agent_thread: Thread,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A failed MCP request escapes the real agent graph; a skill error would be a result."""
    tools = _tools(agent)

    def unreachable(body: dict[str, object]) -> dict[str, object]:
        raise requests.ConnectionError("MCP server unreachable")

    server.payload = unreachable
    call = _call("add", "c1", x=1, y=2)
    model = MockModel(
        responses=[AIMessage(content="", tool_calls=[call]), AIMessage(content="done")]
    )
    _send_first_and_second(
        agent,
        agent_thread,
        create_agent(model=model, tools=tools, system_prompt=""),
        monkeypatch,
    )

    first, kept_call, result, note, second, done = agent._history
    assert (first.content, second.content, done.content) == ("first", "second", "done")
    assert isinstance(kept_call, AIMessage)
    assert [c["id"] for c in kept_call.tool_calls] == ["c1"]
    assert isinstance(result, ToolMessage)
    assert (result.tool_call_id, result.status) == ("c1", "error")
    assert note.content == "This turn failed with ConnectionError: MCP server unreachable"
