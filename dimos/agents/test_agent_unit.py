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
from threading import RLock
from unittest.mock import MagicMock, create_autospec, patch

from langchain_core.messages import HumanMessage
from langchain_core.messages.base import BaseMessage
import pytest
from pytest_mock import MockerFixture
import requests

from dimos.agents.agent import Agent


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


def _mock_session(payload_fn: Callable[[dict[str, object]], dict[str, object]]) -> MagicMock:
    """Return an autospec'd requests.Session whose .post() replies via payload_fn."""

    def _post(url: str, *, json: dict[str, object], timeout: float | None = None) -> MagicMock:
        resp = create_autospec(requests.Response, instance=True, spec_set=True)
        resp.json.return_value = payload_fn(json)
        return resp

    session = create_autospec(requests.Session, instance=True, spec_set=True)
    session.post.side_effect = _post
    return session


@pytest.fixture
def agent(mocker: MockerFixture) -> Iterator[Agent]:
    """An Agent whose MCP requests are answered by a mock session."""
    mocker.patch(
        "dimos.agents.mcp.mcp_client.requests.Session", return_value=_mock_session(_mock_payload)
    )
    agent = Agent(mcp_server_url="http://localhost:9990/mcp")
    yield agent
    agent.stop()


def test_fetch_tools_from_mcp_server(agent: Agent) -> None:
    tools = agent._fetch_tools()

    assert len(tools) == 2
    assert tools[0].name == "add"
    assert tools[1].name == "greet"


def test_tool_invocation_via_mcp(agent: Agent) -> None:
    tools = agent._fetch_tools()
    add_tool = next(t for t in tools if t.name == "add")
    greet_tool = next(t for t in tools if t.name == "greet")

    assert add_tool.func(x=2, y=3) == "5"
    assert greet_tool.func(name="Alice") == "Hello, Alice!"


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


@pytest.fixture
def configured_agent(agent: Agent, monkeypatch: pytest.MonkeyPatch) -> Agent:
    """An agent prepared for testing model initialization."""
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
