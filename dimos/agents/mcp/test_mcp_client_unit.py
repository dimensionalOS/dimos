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
from threading import Event, RLock, Thread
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, create_autospec, patch

from langchain.agents import create_agent
from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_core.messages.base import BaseMessage
import pytest
import requests

from dimos.agents.agent import Agent, _append_image_to_history
from dimos.agents.mcp.constants import TOOL_ORIGIN_KEY, TURN_FAILED_KEY
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


def _mock_session(payload_fn: Callable[[dict[str, object]], dict[str, object]]) -> MagicMock:
    """Return an autospec'd requests.Session whose .post() replies via payload_fn."""

    def _post(url: str, *, json: dict[str, object], timeout: float | None = None) -> MagicMock:
        resp = create_autospec(requests.Response, instance=True, spec_set=True)
        resp.json.return_value = payload_fn(json)
        return resp

    session = create_autospec(requests.Session, instance=True, spec_set=True)
    session.post.side_effect = _post
    return session


@pytest.mark.flaky(reruns=1)
def test_server_url_is_configured_else_the_blueprints_server_else_none() -> None:
    server = SimpleNamespace(mcp_url=lambda: "http://127.0.0.1:9992/mcp")
    client = Agent()
    try:
        assert client._server_url() is None
        client._mcp_server = server
        assert client._server_url() == "http://127.0.0.1:9992/mcp"
    finally:
        client.stop()
    client = Agent(mcp_server_url="http://configured/mcp")
    try:
        client._mcp_server = server
        assert client._server_url() == "http://configured/mcp"
    finally:
        client.stop()


def test_agent_without_an_mcp_server_runs_with_no_tools() -> None:
    client = Agent(model="gpt-4o")
    client._thread = MagicMock()
    client._thread.is_alive.return_value = True
    try:
        with (
            patch("langchain.agents.create_agent") as create,
            patch("dimos.agents.agent.init_model"),
        ):
            client.on_system_modules([])
        assert create.call_args.kwargs["tools"] == []
    finally:
        client.stop()


@pytest.fixture
def mcp_client() -> Agent:
    """Build an Agent wired to a mock requests session."""
    client = Agent(mcp_server_url="http://localhost:9990/mcp")
    client._http_client = _mock_session(_mock_payload)
    try:
        yield client
    finally:
        client.stop()


def test_fetch_tools_from_mcp_server(mcp_client: Agent) -> None:
    tools = mcp_client._fetch_tools()

    assert len(tools) == 2
    assert tools[0].name == "add"
    assert tools[1].name == "greet"


def test_tool_invocation_via_mcp(mcp_client: Agent) -> None:
    tools = mcp_client._fetch_tools()
    add_tool = next(t for t in tools if t.name == "add")
    greet_tool = next(t for t in tools if t.name == "greet")

    assert add_tool.func(x=2, y=3) == "5"
    assert greet_tool.func(name="Alice") == "Hello, Alice!"


def test_image_result_follows_as_a_tool_tagged_user_turn(mcp_client: Agent) -> None:
    """An image cannot ride the tool response, so it follows as a HumanMessage
    naming its tool, or chat UIs would show it as typed input."""
    image = {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,AAAA"}}

    def image_payload(body: dict[str, object]) -> dict[str, object]:
        if body["method"] != "tools/call":
            return _mock_payload(body)
        return {"jsonrpc": "2.0", "id": body["id"], "result": {"content": [image]}}

    mcp_client._http_client = _mock_session(image_payload)
    greet_tool = next(t for t in mcp_client._fetch_tools() if t.name == "greet")

    reply = greet_tool.func(name="Alice")
    msg: BaseMessage = mcp_client._artifacts.pop()

    assert reply.startswith("Tool call started with UUID: ")
    uuid_ = reply.removeprefix("Tool call started with UUID: ").split(".")[0]
    assert isinstance(msg, HumanMessage)
    assert msg.additional_kwargs[TOOL_ORIGIN_KEY] == "greet"
    assert msg.content == [
        {"type": "text", "text": f"This is the artefact for the 'greet' tool with UUID:={uuid_}."},
        image,
    ]


def test_mcp_request_error_propagation(mcp_client: Agent) -> None:
    def error_payload(body: dict[str, object]) -> dict[str, object]:
        return {
            "jsonrpc": "2.0",
            "id": 1,
            "error": {"code": -32601, "message": "Unknown: bad/method"},
        }

    mcp_client._http_client = _mock_session(error_payload)

    try:
        mcp_client._mcp_request("bad/method")
        raise AssertionError("Expected RuntimeError")
    except RuntimeError as e:
        assert "Unknown: bad/method" in str(e)


def test_tool_stream_notification_becomes_human_message(mcp_client: Agent) -> None:
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
    mcp_client._on_tool_stream_message(notification)

    msg: BaseMessage = mcp_client._message_queue.get_nowait()
    assert isinstance(msg, HumanMessage)
    assert "[tool:follow_person]" in str(msg.content)
    assert "Person follow stopped: lost track." in str(msg.content)


def test_tool_stream_ignores_unrelated_frames(mcp_client: Agent) -> None:
    """Unknown methods and empty bodies are dropped on the floor."""

    mcp_client._on_tool_stream_message({"jsonrpc": "2.0", "method": "notifications/other"})
    mcp_client._on_tool_stream_message(
        {"jsonrpc": "2.0", "method": "notifications/message", "params": {"data": ""}}
    )
    mcp_client._on_tool_stream_message(
        {"jsonrpc": "2.0", "method": "notifications/progress", "params": {"message": ""}}
    )

    with pytest.raises(Empty):
        mcp_client._message_queue.get_nowait()


def test_tool_stream_progress_frame_becomes_human_message(mcp_client: Agent) -> None:
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
    mcp_client._on_tool_stream_message(progress_frame)

    msg: BaseMessage = mcp_client._message_queue.get_nowait()
    assert isinstance(msg, HumanMessage)
    assert str(msg.content) == "[tool:follow_person] Found a person"


def test_mcp_tool_call_sends_progress_token(mcp_client: Agent) -> None:
    """Every `tools/call` request carries a `_meta.progressToken`."""
    captured: dict[str, object] = {}

    def fake_request(method: str, params: dict[str, object] | None = None) -> dict[str, object]:
        captured["method"] = method
        captured["params"] = params
        return {"content": [{"type": "text", "text": "ok"}]}

    with patch.object(mcp_client, "_mcp_request", side_effect=fake_request):
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


@pytest.fixture
def configured_mcp_client(mcp_client: Agent, monkeypatch: pytest.MonkeyPatch) -> Agent:
    """Prepare a client for testing agent model initialization."""
    mcp_client.config.model_fixture = None
    mcp_client.config.system_prompt = "System prompt"
    monkeypatch.setattr(mcp_client, "_fetch_tools", MagicMock(return_value=[]))
    mcp_client._lock = RLock()
    mcp_client._thread = MagicMock()
    mcp_client._thread.is_alive.return_value = True
    return mcp_client


def test_on_system_modules_uses_responses_api_model(
    configured_mcp_client: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Production agents use the Responses API required for Luna tool calls."""
    from langchain_openai import ChatOpenAI

    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    configured_mcp_client.config.model = "gpt-5.6-luna"

    with patch("langchain.agents.create_agent") as create_agent:
        configured_mcp_client.on_system_modules([])

    model = create_agent.call_args.kwargs["model"]
    assert isinstance(model, ChatOpenAI)
    assert model.model_name == "gpt-5.6-luna"
    assert model.use_responses_api is True
    assert model.reasoning == {"effort": "medium", "summary": "auto"}


@pytest.mark.parametrize("model_name", ["gpt-4o", "ollama:qwen3:8b", "huggingface:Qwen/Qwen3-8B"])
def test_on_system_modules_resolves_non_reasoning_models(
    configured_mcp_client: Agent, model_name: str
) -> None:
    """Models without Responses reasoning support use provider resolution."""
    configured_mcp_client.config.model = model_name
    resolved_model = MagicMock()

    with (
        patch("langchain.agents.create_agent"),
        patch("langchain.chat_models.init_chat_model", return_value=resolved_model) as init,
    ):
        configured_mcp_client.on_system_modules([])

    init.assert_called_once_with(model=model_name)


def test_the_run_log_names_the_model_backend_and_trace_dir(
    configured_mcp_client: Agent,
) -> None:
    """Release tests switch models between runs, so the log must say which one answered."""

    class ChatOllama:
        pass

    configured_mcp_client.config.model = "ollama:qwen3:8b"
    configured_mcp_client.config.trace_dir = Path("/eval/case/raw")

    with (
        patch("langchain.agents.create_agent"),
        patch("dimos.agents.agent.init_model", return_value=ChatOllama()),
        patch("dimos.agents.agent.logger") as logger,
    ):
        configured_mcp_client.on_system_modules([])

    logger.info.assert_called_once_with(
        "Agent model ready.",
        model="ollama:qwen3:8b",
        backend="ChatOllama",
        trace_dir="/eval/case/raw",
    )


def test_set_trace_dir_rebuilds_the_model_with_capture(
    configured_mcp_client: Agent,
) -> None:
    """Evals repoint raw LLM capture per case; the model must be rebuilt so
    the HTTP hook writes under the new directory. Before the agent exists
    the path is only stored for the first build."""
    configured_mcp_client.config.model = "gpt-4o"
    resolved = MagicMock()

    with (
        patch("langchain.agents.create_agent") as create_agent,
        patch("dimos.agents.agent.init_model", return_value=resolved) as init,
    ):
        configured_mcp_client.set_trace_dir("/eval/case/raw")  # no agent yet: stored only
        assert init.call_count == 0

        configured_mcp_client.on_system_modules([])
        assert init.call_args.kwargs["trace_dir"] == Path("/eval/case/raw")

        configured_mcp_client.set_trace_dir(None)
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
def agent_thread(mcp_client: Agent) -> Iterator[Thread]:
    """Run Agent's message loop, which the `mcp_client` fixture doesn't start."""
    thread = Thread(target=mcp_client._thread_loop, daemon=True)
    thread.start()
    try:
        yield thread
    finally:
        mcp_client._stop_event.set()
        thread.join(timeout=2)


def _send_first_and_second(
    mcp_client: Agent, agent_thread: Thread, graph: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Run `graph` on the messages "first" and "second", and wait for "done"."""
    monkeypatch.setattr(mcp_client, "_state_graph", graph)
    mcp_client._message_queue.put(HumanMessage("first"))
    mcp_client._message_queue.put(HumanMessage("second"))

    def done_or_thread_dead() -> bool:
        done = bool(mcp_client._history) and mcp_client._history[-1].content == "done"
        return done or not agent_thread.is_alive()

    wait_until(
        done_or_thread_dead,
        timeout=5,
        interval=0.01,
        message="the second message was never processed",
    )
    assert agent_thread.is_alive(), "the agent thread died"


def test_a_failed_turn_does_not_stop_the_agent_thread(
    mcp_client: Agent, agent_thread: Thread, monkeypatch: pytest.MonkeyPatch
) -> None:
    call = AIMessage(content="", tool_calls=[_call("add", "c1")])
    graph = _FailsOnceGraph([call])

    _send_first_and_second(mcp_client, agent_thread, graph, monkeypatch)

    second_turn = graph.seen[1]
    first, kept_call, result, note, second, done = mcp_client._history
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
    assert second_turn == mcp_client._history[:5]
    assert (second.content, done.content) == ("second", "done")


def test_only_unanswered_tool_calls_get_a_failure_result(
    mcp_client: Agent, agent_thread: Thread, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A call that returned keeps its real result, so the model knows it ran."""
    both = AIMessage(content="", tool_calls=[_call("add", "c1"), _call("greet", "c2")])
    real_result = ToolMessage(content="3", tool_call_id="c1")
    graph = _FailsOnceGraph([both, real_result])

    _send_first_and_second(mcp_client, agent_thread, graph, monkeypatch)

    first, kept_both, kept_result, failure_result, note = graph.seen[1][:5]
    assert (kept_both, kept_result) == (both, real_result)
    assert isinstance(failure_result, ToolMessage)
    assert (failure_result.tool_call_id, failure_result.status) == ("c2", "error")
    assert note.content == "This turn failed with TimeoutError: LLM API timed out"


def test_a_tool_call_the_mcp_server_never_answers_does_not_stop_the_agent(
    mcp_client: Agent, agent_thread: Thread, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A failed MCP request escapes the real agent graph; a skill error would be a result."""
    tools = mcp_client._fetch_tools()

    def unreachable(body: dict[str, object]) -> dict[str, object]:
        raise requests.ConnectionError("MCP server unreachable")

    mcp_client._http_client = _mock_session(unreachable)
    call = _call("add", "c1", x=1, y=2)
    model = MockModel(
        responses=[AIMessage(content="", tool_calls=[call]), AIMessage(content="done")]
    )
    _send_first_and_second(
        mcp_client,
        agent_thread,
        create_agent(model=model, tools=tools, system_prompt=""),
        monkeypatch,
    )

    first, kept_call, result, note, second, done = mcp_client._history
    assert (first.content, second.content, done.content) == ("first", "second", "done")
    assert isinstance(kept_call, AIMessage)
    assert [c["id"] for c in kept_call.tool_calls] == ["c1"]
    assert isinstance(result, ToolMessage)
    assert (result.tool_call_id, result.status) == ("c1", "error")
    assert note.content == "This turn failed with ConnectionError: MCP server unreachable"


class _AnswersGraph:
    """Fake agent graph that streams `turn` for every message, then "turn N" once it runs out."""

    def __init__(self, *turn: BaseMessage) -> None:
        self.turn = turn
        self.turns = 0

    def stream(
        self, state: dict[str, list[BaseMessage]], stream_mode: str
    ) -> Iterator[dict[str, Any]]:
        self.turns += 1
        for message in self.turn or (AIMessage(content=f"turn {self.turns}"),):
            yield {"model": {"messages": [message]}}


@pytest.fixture
def running_agent(
    mcp_client: Agent, agent_thread: Thread, monkeypatch: pytest.MonkeyPatch
) -> Agent:
    """An agent whose message loop runs, as it does once its blueprint has started."""
    monkeypatch.setattr(mcp_client, "_thread", agent_thread)
    return mcp_client


def test_infer_returns_the_final_answer_of_its_turn(
    mcp_client: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    graph = _AnswersGraph(
        AIMessage(content="", tool_calls=[_call("add", "c1", x=1, y=2)]),
        ToolMessage(content="3", tool_call_id="c1"),
        AIMessage(content=[{"type": "text", "text": "It is "}, {"type": "text", "text": "3."}]),
    )
    monkeypatch.setattr(mcp_client, "_state_graph", graph)

    assert mcp_client.infer(HumanMessage("what is 1 + 2?")).text == "It is 3."
    assert mcp_client._history[0].content == "what is 1 + 2?"


def test_infer_raises_when_its_turn_fails(
    mcp_client: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(mcp_client, "_state_graph", _FailsOnceGraph([]))

    with pytest.raises(RuntimeError, match="TimeoutError: LLM API timed out"):
        mcp_client.infer(HumanMessage("first"))
    assert mcp_client.infer(HumanMessage("second")).text == "done"


def test_infer_needs_a_mcp_client(mcp_client: Agent) -> None:
    with pytest.raises(RuntimeError, match="not running"):
        mcp_client.infer(HumanMessage("too early"))


class _EchoGraph:
    """Answers each message with its own text, after a short pause."""

    def stream(
        self, state: dict[str, list[BaseMessage]], stream_mode: str
    ) -> Iterator[dict[str, Any]]:
        question = state["messages"][-1].content
        time.sleep(0.002)
        yield {"model": {"messages": [AIMessage(content=f"answer to {question}")]}}


def test_infer_under_load_gives_every_caller_its_own_answer(
    running_agent: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Concurrent infer() calls with stream messages arriving among them."""
    monkeypatch.setattr(running_agent, "_state_graph", _EchoGraph())
    answers: dict[str, str] = {}

    def ask(question: str) -> None:
        answers[question] = str(running_agent.infer(HumanMessage(question)).text)

    questions = [f"q{i}" for i in range(20)]
    callers = [Thread(target=ask, args=(question,)) for question in questions]
    same = HumanMessage("same object")
    callers += [Thread(target=running_agent.infer, args=(same,)) for _ in range(5)]
    for i, caller in enumerate(callers):
        caller.start()
        running_agent.add_message(HumanMessage(f"stream {i}"))
    for caller in callers:
        caller.join(timeout=30)

    assert answers == {question: f"answer to {question}" for question in questions}
    assert not any(caller.is_alive() for caller in callers)
    wait_until(lambda: len(running_agent._history) == 100, timeout=5, interval=0.01)
    # Turns never interleave: every question is followed by its own answer.
    history = [str(message.content) for message in running_agent._history]
    assert history[1::2] == [f"answer to {question}" for question in history[::2]]


class _HeldGraph(_AnswersGraph):
    """Starts a turn, then holds it until released."""

    def __init__(self) -> None:
        super().__init__()
        self.started = Event()
        self.release = Event()

    def stream(
        self, state: dict[str, list[BaseMessage]], stream_mode: str
    ) -> Iterator[dict[str, Any]]:
        self.started.set()
        self.release.wait(timeout=5)
        yield from super().stream(state, stream_mode)


def test_infer_called_during_a_turn_raises_once_the_agent_stops(
    running_agent: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    graph = _HeldGraph()
    monkeypatch.setattr(running_agent, "_state_graph", graph)
    errors: list[BaseException] = []

    def ask() -> None:
        try:
            running_agent.infer(HumanMessage("never answered"))
        except RuntimeError as error:
            errors.append(error)

    running_agent.add_message(HumanMessage("a turn from the stream"))
    assert graph.started.wait(timeout=5)
    caller = Thread(target=ask)
    caller.start()
    stopper = Thread(target=running_agent.stop)
    stopper.start()
    wait_until(running_agent._stop_event.is_set, timeout=5, interval=0.01)
    graph.release.set()
    caller.join(timeout=5)
    stopper.join(timeout=5)

    assert not caller.is_alive() and not stopper.is_alive()
    assert len(errors) == 1 and "not running" in str(errors[0])
    assert len(running_agent._history) == 2


def _image(label: str) -> dict[str, Any]:
    return {"type": "image_url", "image_url": {"url": label}}


def _blocks(message: BaseMessage) -> Any:
    """The content blocks of an image message: [text, image]."""
    return message.content


class _CameraGraph:
    """A model whose tools return images.

    A question makes each tool in `looks[question]` return its images; the model then
    answers "pending". Each image arrives as its own message and is answered "saw <label>".
    An image in `chain` makes one more tool call, and one in `broken` fails its model call.
    """

    def __init__(
        self,
        agent: Agent,
        looks: dict[str, list[tuple[str, list[str]]]],
        chain: dict[str, str] | None = None,
        broken: tuple[str, ...] = (),
    ) -> None:
        self.agent, self.looks, self.chain, self.broken = agent, looks, chain or {}, broken

    def stream(
        self, state: dict[str, list[BaseMessage]], stream_mode: str
    ) -> Iterator[dict[str, Any]]:
        last = state["messages"][-1]
        if TOOL_ORIGIN_KEY not in last.additional_kwargs:
            for tool, labels in self.looks.get(str(last.content), []):
                for label in labels:
                    _append_image_to_history(self.agent, tool, label, _image(label))
            yield {"model": {"messages": [AIMessage(content="pending")]}}
            return
        label = _blocks(last)[1]["image_url"]["url"]
        if label in self.broken:
            raise RuntimeError(f"cannot read {label}")
        if label in self.chain:
            _append_image_to_history(
                self.agent, "zoom", self.chain[label], _image(self.chain[label])
            )
        yield {"model": {"messages": [AIMessage(content=f"saw {label}")]}}


def _transcript(agent: Agent) -> list[str]:
    """The history as short strings: questions, answers, and "<tool>:<label>" for images."""
    lines = []
    for message in agent._history:
        if TOOL_ORIGIN_KEY in message.additional_kwargs:
            url = _blocks(message)[1]["image_url"]["url"]
            lines.append(f"{message.additional_kwargs[TOOL_ORIGIN_KEY]}:{url}")
        else:
            lines.append(str(message.content))
    return lines


@pytest.mark.parametrize(
    "looks, chain, answer, transcript",
    [
        pytest.param(
            [("look", ["a"])], {}, "saw a", ["q", "pending", "look:a", "saw a"], id="one image"
        ),
        pytest.param(
            [("look", ["a", "b", "c"])],
            {},
            "saw c",
            ["q", "pending", "look:a", "saw a", "look:b", "saw b", "look:c", "saw c"],
            id="one tool, three images",
        ),
        pytest.param(
            [("left", ["a"]), ("right", ["b"])],
            {},
            "saw b",
            ["q", "pending", "left:a", "saw a", "right:b", "saw b"],
            id="two tools in one turn",
        ),
        pytest.param(
            [("look", ["a", "b"])],
            {"a": "a2"},
            "saw a2",
            ["q", "pending", "look:a", "saw a", "look:b", "saw b", "zoom:a2", "saw a2"],
            id="an image whose answer asks for another image",
        ),
        pytest.param([], {}, "pending", ["q", "pending"], id="no image"),
    ],
)
def test_infer_answers_after_every_image_its_tools_returned(
    mcp_client: Agent,
    monkeypatch: pytest.MonkeyPatch,
    looks: list[tuple[str, list[str]]],
    chain: dict[str, str],
    answer: str,
    transcript: list[str],
) -> None:
    monkeypatch.setattr(mcp_client, "_state_graph", _CameraGraph(mcp_client, {"q": looks}, chain))

    assert mcp_client.infer(HumanMessage("q")).content == answer
    assert _transcript(mcp_client) == transcript
    assert mcp_client._message_queue.empty() and mcp_client._artifacts == []


def test_infer_raises_when_any_image_of_its_turn_fails(
    mcp_client: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    graph = _CameraGraph(mcp_client, {"q": [("look", ["a", "b"])]}, broken=("a",))
    monkeypatch.setattr(mcp_client, "_state_graph", graph)

    with pytest.raises(RuntimeError, match="cannot read a"):
        mcp_client.infer(HumanMessage("q"))
    assert _transcript(mcp_client)[-2:] == ["look:b", "saw b"], "b was still shown"
    assert mcp_client._artifacts == []

    graph.broken = ()
    assert mcp_client.infer(HumanMessage("q")).content == "saw b"


def test_infer_raises_when_the_turn_has_no_answer(
    mcp_client: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    class _Silent:
        def stream(self, state: Any, stream_mode: str) -> Iterator[dict[str, Any]]:
            return iter(())

    monkeypatch.setattr(mcp_client, "_state_graph", _Silent())
    with pytest.raises(RuntimeError, match="no answer"):
        mcp_client.infer(HumanMessage("q"))


def test_concurrent_infer_calls_each_see_their_own_images(
    mcp_client: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    looks = {f"q{i}": [("look", [f"a{i}", f"b{i}"])] for i in range(10)}
    monkeypatch.setattr(mcp_client, "_state_graph", _CameraGraph(mcp_client, looks))
    answers: dict[str, str] = {}

    def ask(question: str) -> None:
        answers[question] = str(mcp_client.infer(HumanMessage(question)).content)

    callers = [Thread(target=ask, args=(question,)) for question in looks]
    for caller in callers:
        caller.start()
    for caller in callers:
        caller.join(timeout=10)

    assert answers == {f"q{i}": f"saw b{i}" for i in range(10)}
    lines = _transcript(mcp_client)
    for i in range(10):
        at = lines.index(f"q{i}")
        assert lines[at : at + 6] == [
            f"q{i}", "pending", f"look:a{i}", f"saw a{i}", f"look:b{i}", f"saw b{i}"
        ]  # fmt: skip


def test_a_stream_turn_shows_its_images_before_the_next_message(
    running_agent: Agent, monkeypatch: pytest.MonkeyPatch
) -> None:
    graph = _CameraGraph(running_agent, {"q1": [("look", ["a", "b"])]})
    held = Event()
    stream = graph.stream

    def hold_first(state: dict[str, list[BaseMessage]], stream_mode: str) -> Iterator[Any]:
        held.wait(timeout=5)  # both messages are queued before the first turn runs
        return stream(state, stream_mode)

    monkeypatch.setattr(graph, "stream", hold_first)
    monkeypatch.setattr(running_agent, "_state_graph", graph)
    events: list[str] = []
    monkeypatch.setattr(
        running_agent.agent_idle, "publish", lambda idle: events.append(f"idle={idle}")
    )
    monkeypatch.setattr(running_agent.agent, "publish", lambda m: events.append(_line(m)))

    running_agent.add_message(HumanMessage("q1"))
    running_agent.add_message(HumanMessage("q2"))
    held.set()
    wait_until(lambda: len(running_agent._history) == 8, timeout=5, interval=0.01)
    wait_until(lambda: events[-1] == "idle=True", timeout=5, interval=0.01)

    assert _transcript(running_agent) == [
        "q1", "pending", "look:a", "saw a", "look:b", "saw b", "q2", "pending"
    ]  # fmt: skip
    assert "idle=True" not in events[: events.index("pending", events.index("q2"))]


def _line(message: BaseMessage) -> str:
    return "image" if TOOL_ORIGIN_KEY in message.additional_kwargs else str(message.content)


def test_a_tool_result_with_several_images_queues_each_in_order(mcp_client: Agent) -> None:
    """Through the real tool wrapper: text plus two images in one MCP result."""
    images = [_image("a"), _image("b")]

    def payload(body: dict[str, object]) -> dict[str, object]:
        if body["method"] != "tools/call":
            return _mock_payload(body)
        content = [{"type": "text", "text": "two frames. "}, *images]
        return {"jsonrpc": "2.0", "id": body["id"], "result": {"content": content}}

    mcp_client._http_client = _mock_session(payload)
    greet = next(t for t in mcp_client._fetch_tools() if t.name == "greet")
    assert greet.func is not None

    reply = greet.func(name="Alice")

    assert reply.startswith("two frames. ") and reply.count("Tool call started with UUID") == 2
    assert [_blocks(m)[1] for m in mcp_client._artifacts] == images
    for artifact in mcp_client._artifacts:
        uuid_ = str(_blocks(artifact)[0]["text"]).split("UUID:=")[1].rstrip(".")
        assert uuid_ in reply
