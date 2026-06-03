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

import json
from threading import Event, Thread
import time

import httpx
from langchain_core.messages import HumanMessage
import pytest

from dimos.agents import annotation as annotation_module
from dimos.agents.annotation import skill
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.mcp.tool_stream import (
    NOTIFICATIONS_MESSAGE_METHOD,
    NOTIFICATIONS_PROGRESS_METHOD,
    ToolStream,
    make_notification,
    make_progress_notification,
)
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class StreamingModule(Module):
    """Returns a status string immediately and fires `count` updates from a
    background thread."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._stream_threads: list[Thread] = []
        self._stop_streaming = Event()

    @skill
    def start_streaming(self, count: int) -> str:
        """Starts streaming count updates back to the agent."""
        self.start_tool("start_streaming")

        def _stream_loop() -> None:
            try:
                for i in range(count):
                    if self._stop_streaming.wait(timeout=0.1):
                        return
                    self.tool_update("start_streaming", f"Update {i + 1} of {count}")
            finally:
                self.stop_tool("start_streaming")

        thread = Thread(target=_stream_loop, daemon=True)
        self._stream_threads.append(thread)
        thread.start()
        return f"Started streaming {count} updates."

    @rpc
    def stop(self) -> None:
        self._stop_streaming.set()
        for thread in self._stream_threads:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()


@pytest.fixture
def mcp_server():
    """Start a blueprint with StreamingModule + McpServer, wait for readiness."""
    global_config.update(viewer="none")
    blueprint = autoconnect(StreamingModule.blueprint(), McpServer.blueprint())
    coordinator = ModuleCoordinator.build(blueprint)

    adapter = McpAdapter()
    if not adapter.wait_for_ready(timeout=15):
        coordinator.stop()
        pytest.fail("MCP server did not become ready")

    yield adapter

    coordinator.stop()


_NOTIFICATION_METHODS = {NOTIFICATIONS_MESSAGE_METHOD, NOTIFICATIONS_PROGRESS_METHOD}


def _frame_tool_name(frame: dict) -> str | None:
    params = frame.get("params") or {}
    if frame.get("method") == NOTIFICATIONS_PROGRESS_METHOD:
        return (params.get("_meta") or {}).get("tool_name")
    return params.get("logger")


def _read_sse_notifications(
    url: str,
    expected: int,
    timeout: float = 10.0,
    tool_name: str | None = None,
) -> list[dict]:
    """Open `GET url` as SSE and collect `expected` notification frames.

    Returns both `notifications/message` and `notifications/progress` frames.
    Callers can disambiguate by `method`.
    """
    collected: list[dict] = []
    deadline = time.monotonic() + timeout
    with httpx.Client(timeout=timeout) as client:
        with client.stream(
            "GET",
            url,
            headers={"Accept": "text/event-stream"},
        ) as response:
            assert response.headers["content-type"].startswith("text/event-stream")
            for line in response.iter_lines():
                if time.monotonic() > deadline:
                    break
                if not line or not line.startswith("data: "):
                    continue
                try:
                    data = json.loads(line[6:])
                except json.JSONDecodeError:
                    continue
                if data.get("method") not in _NOTIFICATION_METHODS:
                    continue
                if tool_name is not None and _frame_tool_name(data) != tool_name:
                    continue
                collected.append(data)
                if len(collected) >= expected:
                    return collected
    return collected


@pytest.fixture
def background_threads():
    threads: list[Thread] = []
    yield threads
    for thread in threads:
        thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


def test_tool_stream_persistent_sse(
    mcp_server: McpAdapter, background_threads: list[Thread]
) -> None:
    """Tool-stream updates flow through the persistent GET /mcp SSE channel as
    `notifications/progress` frames bound to the client's `progressToken`,
    including updates fired from a background thread after the POST response for
    `tools/call` has already been sent."""
    adapter = mcp_server
    adapter.initialize()

    notifications_ready = Event()
    notifications = []

    def _collect() -> None:
        result = _read_sse_notifications(
            adapter.url, expected=3, timeout=15.0, tool_name="start_streaming"
        )
        notifications.extend(result)
        notifications_ready.set()

    reader = Thread(target=_collect, daemon=True)
    background_threads.append(reader)
    reader.start()

    # Give the GET SSE subscriber a moment to attach before firing the tool.
    time.sleep(0.2)

    progress_token = "pt-test-stream"
    result = adapter.call(
        "tools/call",
        {
            "name": "start_streaming",
            "arguments": {"count": 3},
            "_meta": {"progressToken": progress_token},
        },
    )
    assert "Started streaming" in result["result"]["content"][0]["text"]

    assert notifications_ready.wait(timeout=15.0), "Did not receive notifications in time"

    assert len(notifications) == 3
    assert [n["method"] for n in notifications] == [NOTIFICATIONS_PROGRESS_METHOD] * 3
    assert [n["params"]["progressToken"] for n in notifications] == [progress_token] * 3
    assert [n["params"]["progress"] for n in notifications] == [1, 2, 3]
    assert [n["params"]["message"] for n in notifications] == [
        "Update 1 of 3",
        "Update 2 of 3",
        "Update 3 of 3",
    ]
    for n in notifications:
        assert n["params"]["_meta"]["tool_name"] == "start_streaming"


def test_tool_stream_agent(agent_setup) -> None:
    """Tool stream updates arrive at the agent as HumanMessages."""
    history = agent_setup(
        blueprints=[StreamingModule.blueprint()],
        messages=[
            HumanMessage("Start streaming 3 updates using the start_streaming tool with count=3.")
        ],
    )

    # agent_setup returns after the initial tool call round-trip.  The tool
    # stream updates arrive asynchronously afterwards — poll the history list
    # (which is still being mutated by the /agent transport callback).
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        stream_updates = [
            m
            for m in history
            if isinstance(m, HumanMessage) and "[tool:start_streaming]" in str(m.content)
        ]
        if len(stream_updates) >= 3:
            break
        time.sleep(0.2)

    stream_updates = [
        m
        for m in history
        if isinstance(m, HumanMessage) and "[tool:start_streaming]" in str(m.content)
    ]
    assert len(stream_updates) == 3
    assert "Update 1 of 3" in stream_updates[0].content
    assert "Update 2 of 3" in stream_updates[1].content
    assert "Update 3 of 3" in stream_updates[2].content


@pytest.fixture()
def skill_context():
    """Save/restore `_SKILL_CONTEXT.context` around a test.

    On entry the context is set to `{}` so the body runs inside a simulated
    @skill call with no progress token (the most common case). The yielded
    setter lets tests override that. Pass `None` for "outside any skill" or a
    dict with a `progress_token` for the progress-notification path.
    """
    previous = getattr(annotation_module._SKILL_CONTEXT, "context", None)
    annotation_module._SKILL_CONTEXT.context = {}

    def set_context(value):
        annotation_module._SKILL_CONTEXT.context = value

    yield set_context

    annotation_module._SKILL_CONTEXT.context = previous


@pytest.fixture()
def stream_with_transport_mock(mocker, skill_context):
    """ToolStream wired to a mock pLCMTransport so unit tests can inspect publishes.

    Constructs the stream inside a simulated `@skill` context with no progress
    token, so the strict-construction rule is satisfied and the stream takes the
    `notifications/message` fallback path.
    """
    mock_transport = mocker.MagicMock()
    mocker.patch("dimos.agents.mcp.tool_stream.pLCMTransport", return_value=mock_transport)
    stream = ToolStream("test_tool")
    return stream, mock_transport


def test_tool_stream_outside_skill_raises(skill_context) -> None:
    """Constructing a ToolStream outside any @skill call raises RuntimeError."""
    skill_context(None)
    with pytest.raises(RuntimeError, match="must be constructed inside a @skill"):
        ToolStream("out_of_context")


def test_send_publishes_notification(stream_with_transport_mock) -> None:
    stream, mock_transport = stream_with_transport_mock
    stream.send("hello")
    mock_transport.start.assert_called_once()
    mock_transport.publish.assert_called_once()
    frame = mock_transport.publish.call_args.args[0]
    assert frame["method"] == NOTIFICATIONS_MESSAGE_METHOD
    assert frame["params"]["logger"] == "test_tool"
    assert frame["params"]["data"] == "hello"


def test_send_after_stop_does_not_raise(stream_with_transport_mock) -> None:
    stream, mock_transport = stream_with_transport_mock
    stream.stop()
    stream.send("should be ignored")
    mock_transport.publish.assert_not_called()


def test_stop_tears_down_transport(stream_with_transport_mock) -> None:
    stream, mock_transport = stream_with_transport_mock
    stream.send("kick off transport")
    stream.stop()
    mock_transport.stop.assert_called_once()


def test_stop_without_send_does_nothing(stream_with_transport_mock) -> None:
    stream, mock_transport = stream_with_transport_mock
    stream.stop()
    mock_transport.start.assert_not_called()
    mock_transport.stop.assert_not_called()


def test_double_stop_is_safe(stream_with_transport_mock) -> None:
    stream, mock_transport = stream_with_transport_mock
    stream.send("hello")
    stream.stop()
    stream.stop()
    assert stream.is_closed
    mock_transport.stop.assert_called_once()


def test_make_notification_shape() -> None:
    assert make_notification("greet", "hi") == {
        "jsonrpc": "2.0",
        "method": NOTIFICATIONS_MESSAGE_METHOD,
        "params": {"level": "info", "logger": "greet", "data": "hi"},
    }


def test_make_progress_notification_shape() -> None:
    frame = make_progress_notification(
        "pt-abc", progress=2, message="Halfway", tool_name="fan", total=5
    )
    assert frame == {
        "jsonrpc": "2.0",
        "method": NOTIFICATIONS_PROGRESS_METHOD,
        "params": {
            "progressToken": "pt-abc",
            "progress": 2,
            "total": 5,
            "message": "Halfway",
            "_meta": {"tool_name": "fan"},
        },
    }


@pytest.fixture()
def stream_with_progress_context(mocker, skill_context):
    """ToolStream constructed with a skill-context progress_token set."""
    mock_transport = mocker.MagicMock()
    mocker.patch("dimos.agents.mcp.tool_stream.pLCMTransport", return_value=mock_transport)
    skill_context({"progress_token": "pt-unit-1"})
    stream = ToolStream("progress_tool")
    return stream, mock_transport


def test_send_with_progress_token_emits_progress_notifications(
    stream_with_progress_context,
) -> None:
    stream, mock_transport = stream_with_progress_context
    stream.send("one")
    stream.send("two")
    stream.send("three")

    assert mock_transport.publish.call_count == 3
    frames = [call.args[0] for call in mock_transport.publish.call_args_list]
    assert [f["method"] for f in frames] == [NOTIFICATIONS_PROGRESS_METHOD] * 3
    assert [f["params"]["progressToken"] for f in frames] == ["pt-unit-1"] * 3
    assert [f["params"]["progress"] for f in frames] == [1, 2, 3]
    assert [f["params"]["message"] for f in frames] == ["one", "two", "three"]
    assert all(f["params"]["_meta"]["tool_name"] == "progress_tool" for f in frames)


def test_send_without_progress_token_falls_back_to_message(
    stream_with_transport_mock,
) -> None:
    stream, mock_transport = stream_with_transport_mock
    stream.send("hi")
    frame = mock_transport.publish.call_args.args[0]
    assert frame["method"] == NOTIFICATIONS_MESSAGE_METHOD
    assert frame["params"]["logger"] == "test_tool"


def test_skill_wrapper_sets_and_restores_context() -> None:
    """The @skill wrapper pops _mcp_context and save/restores the thread-local."""
    observed: list[dict | None] = []

    @skill
    def inner() -> str:
        observed.append(annotation_module.current_skill_context())
        return "ok"

    assert annotation_module.current_skill_context() is None
    assert inner(_mcp_context={"progress_token": "pt-X"}) == "ok"
    assert observed == [{"progress_token": "pt-X"}]
    # Restored to None after the call.
    assert annotation_module.current_skill_context() is None


def test_skill_wrapper_without_context_still_sets_empty_dict() -> None:
    """Even without an _mcp_context kwarg, inside the call the context is {}.

    This lets ToolStream distinguish "inside a skill with no client token"
    (construct OK, fall back to notifications/message) from "outside any
    skill entirely" (RuntimeError).
    """
    observed: list[dict | None] = []

    @skill
    def inner() -> None:
        observed.append(annotation_module.current_skill_context())

    assert annotation_module.current_skill_context() is None
    inner()
    assert observed == [{}]
    assert annotation_module.current_skill_context() is None


def test_skill_wrapper_nested_calls_restore_outer_context() -> None:
    """Nested @skill calls on the same thread must not clobber the outer frame."""
    outer_seen: list[dict | None] = []

    @skill
    def inner_skill() -> None:
        pass

    @skill
    def outer_skill() -> None:
        inner_skill(_mcp_context={"progress_token": "inner"})
        outer_seen.append(annotation_module.current_skill_context())

    outer_skill(_mcp_context={"progress_token": "outer"})
    assert outer_seen == [{"progress_token": "outer"}]


class _ToolHelperTestModule(Module):
    """A minimal module used to exercise the start_tool/tool_update/stop_tool helpers.

    The `@skill` wrapper establishes the `_SKILL_CONTEXT` for the duration
    of the call, so `self.start_tool(...)` runs under a live context.
    """

    @skill
    def start(self, name: str) -> str:
        self.start_tool(name)
        return "started"

    @skill
    def double_start(self, name: str) -> str:
        self.start_tool(name)
        self.start_tool(name)  # should raise
        return "unreachable"


@pytest.fixture()
def tool_helper_module(mocker):
    """A _ToolHelperTestModule whose ToolStream instances share a mock transport.

    The module is constructed normally; the event-loop thread and LCM RPC
    transport are patched out because these tests only exercise the
    start_tool/tool_update/stop_tool helpers.
    """
    mock_transport = mocker.MagicMock()
    mocker.patch("dimos.agents.mcp.tool_stream.pLCMTransport", return_value=mock_transport)
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc")
    mocker.patch.object(LCMRPC, "start")
    module = _ToolHelperTestModule()
    yield module, mock_transport
    module._close_all_tools()


def test_start_tool_duplicate_raises(tool_helper_module, skill_context) -> None:
    module, _ = tool_helper_module
    module.start_tool("job")
    with pytest.raises(RuntimeError, match="already active"):
        module.start_tool("job")
    module.stop_tool("job")


def test_tool_update_without_start_is_lenient(tool_helper_module) -> None:
    module, mock_transport = tool_helper_module
    # No start_tool call. tool_update should warn and return, not raise.
    module.tool_update("missing", "hello")
    mock_transport.publish.assert_not_called()


def test_stop_tool_without_start_is_noop(tool_helper_module) -> None:
    module, mock_transport = tool_helper_module
    module.stop_tool("missing")
    mock_transport.stop.assert_not_called()


def test_tool_update_routes_to_registered_stream(tool_helper_module, skill_context) -> None:
    module, mock_transport = tool_helper_module
    module.start_tool("job")
    module.tool_update("job", "progress 1")
    module.tool_update("job", "progress 2")
    module.stop_tool("job")

    texts = [c.args[0]["params"]["data"] for c in mock_transport.publish.call_args_list]
    assert texts == ["progress 1", "progress 2"]


def test_stop_tool_pops_from_registry(tool_helper_module, skill_context) -> None:
    module, mock_transport = tool_helper_module
    module.start_tool("job")
    # A send is required for ToolStream to lazily construct the transport;
    # otherwise stop() has nothing to tear down.
    module.tool_update("job", "hello")
    assert "job" in module._tools
    module.stop_tool("job")
    assert "job" not in module._tools
    mock_transport.stop.assert_called_once()


def test_close_all_tools_stops_outstanding(tool_helper_module, skill_context) -> None:
    module, mock_transport = tool_helper_module
    module.start_tool("a")
    module.tool_update("a", "kick a")
    module.start_tool("b")
    module.tool_update("b", "kick b")

    module._close_all_tools()
    assert module._tools == {}
    # Both ToolStream instances share the same mocked transport, so two
    # stop calls landed on it.
    assert mock_transport.stop.call_count == 2
