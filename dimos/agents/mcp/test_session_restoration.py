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

from collections.abc import Iterator
from pathlib import Path
from threading import Event
from unittest.mock import MagicMock

from langchain.agents import create_agent
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage
from pydantic import ValidationError
import pytest

import dimos.agents.mcp.mcp_client as client_module
from dimos.agents.mcp.mcp_client import McpClient, McpClientConfig
from dimos.agents.mcp.session_store import AgentSession
from dimos.agents.testing.mock_model import MockModel


@pytest.fixture
def client(tmp_path: Path) -> Iterator[McpClient]:
    client = McpClient(session_dir=tmp_path)
    client.agent.publish = MagicMock()
    client.agent_idle.publish = MagicMock()
    try:
        yield client
    finally:
        client.stop()


def test_constructor_does_not_write_files(client: McpClient, tmp_path: Path) -> None:
    assert list(tmp_path.iterdir()) == []


def test_successful_turn_saved(client: McpClient) -> None:
    client._start_session()
    graph = create_agent(MockModel(responses=["ready"]), tools=[])
    client._process_message(graph, HumanMessage(content="hello"))
    assert client._session is not None
    session_id = client._session.session_id
    history = list(client._history)
    directory = client._session.path.parent
    client.stop()
    restored = AgentSession(directory, session_id)
    try:
        assert restored.start() == history
        assert [message.content for message in history] == ["hello", "ready"]
    finally:
        restored.close()


def test_restore_precedes_queued_input(
    client: McpClient, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    client._start_session()
    graph = create_agent(MockModel(responses=["I will remember"]), tools=[])
    client._process_message(graph, HumanMessage(content="My name is Ada"))
    assert client._session is not None
    session_id = client._session.session_id
    prior_history = list(client._history)
    client.stop()

    finished = Event()
    restored = McpClient(session_dir=tmp_path, restore_session=session_id)
    restored.agent.publish = MagicMock()
    restored.agent_idle.publish = MagicMock(
        side_effect=lambda idle: finished.set() if idle else None
    )
    monkeypatch.setattr(restored, "_fetch_tools", lambda: [])
    monkeypatch.setattr(
        client_module, "init_model", lambda *args, **kwargs: MockModel(responses=["Ada"])
    )
    received: list[list[BaseMessage]] = []
    original_process = restored._process_message

    def process(state_graph, message: BaseMessage) -> None:
        received.append([*restored._history, message])
        original_process(state_graph, message)

    monkeypatch.setattr(restored, "_process_message", process)
    prompt = HumanMessage(content="What is my name?")
    restored.add_message(prompt)
    try:
        restored.on_system_modules([])
        assert finished.wait(10)
        assert received == [[*prior_history, prompt]]
        assert restored._history[-1].content == "Ada"
        assert restored._session is not None
        assert restored._session.session_id == session_id
    finally:
        restored.stop()


def test_failed_turn_keeps_checkpoint(client: McpClient) -> None:
    client._start_session()
    assert client._session is not None
    previous = client._session.path.read_bytes()

    def stream(*args: object, **kwargs: object):
        yield {"model": {"messages": [AIMessage(content="partial")]}}
        raise RuntimeError("model disconnected")

    graph = MagicMock()
    graph.stream.side_effect = stream
    with pytest.raises(RuntimeError, match="model disconnected"):
        client._process_message(graph, HumanMessage(content="new task"))
    assert client._session.path.read_bytes() == previous
    client.agent_idle.publish.assert_called_once_with(False)


def test_persistence_can_be_disabled(client: McpClient, tmp_path: Path) -> None:
    client.config.persist_history = False
    client._start_session()
    graph = create_agent(MockModel(responses=["ready"]), tools=[])
    client._process_message(graph, HumanMessage(content="hello"))
    assert client._session is None
    assert list(tmp_path.iterdir()) == []


def test_stop_retains_lease_until_worker_finishes(
    client: McpClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    entered = Event()
    release = Event()

    def stream(*args: object, **kwargs: object):
        entered.set()
        assert release.wait(10)
        yield {"model": {"messages": [AIMessage(content="done")]}}

    graph = MagicMock()
    graph.stream.side_effect = stream
    client._state_graph = graph
    client._start_session()
    assert client._session is not None
    directory = client._session.path.parent
    session_id = client._session.session_id
    client.add_message(HumanMessage(content="work"))
    client._thread.start()
    try:
        assert entered.wait(10)
        monkeypatch.setattr(client_module, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.01)
        client.stop()
        with pytest.raises(RuntimeError, match="already in use"):
            AgentSession(directory, session_id).start()
    finally:
        release.set()
        client._thread.join(timeout=10)
    assert not client._thread.is_alive()
    restored = AgentSession(directory, session_id)
    try:
        assert [message.content for message in restored.start()] == ["work", "done"]
    finally:
        restored.close()


def test_thread_start_failure_releases_session(
    client: McpClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(client, "_fetch_tools", lambda: [])
    monkeypatch.setattr(client, "_rebuild_agent", lambda: None)
    monkeypatch.setattr(
        client._thread, "start", MagicMock(side_effect=RuntimeError("cannot start"))
    )
    with pytest.raises(RuntimeError, match="cannot start"):
        client.on_system_modules([])
    assert client._session is None


@pytest.mark.parametrize("session_id", ["../escape", "not-a-uuid", ""])
def test_invalid_restore_id_rejected(session_id: str) -> None:
    with pytest.raises(ValidationError):
        McpClientConfig(restore_session=session_id)


def test_restore_requires_persistence() -> None:
    with pytest.raises(ValidationError, match="requires persist_history"):
        McpClientConfig(
            persist_history=False, restore_session="123e4567-e89b-12d3-a456-426614174000"
        )
