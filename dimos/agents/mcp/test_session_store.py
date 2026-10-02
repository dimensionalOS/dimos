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
import json
from pathlib import Path
import subprocess
import sys
from threading import Thread
from uuid import uuid4

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
import pytest

import dimos.agents.mcp.session_store as session_store
from dimos.agents.mcp.session_store import AgentSession


@pytest.fixture
def session(tmp_path: Path) -> Iterator[AgentSession]:
    session = AgentSession(tmp_path)
    assert session.start() == []
    try:
        yield session
    finally:
        session.close()


def test_round_trip_preserves_messages_and_metadata(session: AgentSession) -> None:
    messages = [
        HumanMessage(content="你好", id="human-1"),
        AIMessage(
            content="",
            tool_calls=[{"id": "call-1", "name": "observe", "args": {}}],
            response_metadata={"model_name": "test-model"},
            usage_metadata={"input_tokens": 12, "output_tokens": 3, "total_tokens": 15},
        ),
        ToolMessage(content="image follows", tool_call_id="call-1", artifact={"frame": 1}),
        HumanMessage(
            content=[
                {"type": "text", "text": "camera image"},
                {"type": "image_url", "image_url": {"url": "data:image/png;base64,AA=="}},
            ]
        ),
        AIMessage(content="A clear path.", id="ai-1"),
    ]
    session.save(messages)
    session.close()
    restored = AgentSession(session.path.parent, session.session_id.upper())
    try:
        assert restored.start() == messages
        assert restored.session_id == session.session_id
    finally:
        restored.close()


def test_new_sessions_are_distinct(session: AgentSession) -> None:
    other = AgentSession(session.path.parent)
    try:
        assert other.start() == []
        assert other.session_id != session.session_id
        assert other.path.exists() and session.path.exists()
    finally:
        other.close()


@pytest.mark.parametrize("session_id", ["../escape", "", "not-a-uuid", "/tmp/session"])
def test_invalid_id_rejected_without_files(tmp_path: Path, session_id: str) -> None:
    with pytest.raises(ValueError):
        AgentSession(tmp_path, session_id)
    assert list(tmp_path.iterdir()) == []


def test_missing_session_does_not_create_empty_history(tmp_path: Path) -> None:
    session = AgentSession(tmp_path, str(uuid4()))
    with pytest.raises(FileNotFoundError):
        session.start()
    assert not session.path.exists()
    # A failed load releases the lease, allowing another restore attempt.
    with pytest.raises(FileNotFoundError):
        AgentSession(tmp_path, session.session_id).start()


@pytest.mark.parametrize(
    "payload",
    [
        "{invalid json",
        json.dumps({"version": 2, "session_id": str(uuid4()), "messages": []}),
        json.dumps({"version": 1, "session_id": str(uuid4()), "messages": []}),
        json.dumps({"version": 1, "messages": []}),
    ],
)
def test_invalid_checkpoint_is_preserved(tmp_path: Path, payload: str) -> None:
    session = AgentSession(tmp_path, str(uuid4()))
    session.path.write_text(payload)
    for _ in range(2):
        with pytest.raises(ValueError, match="Invalid agent session"):
            session.start()
    assert session.path.read_text() == payload


def test_invalid_message_is_rejected(session: AgentSession) -> None:
    session.close()
    data = json.loads(session.path.read_text())
    data["messages"] = [{"type": "unknown", "data": {}}]
    session.path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="Invalid agent session"):
        AgentSession(session.path.parent, session.session_id).start()


def test_failed_replace_keeps_previous_checkpoint(
    session: AgentSession, monkeypatch: pytest.MonkeyPatch
) -> None:
    session.save([HumanMessage(content="completed")])
    original = session.path.read_bytes()

    def fail_replace(*args: object) -> None:
        raise OSError("disk failure")

    monkeypatch.setattr(session_store.os, "replace", fail_replace)
    with pytest.raises(OSError, match="disk failure"):
        session.save([HumanMessage(content="next turn")])
    assert session.path.read_bytes() == original
    assert list(session.path.parent.glob("*.tmp")) == []


def test_concurrent_restore_is_rejected(session: AgentSession) -> None:
    other = AgentSession(session.path.parent, session.session_id)
    with pytest.raises(RuntimeError, match="already in use"):
        other.start()
    session.close()
    try:
        assert other.start() == []
    finally:
        other.close()


def test_lease_blocks_another_process(session: AgentSession) -> None:
    script = (
        "from pathlib import Path; "
        "from dimos.agents.mcp.session_store import AgentSession; "
        f"AgentSession(Path({str(session.path.parent)!r}), {session.session_id!r}).start()"
    )
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    assert result.returncode != 0
    assert "already in use" in result.stderr


def test_lease_can_be_released_by_worker_thread(session: AgentSession) -> None:
    worker = Thread(target=session.close)
    worker.start()
    worker.join(timeout=5)
    assert not worker.is_alive()
    restored = AgentSession(session.path.parent, session.session_id)
    try:
        assert restored.start() == []
    finally:
        restored.close()


def test_save_requires_lease(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="without its lease"):
        AgentSession(tmp_path).save([])


def test_process_exit_releases_lease(tmp_path: Path) -> None:
    script = (
        "from pathlib import Path; "
        "from langchain_core.messages import HumanMessage; "
        "from dimos.agents.mcp.session_store import AgentSession; "
        f"session = AgentSession(Path({str(tmp_path)!r})); "
        "session.start(); session.save([HumanMessage(content='checkpoint')]); "
        "print(session.session_id)"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=True
    )
    restored = AgentSession(tmp_path, result.stdout.strip())
    try:
        assert restored.start() == [HumanMessage(content="checkpoint")]
    finally:
        restored.close()
