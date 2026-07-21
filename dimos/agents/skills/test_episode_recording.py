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

"""Unit tests for EpisodeRecordingSkillContainer.

Constructs the container with its boot side effects (asyncio loop + RPC
transport that `Module.__init__` starts) patched out, binds a stub episode
controller to the injected `_episode` attribute, and calls the skill methods
directly — asserting they forward the right event/label and return the
controller's message.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator

import pytest
import pytest_mock

from dimos.agents.skills.episode_recording import EpisodeRecordingSkillContainer
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class _StubControl:
    """Records set_episode calls; stands in for the injected EpisodeMonitorModule."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, str | None]] = []

    def set_episode(self, event: str, task_label: str | None = None) -> str:
        self.calls.append((event, task_label))
        return f"stub:{event}:{task_label}"


@pytest.fixture
def make_container(
    mocker: pytest_mock.MockerFixture,
) -> Iterator[Callable[[], tuple[EpisodeRecordingSkillContainer, _StubControl]]]:
    """Factory for a container with boot patched out and a stub controller bound."""
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)

    built: list[EpisodeRecordingSkillContainer] = []

    def _make() -> tuple[EpisodeRecordingSkillContainer, _StubControl]:
        c = EpisodeRecordingSkillContainer()
        stub = _StubControl()
        c._episode = stub  # type: ignore[assignment]
        built.append(c)
        return c, stub

    yield _make
    for c in built:
        c.stop()


def test_start_recording_forwards_label(
    make_container: Callable[[], tuple[EpisodeRecordingSkillContainer, _StubControl]],
) -> None:
    c, stub = make_container()
    out = c.start_recording("kitchen run")
    assert stub.calls[-1] == ("start", "kitchen run")
    assert "kitchen run" in out


def test_stop_recording_saves(
    make_container: Callable[[], tuple[EpisodeRecordingSkillContainer, _StubControl]],
) -> None:
    c, stub = make_container()
    c.stop_recording()
    assert stub.calls[-1] == ("save", None)


def test_discard_recording_discards(
    make_container: Callable[[], tuple[EpisodeRecordingSkillContainer, _StubControl]],
) -> None:
    c, stub = make_container()
    c.discard_recording()
    assert stub.calls[-1] == ("discard", None)
