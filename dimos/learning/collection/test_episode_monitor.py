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

"""Unit tests for the EpisodeMonitor state machine.

The module is constructed normally; only its boot side effects (the asyncio
loop + RPC transport that `Module.__init__` starts) are patched out, and its
`status` Out port is replaced with a mock so published EpisodeStatus events can
be inspected. Drives the button/keyboard handlers directly and asserts on the
state machine these events feed into `extract_episodes`.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator

import pytest
import pytest_mock

from dimos.learning.collection.episode_monitor import (
    EpisodeMonitorModule,
    EpisodeStatus,
    KeyPress,
)
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons


@pytest.fixture
def make_monitor(
    mocker: pytest_mock.MockerFixture,
) -> Iterator[Callable[..., EpisodeMonitorModule]]:
    """Factory for an EpisodeMonitorModule with its boot patched out.

    `Module.__init__` starts an asyncio loop + RPC transport; patch both so the
    test exercises only the state machine. The `status` port is a mock whose
    `publish` calls record the emitted EpisodeStatus. Every built module is
    stopped on teardown.
    """
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)

    built: list[EpisodeMonitorModule] = []

    def _make(**config: object) -> EpisodeMonitorModule:
        m = EpisodeMonitorModule(**config)
        m.status = mocker.MagicMock()  # type: ignore[assignment]
        built.append(m)
        return m

    yield _make
    for m in built:
        m.stop()


def _events(monitor: EpisodeMonitorModule) -> list[EpisodeStatus]:
    """The EpisodeStatus objects published on the monitor's `status` port."""
    return [call.args[0] for call in monitor.status.publish.call_args_list]  # type: ignore[attr-defined]


def _press(monitor: EpisodeMonitorModule, alias: str) -> None:
    """Rising edge: release-then-press the given Quest button alias."""
    attr = BUTTON_ALIASES[alias]
    released = Buttons()
    pressed = Buttons()
    pressed.set_attribute(attr, True)
    monitor._on_buttons(released)
    monitor._on_buttons(pressed)


def test_toggle_starts_then_saves(make_monitor: Callable[..., EpisodeMonitorModule]) -> None:
    m = make_monitor()  # default map: toggle=B, discard=Y
    _press(m, "B")  # idle → recording
    _press(m, "B")  # recording → idle (saved)

    events = _events(m)
    assert [e.last_event for e in events] == ["start", "save"]
    assert events[-1].state == "idle"
    assert events[-1].episodes_saved == 1
    assert events[-1].episodes_discarded == 0


def test_discard_does_not_count_as_saved(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    _press(m, "B")  # start
    _press(m, "Y")  # discard

    last = _events(m)[-1]
    assert last.state == "idle"
    assert last.episodes_saved == 0
    assert last.episodes_discarded == 1


def test_start_while_recording_autocommits_previous(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    # toggle (start), then an explicit start via keyboard while still recording:
    # the in-progress episode auto-commits (matches the offline extractor).
    m = make_monitor(keyboard_map={"start": "r"})
    _press(m, "B")  # recording
    m._on_keyboard(KeyPress(key="r", ts=2.0))  # start again → auto-commit prior

    last = _events(m)[-1]
    assert last.last_event == "start"
    assert last.state == "recording"
    assert last.episodes_saved == 1  # the auto-committed one


def test_no_event_without_rising_edge(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    pressed = Buttons()
    pressed.right_secondary = True  # B held
    m._on_buttons(pressed)
    m._on_buttons(pressed)  # still held — no new edge
    assert [e.last_event for e in _events(m)] == ["start"]


def test_published_status_is_internally_consistent(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    # Every published event's counters/state must match the event it carries —
    # the snapshot is taken under the same lock as the mutation.
    m = make_monitor()
    _press(m, "B")  # start
    _press(m, "B")  # save  (1)
    _press(m, "B")  # start
    _press(m, "B")  # save  (2)
    _press(m, "B")  # start
    _press(m, "Y")  # discard (1)

    events = _events(m)
    for e in events:
        if e.last_event == "start":
            assert e.state == "recording"
        elif e.last_event in ("save", "discard"):
            assert e.state == "idle"
    assert events[-1].episodes_saved == 2
    assert events[-1].episodes_discarded == 1


def test_reset_counters(make_monitor: Callable[..., EpisodeMonitorModule]) -> None:
    m = make_monitor()
    _press(m, "B")
    _press(m, "B")
    status = m.reset_counters()
    assert status.episodes_saved == 0
    assert status.episodes_discarded == 0
    assert status.state == "idle"
    assert status.last_event == "init"
