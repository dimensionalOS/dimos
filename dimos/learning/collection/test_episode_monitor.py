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
from concurrent.futures import ThreadPoolExecutor
import threading
from typing import cast

import pytest
import pytest_mock

from dimos.learning.collection.episode_monitor import (
    EpisodeCommand,
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


def test_reset_counters_resets_task_label(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    m.set_episode("start", "old task")
    m.reset_counters()
    m.set_episode("start")

    assert _events(m)[-1].task_label is None


def test_episode_command_is_single_runtime_enum() -> None:
    assert {command.value for command in EpisodeCommand} == {
        "start",
        "save",
        "discard",
        "toggle",
    }


# ── set_episode RPC (runtime / agent-driven trigger) ──────────────────────────


def test_set_episode_start_sets_label(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    msg = m.set_episode("start", "kitchen run")
    ev = _events(m)[-1]
    assert ev.last_event == "start"
    assert ev.state == "recording"
    assert ev.task_label == "kitchen run"
    assert "kitchen run" in msg


def test_set_episode_save_carries_label_into_next_take(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    m.set_episode("start", "kitchen run")
    m.set_episode("save")
    ev = _events(m)[-1]
    assert ev.last_event == "save"
    assert ev.state == "idle"
    assert ev.episodes_saved == 1
    assert ev.task_label == "kitchen run"
    # Collection sessions normally repeat the same task across many takes.
    m.set_episode("start")
    assert _events(m)[-1].task_label == "kitchen run"


def test_set_episode_discard(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    m.set_episode("start", "bad take")
    m.set_episode("discard")
    ev = _events(m)[-1]
    assert ev.last_event == "discard"
    assert ev.state == "idle"
    assert ev.episodes_discarded == 1


def test_set_episode_invalid_event_is_noop(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    msg = m.set_episode(cast("EpisodeCommand", "bogus"))
    assert "error" in msg.lower()
    assert _events(m) == []  # no transition emitted


@pytest.mark.parametrize("event", ["save", "discard"])
def test_set_episode_termination_while_idle_is_noop(
    make_monitor: Callable[..., EpisodeMonitorModule],
    event: EpisodeCommand,
) -> None:
    m = make_monitor()

    msg = m.set_episode(event)

    assert "no episode" in msg.lower()
    assert _events(m) == []


def test_concurrent_starts_keep_their_own_labels(
    make_monitor: Callable[..., EpisodeMonitorModule],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    m = make_monitor()
    original_transition = m._transition
    barrier = threading.Barrier(2)

    def synchronized_transition(
        event: EpisodeCommand,
        ts: float,
        task_label: str | None = None,
    ) -> EpisodeStatus | None:
        barrier.wait(timeout=2)
        return original_transition(event, ts, task_label)

    monkeypatch.setattr(m, "_transition", synchronized_transition)
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [
            executor.submit(m.set_episode, "start", "alpha"),
            executor.submit(m.set_episode, "start", "beta"),
        ]
        for future in futures:
            future.result(timeout=3)

    assert sorted(event.task_label or "" for event in _events(m)) == ["alpha", "beta"]


def test_concurrent_transitions_publish_in_state_order(
    make_monitor: Callable[..., EpisodeMonitorModule],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    m = make_monitor()
    original_emit = m._emit
    alpha_ready = threading.Event()
    release_alpha = threading.Event()

    def delayed_emit(status: EpisodeStatus) -> EpisodeStatus:
        if status.task_label == "alpha":
            alpha_ready.set()
            assert release_alpha.wait(timeout=2)
        return original_emit(status)

    monkeypatch.setattr(m, "_emit", delayed_emit)
    with ThreadPoolExecutor(max_workers=2) as executor:
        alpha = executor.submit(m.set_episode, EpisodeCommand.START, "alpha")
        assert alpha_ready.wait(timeout=2)
        beta = executor.submit(m.set_episode, EpisodeCommand.START, "beta")
        release_alpha.set()
        alpha.result(timeout=3)
        beta.result(timeout=3)

    assert [event.task_label for event in _events(m)] == ["alpha", "beta"]


def test_recording_skills_drive_episode_state(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()

    assert "demo" in m.start_recording("demo")
    assert "saved" in m.stop_recording().lower()
    assert [event.last_event for event in _events(m)] == ["start", "save"]


def test_start_recording_reuses_previous_task_label(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    m.start_recording("demo")
    m.stop_recording()

    assert "demo" in m.start_recording()
    assert _events(m)[-1].task_label == "demo"


def test_discard_recording_skill_discards_episode(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()

    m.start_recording("bad take")
    assert "discarded" in m.discard_recording().lower()
    assert _events(m)[-1].last_event == "discard"
