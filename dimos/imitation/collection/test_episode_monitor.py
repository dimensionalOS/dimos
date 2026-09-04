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
import threading

from pydantic import ValidationError
import pytest
import pytest_mock

from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
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
        config.setdefault("task", "pick up the block")
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
    """Deliver one debounced Quest button-press edge."""
    attr = BUTTON_ALIASES[alias]
    pressed = Buttons()
    pressed.set_attribute(attr, True)
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
    assert events[-1].task_label == "pick up the block"


def test_task_is_required(make_monitor: Callable[..., EpisodeMonitorModule]) -> None:
    with pytest.raises(ValidationError, match="task"):
        EpisodeMonitorModule()


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
    m = make_monitor(button_map={"start": "A"})
    _press(m, "A")
    _press(m, "A")

    last = _events(m)[-1]
    assert last.last_event == "start"
    assert last.state == "recording"
    assert last.episodes_saved == 1  # the auto-committed one


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


def test_shutdown_discards_recording(make_monitor: Callable[..., EpisodeMonitorModule]) -> None:
    m = make_monitor()
    _press(m, "B")

    m.stop()

    last = _events(m)[-1]
    assert last.last_event == "discard"
    assert last.state == "idle"
    assert last.episodes_discarded == 1


def test_invalid_button_mapping_fails_at_startup(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    with pytest.raises(ValidationError, match="unknown Quest button mappings"):
        make_monitor(button_map={"toggle": "not_a_button"})


def test_duplicate_button_mapping_fails_at_startup(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    with pytest.raises(ValidationError, match="distinct Quest button"):
        make_monitor(button_map={"toggle": "B", "discard": "right_secondary"})


def test_buttons_are_ignored_after_shutdown_begins(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    m = make_monitor()
    m.stop()

    _press(m, "B")

    assert _events(m) == []


def test_stop_waits_for_in_flight_transition_and_blocks_later_transitions(
    make_monitor: Callable[..., EpisodeMonitorModule],
) -> None:
    class TrackingLock:
        def __init__(self) -> None:
            self._lock = threading.Lock()
            self.shutdown_attempted = threading.Event()

        def __enter__(self) -> None:
            if threading.current_thread().name == "episode-monitor-shutdown":
                self.shutdown_attempted.set()
            self._lock.acquire()

        def __exit__(self, *_: object) -> None:
            self._lock.release()

    m = make_monitor()
    transition_lock = TrackingLock()
    m._transition_lock = transition_lock  # type: ignore[assignment]
    m._transition("start", 1.0)
    emit_entered = threading.Event()
    release_emit = threading.Event()
    stop_done = threading.Event()
    original_emit = m._emit

    def blocking_emit(status: EpisodeStatus) -> EpisodeStatus:
        emit_entered.set()
        assert release_emit.wait(timeout=5.0)
        return original_emit(status)

    def stop_monitor() -> None:
        m.stop()
        stop_done.set()

    m._emit = blocking_emit  # type: ignore[method-assign]
    transition = threading.Thread(target=m._transition, args=("save", 2.0))
    transition.start()
    assert emit_entered.wait(timeout=5.0)

    shutdown = threading.Thread(target=stop_monitor, name="episode-monitor-shutdown")
    shutdown.start()
    try:
        assert transition_lock.shutdown_attempted.wait(timeout=5.0)
        assert not stop_done.is_set()
    finally:
        release_emit.set()
    transition.join(timeout=5.0)
    shutdown.join(timeout=5.0)

    assert stop_done.is_set()
    assert not transition.is_alive()
    assert not shutdown.is_alive()
    event_count = len(_events(m))
    m._transition("start", 3.0)
    assert len(_events(m)) == event_count
