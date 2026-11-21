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

"""Tests for the execution pieces shared by ONNX policy tasks."""

from __future__ import annotations

import threading
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import onnxruntime as ort
import pytest

from dimos.control.task import CoordinatorState
from dimos.control.tasks.policy_execution import OnnxActor, PolicyTask, PolicyTick
from dimos.utils.testing.waiting import wait_until


class _StubSession:
    def __init__(self, path: str, providers: Any = None) -> None:
        self.path = path
        self.providers = list(providers or [])
        self._input = MagicMock()
        self._input.name = "obs"
        self.feeds: list[np.ndarray] = []

    def get_inputs(self) -> list[Any]:
        return [self._input]

    def get_providers(self) -> list[str]:
        return self.providers

    def run(self, outputs: Any, feed: dict[str, np.ndarray]) -> list[np.ndarray]:
        del outputs
        self.feeds.append(feed["obs"])
        return [np.full((1, 4), 0.25, dtype=np.float32)]


@pytest.fixture
def stub_ort(monkeypatch: pytest.MonkeyPatch) -> list[_StubSession]:
    sessions: list[_StubSession] = []

    def factory(path: str, providers: Any = None) -> _StubSession:
        session = _StubSession(path, providers)
        sessions.append(session)
        return session

    monkeypatch.setattr(ort, "InferenceSession", factory)
    monkeypatch.setattr(ort, "get_available_providers", lambda: ["CPUExecutionProvider"])
    return sessions


def test_onnx_actor_loads_names_providers_and_flattens_output(
    stub_ort: list[_StubSession],
) -> None:
    actor = OnnxActor("/models/actor.onnx")

    assert stub_ort[0].providers == ["CPUExecutionProvider"]
    np.testing.assert_array_equal(
        actor.run(np.zeros((1, 8), dtype=np.float32)), np.full(4, 0.25, dtype=np.float32)
    )
    assert (actor.input_name, actor.output_name) == ("obs", None)
    assert stub_ort[0].path == "/models/actor.onnx"
    np.testing.assert_array_equal(stub_ort[0].feeds[0], np.zeros((1, 8), dtype=np.float32))

    explicit = OnnxActor(
        "/models/actor.onnx",
        providers=["CPUExecutionProvider"],
        input_name="observation",
        output_name="action",
    )
    assert (explicit.input_name, explicit.output_name) == ("observation", "action")
    assert explicit.providers == ["CPUExecutionProvider"]


class _CadenceTask(PolicyTask):
    """Bare task used to exercise the cadence helpers without an actor."""

    def __init__(self, *, every_n_ticks: int | None = None, hz: float | None = None) -> None:
        super().__init__("cadence", ["a/1"], priority=1, every_n_ticks=every_n_ticks, hz=hz)


def test_decimation_scheduler_counts_polls_from_reset() -> None:
    task = _CadenceTask(every_n_ticks=3)
    assert [task._cadence_due(0.0) for _ in range(6)] == [False, False, True] * 2

    task._reset_cadence()
    assert [task._cadence_due(0.0) for _ in range(3)] == [False, False, True]

    # The rate-deadline tolerance is irrelevant to decimation.
    decimated = _CadenceTask(every_n_ticks=5)
    assert [decimated._cadence_due(index * 0.01) for index in range(11)] == [
        index % 5 == 4 for index in range(11)
    ]


def test_rate_scheduler_runs_when_due_and_never_bursts() -> None:
    task = _CadenceTask(hz=20.0)
    task._reset_cadence(100.0)

    assert task._cadence_due(100.0)
    task._mark_cadence(100.0)
    assert not task._cadence_due(100.02)
    assert task._cadence_due(100.05)
    task._mark_cadence(100.05)
    assert not task._cadence_due(100.06)

    # A missed deadline retries until a result is actually used.
    assert task._cadence_due(100.20)
    assert task._cadence_due(100.21)
    task._mark_cadence(100.21)
    assert not task._cadence_due(100.22)
    assert task._cadence_due(100.26)

    # A one-second stall schedules from now, never once per missed period.
    stalled = _CadenceTask(hz=10.0)
    stalled._reset_cadence(0.0)
    assert stalled._cadence_due(0.0)
    stalled._mark_cadence(0.0)
    assert stalled._cadence_due(1.0)
    stalled._mark_cadence(1.0)
    assert not stalled._cadence_due(1.05)
    assert stalled._cadence_due(1.1)


def _legacy_fires(times: list[float], period_s: float) -> list[float]:
    """The pre-refactor Metal deadline state machine, kept here as the oracle."""
    next_tick: float | None = 0.0
    fired: list[float] = []
    for now in times:
        if next_tick is not None and now + 1e-9 < next_tick:
            continue
        fired.append(now)
        scheduled = (next_tick if next_tick is not None else now) + period_s
        next_tick = now + period_s if scheduled <= now else scheduled
    return fired


def test_rate_scheduler_matches_the_legacy_20hz_indices_at_100hz() -> None:
    """Differential: 100 Hz coordinator ticks must infer on ticks 0,5,...,100."""
    times = [index * 0.01 for index in range(101)]
    task = _CadenceTask(hz=20.0)
    task._reset_cadence(times[0])
    actual: list[float] = []
    for now in times:
        if task._cadence_due(now):
            actual.append(now)
            task._mark_cadence(now)

    expected = [times[index] for index in range(0, 101, 5)]
    assert _legacy_fires(times, 0.05) == expected
    assert actual == expected


@pytest.mark.parametrize("interval", [0.01, 0.02, 0.035])
def test_rate_scheduler_matches_the_legacy_oracle_for_late_ticks(interval: float) -> None:
    """Late and jittered ticks schedule from the tick that actually ran."""
    # A stall (1.0 s), a tick a hair early (0.1500000005), and normal ticks.
    times = [
        0.0,
        0.04,
        0.1500000005,
        0.150000002,
        0.31,
        0.36,
        0.41,
        1.0,
        *[1.0 + index * interval for index in range(1, 41)],
    ]
    task = _CadenceTask(hz=20.0)
    task._reset_cadence(times[0])
    actual: list[float] = []
    for now in times:
        if task._cadence_due(now):
            actual.append(now)
            task._mark_cadence(now)
    assert actual == _legacy_fires(times, 0.05)


def test_scheduler_rejects_ambiguous_or_invalid_cadence() -> None:
    with pytest.raises(ValueError):
        _CadenceTask()
    with pytest.raises(ValueError):
        _CadenceTask(every_n_ticks=2, hz=10.0)
    with pytest.raises(ValueError):
        _CadenceTask(every_n_ticks=0)
    with pytest.raises(ValueError):
        _CadenceTask(hz=0.0)


class _StubPolicyTask(PolicyTask):
    """Smallest real consumer of the shared task skeleton."""

    def __init__(self, *, dry_run: bool = False, hz: float = 20.0) -> None:
        super().__init__(
            "stub_policy",
            ["a/1", "a/2"],
            priority=7,
            hz=hz,
            dry_run=dry_run,
        )
        self.acts = 0
        self.resets = 0
        self.ended = 0
        self.act_ticks: list[PolicyTick] = []
        self.reset_ticks: list[PolicyTick] = []
        self.command = [0.0, 0.0]
        self.fail_act = False
        self.on_act: Any = None
        self.on_reset: Any = None

    def prepare(self, state: CoordinatorState) -> Any:
        if not self.session_active:
            if not self._session_requested:
                return None
            self.begin_session(state.t_now)
            return PolicyTick(self.session_epoch, state.t_now, reset=True)
        if not self._cadence_due(state.t_now):
            return self._held_command()
        return PolicyTick(self.session_epoch, state.t_now)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        del by_task, joints

    def reset_policy(self, tick: PolicyTick) -> None:
        self.resets += 1
        self.reset_ticks.append(tick)
        if self.on_reset is not None:
            self.on_reset()

    def act(self, tick: PolicyTick) -> np.ndarray:
        self.acts += 1
        self.act_ticks.append(tick)
        if self.on_act is not None:
            self.on_act()
        if self.fail_act:
            raise RuntimeError("actor failed")
        return np.asarray(self.command, dtype=np.float32)

    def on_session_ended(self) -> None:
        self.ended += 1


def _state(t_now: float) -> CoordinatorState:
    return CoordinatorState(joints=MagicMock(), t_now=t_now, dt=0.01)


def test_session_epoch_advances_across_sessions_and_stops_on_inactive_close() -> None:
    task = _StubPolicyTask()
    task.start()
    task.request_session(0.0)
    assert task.compute(_state(0.0)) is not None
    first = task.session_epoch

    task.end_session()
    task.end_session()  # closing an inactive session keeps the generation
    assert task.session_epoch == first + 1

    task.request_session(1.0)
    assert task.compute(_state(1.0)) is not None
    assert task.session_epoch == first + 2


def test_policy_task_owns_claim_cadence_hold_and_dry_run() -> None:
    task = _StubPolicyTask()
    assert not task.is_active()
    assert task.claim().joints == frozenset({"a/1", "a/2"})
    task.start()
    assert task.is_active() and not task.session_active
    assert task.compute(_state(0.0)) is None

    task.request_session(0.0)
    assert task.is_active()
    first = task.compute(_state(0.0))
    assert first is not None and task.session_active and task.policy_ticks == 1
    # Between 20 Hz deadlines the shared hold re-emits the last command.
    held = task.compute(_state(0.02))
    assert held is not None and held.positions == first.positions
    assert task.acts == 1
    # The deadline tolerance keeps 0.05 due even from accumulated floats.
    assert task.compute(_state(0.05)) is not None
    assert task.acts == 2

    task.set_dry_run(True)
    assert task.compute(_state(0.10)) is None
    assert task.acts == 3 and task.policy_ticks == 3


def _block_phase(task: _StubPolicyTask, phase: str) -> tuple[threading.Event, threading.Event]:
    """Block the next act() or reset_policy() call so its ordering is observable."""
    entered, release = threading.Event(), threading.Event()

    def block() -> None:
        entered.set()
        release.wait(5)

    if phase == "act":
        task.on_act = block
    else:
        task.on_reset = block
    return entered, release


def _compute_in_thread(task: _StubPolicyTask, t_now: float) -> tuple[threading.Thread, list[Any]]:
    outputs: list[Any] = []
    worker = threading.Thread(
        target=lambda: outputs.append(task.compute(_state(t_now))), daemon=True
    )
    worker.start()
    return worker, outputs


@pytest.mark.parametrize("phase", ["act", "reset"])
@pytest.mark.parametrize(
    "ordering",
    ["new-session-while-blocked", "finish-before-new-session"],
    ids=["overlap", "finish-first"],
)
def test_new_session_ordering_around_a_cancelled_call(phase: str, ordering: str) -> None:
    """A new session opens while the old call is blocked, or only after it finishes."""
    task = _StubPolicyTask()
    task.command = [1.5, -2.0]
    task.start()
    task.request_session(0.0)
    entered, release = _block_phase(task, phase)

    old_worker, old_outputs = _compute_in_thread(task, 0.0)
    assert entered.wait(2)
    first_epoch = task.session_epoch
    assert task.session_active

    # Invalidate the old session and request a new one while its call is blocked.
    task.end_session()
    task.request_session(0.5)

    if ordering == "finish-before-new-session":
        release.set()
        old_worker.join(2)
        assert old_outputs == [None]
        new_worker, new_outputs = _compute_in_thread(task, 0.5)
    else:
        new_worker, new_outputs = _compute_in_thread(task, 0.5)
        wait_until(
            lambda: task.session_active and task.session_epoch == first_epoch + 2,
            timeout=2,
            interval=0.005,
            message="the new session did not open while the old call was blocked",
        )
        assert old_worker.is_alive() and new_worker.is_alive()
        assert old_outputs == [] and new_outputs == []
        release.set()
        old_worker.join(2)

    new_worker.join(2)
    assert not old_worker.is_alive() and not new_worker.is_alive() and old_outputs == [None]
    assert len(new_outputs) == 1 and new_outputs[0] is not None
    assert task.session_active and task.session_epoch == first_epoch + 2
    assert (task.ended, task.resets, task.acts) == (1, 2, 2 if phase == "act" else 1)
    # The new session's reset and act carry its own generation and tick time.
    assert (task.reset_ticks[-1].epoch, task.reset_ticks[-1].t_now) == (
        task.session_epoch,
        0.5,
    )
    assert (task.act_ticks[-1].epoch, task.act_ticks[-1].t_now) == (task.session_epoch, 0.5)
    assert new_outputs[0].positions == task.command


def test_policy_task_inference_error_and_bad_command_close_the_session() -> None:
    task = _StubPolicyTask()
    task.start()
    task.request_session(0.0)
    task.fail_act = True
    assert task.compute(_state(0.0)) is None
    assert not task.session_active
    task.fail_act = False

    task.request_session(0.1)
    task.command = [0.0, float("nan")]
    assert task.compute(_state(0.1)) is None
    assert not task.session_active
