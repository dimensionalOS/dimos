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
from dimos.control.tasks.policy_execution import (
    OnnxActor,
    PolicyCommandHold,
    PolicySessionGuard,
    PolicyTask,
    PolicyTick,
    PolicyTickScheduler,
)


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


def test_onnx_actor_loads_first_input_and_flattens_output(stub_ort: list[_StubSession]) -> None:
    actor = OnnxActor("/models/actor.onnx", providers=["CPUExecutionProvider"])

    result = actor.run(np.zeros((1, 8), dtype=np.float32))

    assert actor.input_name == "obs"
    assert actor.output_name is None
    assert actor.providers == ["CPUExecutionProvider"]
    np.testing.assert_array_equal(result, np.full(4, 0.25, dtype=np.float32))
    assert stub_ort[0].path == "/models/actor.onnx"
    np.testing.assert_array_equal(stub_ort[0].feeds[0], np.zeros((1, 8), dtype=np.float32))


def test_onnx_actor_forwards_explicit_io_names(stub_ort: list[_StubSession]) -> None:
    actor = OnnxActor(
        "/models/actor.onnx",
        providers=["CPUExecutionProvider"],
        input_name="observation",
        output_name="action",
    )

    assert actor.input_name == "observation"
    assert actor.output_name == "action"


def test_onnx_actor_defaults_to_cpu_without_cuda_preference(stub_ort: list[_StubSession]) -> None:
    actor = OnnxActor("/models/actor.onnx")

    assert stub_ort[0].providers == ["CPUExecutionProvider"]
    assert actor.providers == ["CPUExecutionProvider"]


def test_decimation_scheduler_runs_every_nth_poll_from_reset() -> None:
    scheduler = PolicyTickScheduler.every(3)

    assert [scheduler.poll(0.0) for _ in range(6)] == [False, False, True, False, False, True]
    assert scheduler.tick_count == 6

    scheduler.reset()
    assert [scheduler.poll(0.0) for _ in range(3)] == [False, False, True]


def test_rate_scheduler_runs_immediately_and_advances_only_when_marked() -> None:
    scheduler = PolicyTickScheduler.at_hz(20.0)
    scheduler.reset(t_now=100.0)

    assert scheduler.poll(100.0)
    scheduler.mark(100.0)
    assert not scheduler.poll(100.02)
    assert scheduler.poll(100.05)
    scheduler.mark(100.05)
    assert not scheduler.poll(100.06)

    # A missed deadline retries until a result is actually used.
    assert scheduler.poll(100.20)
    assert scheduler.poll(100.21)
    scheduler.mark(100.21)
    assert not scheduler.poll(100.22)
    assert scheduler.poll(100.26)


def test_rate_scheduler_never_bursts_after_a_stall() -> None:
    scheduler = PolicyTickScheduler.at_hz(10.0)
    scheduler.reset(t_now=0.0)
    assert scheduler.poll(0.0)
    scheduler.mark(0.0)

    # The robot stalled for a second; the next inference is scheduled from now,
    # not replayed once per missed period.
    assert scheduler.poll(1.0)
    scheduler.mark(1.0)
    assert not scheduler.poll(1.05)
    assert scheduler.poll(1.1)


def test_rate_scheduler_matches_the_legacy_20hz_indices_at_100hz() -> None:
    """Differential: 100 Hz coordinator ticks must infer on ticks 0,5,...,100."""

    def legacy_indices(times: list[float], period_s: float) -> list[int]:
        # The Metal task's pre-refactor state machine, kept here as the oracle.
        next_tick: float | None = 0.0
        indices: list[int] = []
        for index, now in enumerate(times):
            if next_tick is not None and now + 1e-9 < next_tick:
                continue
            indices.append(index)
            scheduled = (next_tick if next_tick is not None else now) + period_s
            next_tick = now + period_s if scheduled <= now else scheduled
        return indices

    times = [index * 0.01 for index in range(101)]
    scheduler = PolicyTickScheduler.at_hz(20.0)
    scheduler.reset(times[0])
    actual: list[int] = []
    for index, now in enumerate(times):
        if scheduler.poll(now):
            actual.append(index)
            scheduler.mark(now)

    assert legacy_indices(times, 0.05) == list(range(0, 101, 5))
    assert actual == legacy_indices(times, 0.05)


@pytest.mark.parametrize("interval", [0.01, 0.02, 0.035])
def test_rate_scheduler_matches_the_legacy_oracle_for_late_ticks(interval: float) -> None:
    """Late and jittered ticks schedule from the tick that actually ran."""

    def legacy(times: list[float], period_s: float) -> list[float]:
        next_tick: float | None = 0.0
        fired: list[float] = []
        for now in times:
            if next_tick is not None and now + 1e-9 < next_tick:
                continue
            fired.append(now)
            scheduled = (next_tick if next_tick is not None else now) + period_s
            next_tick = now + period_s if scheduled <= now else scheduled
        return fired

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
    scheduler = PolicyTickScheduler.at_hz(20.0)
    scheduler.reset(times[0])
    actual = []
    for now in times:
        if scheduler.poll(now):
            actual.append(now)
            scheduler.mark(now)
    assert actual == legacy(times, 0.05)


def test_decimation_scheduler_is_unaffected_by_the_rate_tolerance() -> None:
    scheduler = PolicyTickScheduler.every(5)
    assert [scheduler.poll(index * 0.01) for index in range(11)] == [
        index % 5 == 4 for index in range(11)
    ]


def test_scheduler_rejects_ambiguous_or_invalid_cadence() -> None:
    with pytest.raises(ValueError):
        PolicyTickScheduler()
    with pytest.raises(ValueError):
        PolicyTickScheduler(every_n_ticks=2, hz=10.0)
    with pytest.raises(ValueError):
        PolicyTickScheduler.every(0)
    with pytest.raises(ValueError):
        PolicyTickScheduler.at_hz(0.0)


def test_command_hold_reemits_and_respects_dry_run() -> None:
    hold = PolicyCommandHold(["a/1", "a/2"])
    assert hold.hold() is None
    assert hold.output() is None

    hold.set([0.1, 0.2])
    held = hold.hold()
    assert held is not None
    assert held.joint_names == ["a/1", "a/2"]
    np.testing.assert_allclose(held.positions or [], [0.1, 0.2])

    hold.set_dry_run(True)
    assert hold.hold() is None
    assert hold.emit([0.3, 0.4]) is None
    np.testing.assert_allclose(hold.positions or [], [0.3, 0.4])

    hold.set_dry_run(False)
    emitted = hold.emit([0.5, 0.6])
    assert emitted is not None
    np.testing.assert_allclose(emitted.positions or [], [0.5, 0.6])

    hold.clear()
    assert hold.positions is None
    assert hold.hold() is None
    hold.set_dry_run(False)
    emitted = hold.emit([0.5, 0.6])
    assert emitted is not None
    np.testing.assert_allclose(emitted.positions or [], [0.5, 0.6])

    hold.clear()
    assert hold.positions is None
    assert hold.hold() is None


def test_session_guard_tracks_the_active_generation() -> None:
    guard = PolicySessionGuard()
    assert not guard.active
    assert not guard.current(0)

    first = guard.begin()
    assert guard.active
    assert guard.current(first)
    assert not guard.current(first - 1)

    guard.invalidate()
    assert not guard.active
    assert not guard.current(first)

    second = guard.begin()
    assert second == first + 2
    assert guard.current(second)
    assert not guard.current(first)


def test_session_guard_invalidating_an_inactive_session_keeps_the_epoch() -> None:
    guard = PolicySessionGuard()
    guard.invalidate()
    assert guard.epoch == 0

    epoch = guard.begin()
    guard.invalidate()
    assert guard.epoch == epoch + 1
    guard.invalidate()
    assert guard.epoch == epoch + 1


class _StubPolicyTask(PolicyTask):
    """Smallest real consumer of the shared task skeleton."""

    def __init__(self, *, dry_run: bool = False, hz: float = 20.0) -> None:
        super().__init__(
            "stub_policy",
            ["a/1", "a/2"],
            scheduler=PolicyTickScheduler.at_hz(hz),
            priority=7,
            dry_run=dry_run,
        )
        self.acts = 0
        self.resets = 0
        self.ended = 0
        self.command = [0.0, 0.0]
        self.fail_act = False
        self.on_act: Any = None
        self.on_reset: Any = None

    def prepare(self, state: CoordinatorState) -> Any:
        if not self.session_active:
            if not self._session_requested:
                return None
            self.begin_session_locked(state.t_now)
            return PolicyTick(self.session_epoch, state.t_now, reset=True)
        if not self._scheduler.poll(state.t_now):
            return self._hold.hold()
        return PolicyTick(self.session_epoch, state.t_now)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        del by_task, joints

    def reset_policy(self, tick: PolicyTick) -> None:
        self.resets += 1
        if self.on_reset is not None:
            self.on_reset()

    def act(self, tick: PolicyTick) -> np.ndarray:
        self.acts += 1
        if self.on_act is not None:
            self.on_act()
        if self.fail_act:
            raise RuntimeError("actor failed")
        return np.asarray(self.command, dtype=np.float32)

    def on_session_ended(self) -> None:
        self.ended += 1


def _state(t_now: float) -> CoordinatorState:
    return CoordinatorState(joints=MagicMock(), t_now=t_now, dt=0.01)


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


def test_policy_task_cancellation_drops_an_in_flight_result() -> None:
    task = _StubPolicyTask()
    task.start()
    task.request_session(0.0)
    entered, finish = threading.Event(), threading.Event()
    task.on_act = lambda: (entered.set(), finish.wait(5))
    outputs: list[Any] = []
    worker = threading.Thread(target=lambda: outputs.append(task.compute(_state(0.0))), daemon=True)
    worker.start()
    assert entered.wait(2)
    task.invalidate_session()
    finish.set()
    worker.join(2)
    assert outputs == [None]
    assert not task.session_active and task.ended == 1


def test_policy_task_cancellation_is_responsive_during_reset() -> None:
    task = _StubPolicyTask()
    task.start()
    task.request_session(0.0)
    entered, finish = threading.Event(), threading.Event()
    task.on_reset = lambda: (entered.set(), finish.wait(5))
    outputs: list[Any] = []
    worker = threading.Thread(target=lambda: outputs.append(task.compute(_state(0.0))), daemon=True)
    worker.start()
    assert entered.wait(2)
    done = threading.Event()

    def stop() -> None:
        task.invalidate_session()
        done.set()

    stopper = threading.Thread(target=stop, daemon=True)
    stopper.start()
    assert done.wait(2), "invalidation blocked on the runtime reset"
    finish.set()
    worker.join(2)
    stopper.join(2)
    assert outputs == [None] and task.resets == 1


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
