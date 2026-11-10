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

from typing import Any
from unittest.mock import MagicMock

import numpy as np
import onnxruntime as ort
import pytest

from dimos.control.tasks.policy_execution import (
    OnnxActor,
    PolicyCommandHold,
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
