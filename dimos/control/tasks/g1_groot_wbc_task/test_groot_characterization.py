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

"""Characterization of the G1 GR00T WBC task across the execution refactor.

The numbers in ``test_data/groot_wbc_characterization.json`` were recorded from
the task on ``main`` before it was migrated onto the shared policy execution
helpers. The scenario is deterministic (stubbed actor, scripted coordinator
state), so the refactored task must reproduce it exactly: this is the evidence
that sharing execution mechanics did not change the trained controller's
observation, action, decimation or arming semantics.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import onnxruntime as ort
import pytest

from dimos.control.components import make_humanoid_joints
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    G1GrootWBCTask,
    G1GrootWBCTaskConfig,
)
from dimos.hardware.whole_body.spec import IMUState

_DATA = Path(__file__).parent / "test_data" / "groot_wbc_characterization.jsonl"


class _ScriptedSession:
    """Deterministic actor keyed off the observation it receives."""

    def __init__(self, path: str, providers: Any = None) -> None:
        self.label = "balance" if "balance" in str(path) else "walk"
        self._providers = list(providers or [])
        self._input = MagicMock()
        self._input.name = "obs"

    def get_inputs(self) -> list[Any]:
        return [self._input]

    def get_providers(self) -> list[str]:
        return self._providers

    def run(self, outputs: Any, feed: dict[str, np.ndarray]) -> list[np.ndarray]:
        del outputs
        obs = np.asarray(feed["obs"], dtype=np.float32).reshape(-1)
        phase = float(np.sum(obs * np.linspace(0.001, 0.002, obs.size, dtype=np.float32)))
        scale = 0.5 if self.label == "balance" else 1.0
        action = np.array(
            [scale * np.sin(phase + index * 0.37) for index in range(15)],
            dtype=np.float32,
        )
        return [action.reshape(1, 15)]


@pytest.fixture
def joints_29() -> list[str]:
    return make_humanoid_joints("g1")


@pytest.fixture
def patched_ort(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(ort, "InferenceSession", _ScriptedSession)
    monkeypatch.setattr(ort, "get_available_providers", lambda: ["CPUExecutionProvider"])


def _state_at(t_now: float, joint_names: list[str], missing: str | None = None) -> CoordinatorState:
    positions = {name: 0.01 * index + 0.002 * t_now for index, name in enumerate(joint_names)}
    velocities = {name: 0.02 * index - 0.001 * t_now for index, name in enumerate(joint_names)}
    if missing is not None:
        positions.pop(missing)
        velocities.pop(missing)
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=positions,
            joint_velocities=velocities,
            joint_efforts={name: 0.0 for name in joint_names},
            timestamp=t_now,
        ),
        imu={
            "g1": IMUState(
                quaternion=(0.99, 0.01, -0.02, 0.03),
                gyroscope=(0.01, -0.02, 0.03),
                accelerometer=(0.0, 0.0, -9.81),
                rpy=(0.0, 0.0, 0.0),
            )
        },
        t_now=t_now,
        dt=0.002,
    )


def run_scenario() -> list[dict[str, Any]]:
    """Run the fixed scenario and return every tick's observable result."""
    joints = make_humanoid_joints("g1")
    adapter = MagicMock()
    adapter.read_imu.return_value = IMUState(
        quaternion=(1.0, 0.0, 0.0, 0.0),
        gyroscope=(0.0, 0.0, 0.0),
        accelerometer=(0.0, 0.0, -9.81),
        rpy=(0.0, 0.0, 0.0),
    )
    task = G1GrootWBCTask(
        name="characterization",
        config=G1GrootWBCTaskConfig(
            balance_onnx="/fake/balance.onnx",
            walk_onnx="/fake/walk.onnx",
            joint_names=joints[:15],
            all_joint_names=joints,
            priority=50,
            auto_arm=False,
            default_ramp_seconds=1.0,
        ),
        adapter=adapter,
    )
    records: list[dict[str, Any]] = []

    def step(t_now: float, missing: str | None = None) -> None:
        output = task.compute(_state_at(t_now, joints, missing))
        snapshot = task.state_snapshot()
        records.append(
            {
                "t": t_now,
                "positions": None
                if output is None
                else [round(v, 7) for v in (output.positions or [])],
                "armed": snapshot["armed"],
                "arming": snapshot["arming"],
                "arm_pending": snapshot["arm_pending"],
                "active": snapshot["active"],
                "dry_run": snapshot["dry_run"],
            }
        )

    try:
        task.start()
        step(0.0)
        step(0.1, missing=joints[3])
        assert task.arm(ramp_seconds=1.0)
        step(0.2)
        step(0.7)
        step(1.2)
        task.set_velocity_command(0.4, -0.2, 0.1, t_now=1.2)
        for index in range(20):
            step(1.25 + 0.05 * index)
        for index in range(20):
            step(2.3 + 0.05 * index)
        task.set_dry_run(True)
        for index in range(12):
            step(3.4 + 0.05 * index)
        task.set_dry_run(False)
        for index in range(12):
            step(4.1 + 0.05 * index)
        assert task.reset_runtime_state(reactivate=True)
        step(5.0)
        step(5.05)
        task.stop()
        step(5.1)
    finally:
        task.stop()

    return records


@pytest.fixture
def scenario(patched_ort: None) -> Iterator[list[dict[str, Any]]]:
    yield run_scenario()


def test_refactor_preserves_recorded_groot_behavior(scenario: list[dict[str, Any]]) -> None:
    expected = [json.loads(line) for line in _DATA.read_text().splitlines() if line.strip()]
    assert len(scenario) == len(expected)
    for recorded, reference in zip(scenario, expected, strict=True):
        assert recorded == reference


def test_characterization_covers_inference_and_modes(
    scenario: list[dict[str, Any]],
) -> None:
    """Guard that the recorded scenario really exercised the interesting paths."""
    positions = [record["positions"] for record in scenario]
    assert sum(value is not None for value in positions) > 20
    assert sum(value is None for value in positions) > 10
    assert any(record["arming"] for record in scenario)
    assert any(record["dry_run"] for record in scenario)
    assert scenario[-1]["active"] is False
