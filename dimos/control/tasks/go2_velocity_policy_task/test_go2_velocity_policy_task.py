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

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.go2_velocity_policy_task import go2_velocity_policy_task
from dimos.control.tasks.go2_velocity_policy_task.go2_velocity_policy_task import (
    GO2_DEFAULT_JOINT_POSITIONS,
    GO2_JOINT_SUFFIXES,
    Go2VelocityPolicyTask,
    Go2VelocityPolicyTaskConfig,
)
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class _StubSession:
    def __init__(self, *_args: Any, **_kwargs: Any) -> None:
        model_input = MagicMock(name="input")
        model_input.name = "observation"
        model_input.shape = [1, 45]
        model_output = MagicMock(name="output")
        model_output.name = "action"
        model_output.shape = [1, 12]
        self._inputs = [model_input]
        self._outputs = [model_output]
        self.last_feed: dict[str, np.ndarray] | None = None

    def get_inputs(self) -> list[Any]:
        return self._inputs

    def get_outputs(self) -> list[Any]:
        return self._outputs

    def get_providers(self) -> list[str]:
        return ["CPUExecutionProvider"]

    def run(self, outputs: list[str], feed: dict[str, np.ndarray]) -> list[np.ndarray]:
        assert outputs == ["action"]
        self.last_feed = feed
        return [np.full((1, 12), 0.2, dtype=np.float32)]


@pytest.fixture
def task(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Go2VelocityPolicyTask:
    model_path = tmp_path / "policy.onnx"
    model_path.write_bytes(b"model")
    monkeypatch.setattr(go2_velocity_policy_task.ort, "InferenceSession", _StubSession)
    value = Go2VelocityPolicyTask(
        "go2_policy",
        Go2VelocityPolicyTaskConfig(
            model_path=model_path,
            hardware_id="go2",
            joint_names=[f"go2/{name}" for name in GO2_JOINT_SUFFIXES],
        ),
    )
    value.start()
    return value


def _state(t_now: float = 1.0) -> CoordinatorState:
    names = [f"go2/{name}" for name in GO2_JOINT_SUFFIXES]
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict(zip(names, GO2_DEFAULT_JOINT_POSITIONS, strict=True)),
            joint_velocities=dict.fromkeys(names, 0.0),
            joint_efforts=dict.fromkeys(names, 0.0),
        ),
        imu={
            "go2": IMUState(
                quaternion=(1.0, 0.0, 0.0, 0.0),
                gyroscope=(0.1, 0.2, 0.3),
            )
        },
        t_now=t_now,
        dt=0.02,
    )


def test_policy_uses_coordinator_state_and_emits_all_joint_targets(
    task: Go2VelocityPolicyTask,
) -> None:
    task.on_twist_command(
        Twist(linear=Vector3(3.0, -2.0, 0.0), angular=Vector3(0.0, 0.0, 2.0)),
        1.0,
    )

    output = task.compute(_state())

    assert output is not None
    assert output.joint_names == [f"go2/{name}" for name in GO2_JOINT_SUFFIXES]
    np.testing.assert_allclose(
        output.positions,
        np.asarray(GO2_DEFAULT_JOINT_POSITIONS) + 0.1,
        atol=1e-6,
    )
    session = task._session
    assert isinstance(session, _StubSession)
    assert session.last_feed is not None
    policy_input = session.last_feed["observation"][0]
    np.testing.assert_allclose(policy_input[0:3], (0.1, 0.2, 0.3))
    np.testing.assert_allclose(policy_input[3:6], (0.0, 0.0, -1.0))
    np.testing.assert_allclose(policy_input[6:9], (2.0, -1.0, 1.0))


def test_policy_uses_zero_command_after_timeout(task: Go2VelocityPolicyTask) -> None:
    task.on_twist_command(
        Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3()),
        1.0,
    )

    assert task.compute(_state(t_now=2.0)) is not None

    session = task._session
    assert isinstance(session, _StubSession)
    assert session.last_feed is not None
    np.testing.assert_array_equal(session.last_feed["observation"][0, 6:9], np.zeros(3))


def test_policy_waits_for_complete_state(task: Go2VelocityPolicyTask) -> None:
    state = _state()
    state.joints.joint_positions.pop(next(iter(state.joints.joint_positions)))

    assert task.compute(state) is None
