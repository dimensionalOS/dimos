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

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import (
    ControlIKResult,
    IKControlRuntimeError,
    PinkControlIKConfig,
)
from dimos.control.tasks.eef_twist_task.eef_twist_task import EEFTwistTask, EEFTwistTaskConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped


@dataclass
class FakePose:
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]

    def copy(self) -> FakePose:
        return FakePose(self.translation.copy(), self.rotation.copy())


class FakeIK:
    def __init__(self) -> None:
        self.nq = 3
        self.fk_calls: list[np.ndarray] = []
        self.solve_calls: list[FakePose] = []
        self.dt_calls: list[float] = []
        self.solution = np.array([0.01, 0.02, 0.03], dtype=np.float64)
        self.raise_runtime = False

    def forward_kinematics(self, q_current: NDArray[np.float64]) -> FakePose:
        self.fk_calls.append(q_current.copy())
        return FakePose(q_current.copy(), np.eye(3, dtype=np.float64))

    def solve(self, pose: FakePose, q_current: NDArray[np.float64], dt: float) -> ControlIKResult:
        if self.raise_runtime:
            raise IKControlRuntimeError("synthetic solver failure")
        self.solve_calls.append(pose.copy())
        self.dt_calls.append(dt)
        return ControlIKResult(self.solution.copy(), self.solution - q_current)


@pytest.fixture
def fake_ik(mocker) -> FakeIK:
    ik = FakeIK()
    mocker.patch(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.PinkControlIK",
        return_value=ik,
    )
    return ik


@pytest.fixture
def task(fake_ik: FakeIK) -> EEFTwistTask:
    return EEFTwistTask(
        "eef",
        EEFTwistTaskConfig(
            joint_names=["arm/joint1", "arm/joint2", "arm/joint3"],
            control_ik=PinkControlIKConfig(
                robot_model=RobotModelConfig(
                    name="fake",
                    model_path="fake.urdf",
                    base_pose=PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]),
                    joint_names=["arm/joint1", "arm/joint2", "arm/joint3"],
                    end_effector_link="tool",
                    home_joints=[0.0, 0.0, 0.0],
                )
            ),
            timeout=0.3,
            max_joint_delta_deg=15.0,
            min_dt=0.02,
            max_dt=0.03,
        ),
    )


def _state(
    t_now: float, positions: list[float] | None = None, dt: float = 0.01
) -> CoordinatorState:
    values = [0.0, 0.0, 0.0] if positions is None else positions
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={f"arm/joint{i + 1}": value for i, value in enumerate(values)},
        ),
        t_now=t_now,
        dt=dt,
    )


def _twist(x: float = 0.1) -> TwistStamped:
    return TwistStamped(frame_id="eef", linear=[x, 0.0, 0.0], angular=[0.0, 0.0, 0.0])


def test_first_nonzero_command_activates_seeds_from_fk_and_outputs_servo_position(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.is_active()

    output = task.compute(_state(1.01))

    assert output is not None
    assert output.mode == ControlMode.SERVO_POSITION
    assert output.joint_names == ["arm/joint1", "arm/joint2", "arm/joint3"]
    assert output.positions == [0.01, 0.02, 0.03]
    assert fake_ik.solve_calls[0].translation[0] > 0.0


def test_twist_task_rejects_cartesian_commands_without_activation(task: EEFTwistTask) -> None:
    assert not task.on_cartesian_command(object(), t_now=1.0)
    assert not task.is_active()


def test_ik_runtime_error_is_a_bounded_hold(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    fake_ik.raise_runtime = True
    hold = task.compute(_state(1.01))
    assert hold is not None
    assert hold.mode == ControlMode.SERVO_POSITION
    assert hold.positions == [0.0, 0.0, 0.0]


def test_integration_uses_current_fk_and_coordinator_dt(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    assert task.on_ee_twist_command(_twist(1.0), t_now=1.0)

    first = task.compute(_state(1.01, dt=0.01))
    fake_ik.solution = np.array([0.51, 0.0, 0.0], dtype=np.float64)
    second = task.compute(_state(1.04, positions=[0.5, 0.0, 0.0], dt=0.01))

    assert first is not None
    assert second is not None
    assert fake_ik.dt_calls == [0.02, 0.02]
    assert fake_ik.solve_calls[1].translation[0] > fake_ik.solve_calls[0].translation[0]


def test_control_ik_result_positions_are_used_when_shape_is_valid(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    output = task.compute(_state(1.01))

    assert output is not None
    assert output.positions == [0.01, 0.02, 0.03]


def test_non_finite_ik_solution_is_rejected(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    fake_ik.solution = np.array([np.nan, 0.0, 0.0], dtype=np.float64)

    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    output = task.compute(_state(1.01))

    assert output is not None
    assert output.mode == ControlMode.SERVO_POSITION
    assert output.positions == [0.0, 0.0, 0.0]


def test_non_finite_twist_is_rejected_without_activating_task(task: EEFTwistTask) -> None:
    accepted = task.on_ee_twist_command(
        TwistStamped(frame_id="eef", linear=[np.nan, 0.0, 0.0], angular=[0.0, 0.0, 0.0]),
        t_now=1.0,
    )

    assert accepted is False
    assert not task.is_active()


def test_missing_joint_state_skips_fk_and_ik(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)

    output = task.compute(_state(1.01, positions=[0.0, 0.0]))

    assert output is None
    assert fake_ik.fk_calls == []
    assert fake_ik.solve_calls == []


def test_joint_delta_rejection_returns_a_hold(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    fake_ik.solution = np.array([10.0, 0.0, 0.0], dtype=np.float64)

    rejected = task.compute(_state(1.01))

    assert rejected is not None
    assert rejected.mode == ControlMode.SERVO_POSITION
    assert rejected.positions == [0.0, 0.0, 0.0]


def test_timeout_and_zero_command_clear_then_next_nonzero_reseeds(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.compute(_state(1.01)) is not None

    assert task.compute(_state(1.5)) is None
    assert not task.is_active()

    fake_ik.solution = np.array([1.01, 0.0, 0.0], dtype=np.float64)
    assert task.on_ee_twist_command(_twist(), t_now=2.0)
    assert task.compute(_state(2.01, positions=[1.0, 0.0, 0.0])) is not None
    assert fake_ik.solve_calls[-1].translation[0] > 1.0

    assert task.on_ee_twist_command(_twist(0.0), t_now=2.02)
    assert not task.is_active()
