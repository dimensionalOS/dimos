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
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import pinocchio
import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import TaskConfig
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import (
    ControlIKResult,
    IKControlRuntimeError,
    PinkControlIKConfig,
)
from dimos.control.tasks.teleop_task.teleop_task import (
    TeleopIKTask,
    TeleopIKTaskConfig,
    create_task,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.teleop.quest.quest_types import Buttons


@dataclass
class _FakePinkIK:
    nq: int = 2

    def __post_init__(self) -> None:
        self.fk_calls: list[NDArray[np.float64]] = []
        self.solve_calls: list[tuple[pinocchio.SE3, NDArray[np.float64], float]] = []
        self.solution = np.array([0.01, 0.02], dtype=np.float64)
        self.raise_runtime = False

    def forward_kinematics(self, q: NDArray[np.float64]) -> pinocchio.SE3:
        self.fk_calls.append(q.copy())
        return pinocchio.SE3(
            pinocchio.exp3(np.array([0.0, 0.0, 0.2], dtype=np.float64)),
            np.array([q[0], q[1], 0.3], dtype=np.float64),
        )

    def solve(
        self,
        target: pinocchio.SE3,
        measured: NDArray[np.float64],
        dt: float,
    ) -> ControlIKResult:
        if self.raise_runtime:
            raise IKControlRuntimeError("synthetic Pink failure")
        self.solve_calls.append((target.copy(), measured.copy(), dt))
        return ControlIKResult(self.solution.copy(), self.solution - measured)


def _robot(path: Path) -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=path,
        base_pose=PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]),
        joint_names=["joint1", "joint2"],
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2"),
                base_link="base",
                tip_link="tool",
            )
        ],
        joint_name_mapping={"arm/joint1": "joint1", "arm/joint2": "joint2"},
        home_joints=[0.0, 0.0],
    )


def _pink_config(path: Path) -> PinkControlIKConfig:
    return PinkControlIKConfig.model_validate({"robot_model": _robot(path)})


def _state(
    t_now: float,
    positions: tuple[float, ...] = (0.0, 0.0),
    *,
    dt: float = 0.01,
) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={
                f"arm/joint{index + 1}": position for index, position in enumerate(positions)
            }
        ),
        t_now=t_now,
        dt=dt,
    )


def _delta(
    position: tuple[float, float, float] = (0.1, -0.2, 0.4),
    angle: float = 0.3,
) -> PoseStamped:
    quaternion = pinocchio.Quaternion(pinocchio.exp3(np.array([0.0, 0.0, angle])))
    return PoseStamped(
        position=list(position),
        orientation=[quaternion.x, quaternion.y, quaternion.z, quaternion.w],
    )


@pytest.fixture
def fake_ik(mocker: MockerFixture) -> _FakePinkIK:
    backend = _FakePinkIK()
    mocker.patch(
        "dimos.control.tasks.cartesian_ik_task.cartesian_ik_task.PinkControlIK",
        return_value=backend,
    )
    return backend


@pytest.fixture
def task(tmp_path: Path, fake_ik: _FakePinkIK) -> TeleopIKTask:
    return TeleopIKTask(
        "teleop_arm",
        TeleopIKTaskConfig(
            joint_names=["arm/joint1", "arm/joint2"],
            control_ik=_pink_config(tmp_path / "unused.urdf"),
            hand="right",
            min_dt=0.02,
            max_dt=0.03,
            max_joint_delta_deg=5.0,
        ),
    )


@pytest.fixture
def gripper_task(tmp_path: Path, fake_ik: _FakePinkIK) -> TeleopIKTask:
    return TeleopIKTask(
        "teleop_arm",
        TeleopIKTaskConfig(
            joint_names=["arm/joint1", "arm/joint2"],
            control_ik=_pink_config(tmp_path / "unused.urdf"),
            hand="right",
            gripper_joint="arm/gripper",
            gripper_open_pos=0.8,
            gripper_closed_pos=0.0,
        ),
    )


def test_delta_is_composed_with_one_measured_engagement_baseline(
    task: TeleopIKTask, fake_ik: _FakePinkIK
) -> None:
    assert task.on_cartesian_command(_delta(), t_now=1.0)

    first = task.compute(_state(1.01, (1.0, 2.0), dt=1.0))
    second = task.compute(_state(1.02, (1.5, 2.5), dt=0.001))

    assert first is not None
    assert second is not None
    assert len(fake_ik.fk_calls) == 1
    first_target, first_measured, first_dt = fake_ik.solve_calls[0]
    second_target, second_measured, second_dt = fake_ik.solve_calls[1]
    baseline_rotation = pinocchio.exp3(np.array([0.0, 0.0, 0.2]))
    delta_rotation = pinocchio.exp3(np.array([0.0, 0.0, 0.3]))
    assert np.allclose(first_target.translation, [1.1, 1.8, 0.7])
    assert np.allclose(first_target.rotation, delta_rotation @ baseline_rotation)
    assert np.allclose(second_target.translation, first_target.translation)
    assert np.allclose(first_measured, [1.0, 2.0])
    assert np.allclose(second_measured, [1.5, 2.5])
    assert (first_dt, second_dt) == (0.03, 0.02)


def test_missing_joint_state_defers_baseline_and_output(
    task: TeleopIKTask, fake_ik: _FakePinkIK
) -> None:
    assert task.on_cartesian_command(_delta(), t_now=1.0)

    output = task.compute(_state(1.01, (0.0,)))

    assert output is None
    assert fake_ik.fk_calls == []
    assert fake_ik.solve_calls == []


def test_solver_failure_and_excessive_delta_return_measured_hold(
    task: TeleopIKTask, fake_ik: _FakePinkIK
) -> None:
    assert task.on_cartesian_command(_delta(), t_now=1.0)
    fake_ik.raise_runtime = True
    failed = task.compute(_state(1.01, (0.4, 0.5)))

    fake_ik.raise_runtime = False
    fake_ik.solution = np.array([2.0, 0.5], dtype=np.float64)
    rejected = task.compute(_state(1.02, (0.4, 0.5)))

    assert failed is not None
    assert failed.positions == [0.4, 0.5]
    assert rejected is not None
    assert rejected.positions == [0.4, 0.5]
    assert rejected.mode == ControlMode.SERVO_POSITION


def test_release_timeout_stop_and_clear_force_fresh_baselines(
    task: TeleopIKTask, fake_ik: _FakePinkIK
) -> None:
    pressed = Buttons()
    pressed.right_primary = True
    released = Buttons()

    assert task.on_teleop_buttons(pressed, 0.0)
    assert task.on_cartesian_command(_delta(), 1.0)
    assert task.compute(_state(1.01)) is not None
    assert task.on_teleop_buttons(released, 1.02)
    assert not task.is_active()

    assert task.on_teleop_buttons(pressed, 2.0)
    assert task.on_cartesian_command(_delta(), 2.0)
    assert task.compute(_state(2.01, (0.1, 0.2))) is not None
    assert task.compute(_state(3.0, (0.1, 0.2))) is None

    assert task.on_cartesian_command(_delta(), 4.0)
    assert task.compute(_state(4.01, (0.2, 0.3))) is not None
    task.stop()
    task.start()
    assert task.on_cartesian_command(_delta(), 5.0)
    assert task.compute(_state(5.01, (0.3, 0.4))) is not None
    task.clear()
    assert len(fake_ik.fk_calls) == 4


def test_estop_rejects_commands_and_never_replays_them(
    gripper_task: TeleopIKTask, fake_ik: _FakePinkIK
) -> None:
    assert gripper_task.on_cartesian_command(_delta(), 1.0)
    assert gripper_task.compute(_state(1.01)) is not None

    gripper_task.set_estop(True)
    assert not gripper_task.on_cartesian_command(_delta((9.0, 0.0, 0.0)), 2.0)
    assert not gripper_task.on_gripper_trigger(1.0)
    assert not gripper_task.is_active()

    gripper_task.set_estop(False)
    assert gripper_task.compute(_state(2.01)) is None
    assert gripper_task.on_cartesian_command(_delta(), 3.0)
    assert gripper_task.compute(_state(3.01, (0.2, 0.3))) is not None
    assert len(fake_ik.fk_calls) == 2


def test_gripper_claim_interpolation_and_hold_output(
    gripper_task: TeleopIKTask, fake_ik: _FakePinkIK
) -> None:
    assert gripper_task.on_gripper_trigger(0.25)
    assert gripper_task.on_cartesian_command(_delta(), 1.0)
    fake_ik.raise_runtime = True

    output = gripper_task.compute(_state(1.01, (0.4, 0.5)))

    assert gripper_task.claim().joints == frozenset({"arm/joint1", "arm/joint2", "arm/gripper"})
    assert output is not None
    assert output.joint_names == ["arm/joint1", "arm/joint2", "arm/gripper"]
    assert output.positions == pytest.approx([0.4, 0.5, 0.6])


def test_factory_requires_pink_configuration_and_matching_model(tmp_path: Path) -> None:
    legacy = TaskConfig(
        name="teleop",
        type="teleop_ik",
        joint_names=["arm/joint1", "arm/joint2"],
        params={"model_path": "legacy.xml", "ee_joint_id": 2, "hand": "right"},
    )
    with pytest.raises(ValueError, match="control_ik"):
        create_task(legacy, {})

    mismatched = TaskConfig(
        name="teleop",
        type="teleop_ik",
        joint_names=["wrong/joint1", "wrong/joint2"],
        params={
            "control_ik": {"robot_model": _robot(tmp_path / "unused.urdf")},
            "hand": "right",
        },
    )
    with pytest.raises(ValueError, match="task joints must match"):
        create_task(mismatched, {})
