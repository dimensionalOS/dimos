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

from dataclasses import replace

import pytest
from pytest_mock import MockerFixture

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.gripper_task.gripper_task import (
    GripperControlTask,
    GripperControlTaskConfig,
)
from dimos.control.tasks.teach_task.teach_task import (
    TeachControlTask,
    TeachControlTaskConfig,
    create_task,
)
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.hardware.whole_body.spec import WholeBodyAdapter, WholeBodyConfig
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS
from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE


def _state(positions: dict[str, float]) -> CoordinatorState:
    return CoordinatorState(joints=JointStateSnapshot(joint_positions=positions))


def _whole_body(
    mocker: MockerFixture,
    *,
    joints: list[str] | None = None,
    kp: tuple[float, ...] | None = None,
    kd: tuple[float, ...] | None = None,
) -> ConnectedWholeBody:
    names = list(joints or OPENYAM_JOINTS)
    component = HardwareComponent(
        hardware_id="robot",
        hardware_type=HardwareType.WHOLE_BODY,
        joints=names,
        wb_config=WholeBodyConfig(
            kp=(0.0,) * len(names) if kp is None else kp,
            kd=(1.0,) * len(names) if kd is None else kd,
        ),
    )
    return ConnectedWholeBody(mocker.Mock(spec=WholeBodyAdapter), component)


def _cfg(joints: list[str] | None = None) -> TaskConfig:
    return TaskConfig(
        name="teach",
        type="teach",
        joint_names=list(OPENYAM_JOINTS if joints is None else joints),
        priority=10,
    )


def test_teach_task_mirrors_a_complete_measured_state_in_order() -> None:
    task = TeachControlTask(
        "teach",
        TeachControlTaskConfig(tuple(OPENYAM_JOINTS), priority=10),
    )
    positions = {name: float(index) for index, name in enumerate(reversed(OPENYAM_JOINTS))}

    output = task.compute(_state(positions))

    assert task.is_active()
    assert task.claim().mode is ControlMode.SERVO_POSITION
    assert output is not None
    assert output.joint_names == OPENYAM_JOINTS
    assert output.positions == [positions[name] for name in OPENYAM_JOINTS]


@pytest.mark.parametrize("bad_position", [None, float("nan"), float("inf")])
def test_teach_task_rejects_incomplete_or_nonfinite_state(bad_position: float | None) -> None:
    task = TeachControlTask("teach", TeachControlTaskConfig(tuple(OPENYAM_JOINTS)))
    positions = {name: 0.0 for name in OPENYAM_JOINTS}
    if bad_position is None:
        del positions[OPENYAM_JOINTS[0]]
    else:
        positions[OPENYAM_JOINTS[0]] = bad_position

    assert task.compute(_state(positions)) is None


def test_gripper_preempts_only_the_seventh_teach_action() -> None:
    teach = TeachControlTask("teach", TeachControlTaskConfig(tuple(OPENYAM_JOINTS), priority=10))
    gripper = GripperControlTask(
        "arm_gripper",
        GripperControlTaskConfig([OPENYAM_JOINTS[-1]], priority=20),
        limits=[(0.0, 1.0)],
    )
    assert gripper.set_normalized([1.0])
    state = _state({name: index / 10 for index, name in enumerate(OPENYAM_JOINTS)})
    commands = [
        (teach, teach.claim(), teach.compute(state)),
        (gripper, gripper.claim(), gripper.compute(state)),
    ]

    winners, _ = TickLoop._arbitrate(object.__new__(TickLoop), commands)

    assert list(winners) == OPENYAM_JOINTS
    assert list(winners) == OPENYAM_LEARNING_PROFILE.dataprep_config().action["action"].names
    assert [value for value, _, _ in winners.values()][:-1] == pytest.approx(
        [index / 10 for index in range(6)]
    )
    assert winners[OPENYAM_JOINTS[-1]] == (1.0, ControlMode.SERVO_POSITION, "arm_gripper")


def test_factory_accepts_zero_stiffness_whole_body_hardware(
    mocker: MockerFixture,
) -> None:
    task = create_task(_cfg(), {"robot": _whole_body(mocker)})
    assert task.claim().joints == frozenset(OPENYAM_JOINTS)


def test_factory_rejects_non_whole_body_hardware(mocker: MockerFixture) -> None:
    component = HardwareComponent(
        hardware_id="robot",
        hardware_type=HardwareType.MANIPULATOR,
        joints=list(OPENYAM_JOINTS),
    )
    hardware = {
        "robot": ConnectedHardware(mocker.Mock(spec=ManipulatorAdapter), component),
    }

    with pytest.raises(ValueError, match="requires whole-body hardware"):
        create_task(_cfg(), hardware)


@pytest.mark.parametrize(
    ("cfg", "component_update", "match"),
    [
        (_cfg([]), {}, "requires at least one joint"),
        (_cfg([OPENYAM_JOINTS[0], OPENYAM_JOINTS[0]]), {}, "duplicates"),
        (_cfg(["missing"]), {}, "not owned"),
        (_cfg(), {"kp": (1.0,) * len(OPENYAM_JOINTS)}, "zero stiffness"),
        (
            _cfg(),
            {"kd": (float("nan"),) * len(OPENYAM_JOINTS)},
            "must be finite",
        ),
    ],
)
def test_factory_rejects_unsafe_configuration(
    mocker: MockerFixture,
    cfg: TaskConfig,
    component_update: dict[str, tuple[float, ...]],
    match: str,
) -> None:
    hardware = _whole_body(mocker)
    if component_update:
        assert hardware.component.wb_config is not None
        hardware.component.wb_config = replace(hardware.component.wb_config, **component_update)

    with pytest.raises(ValueError, match=match):
        create_task(cfg, {"robot": hardware})
