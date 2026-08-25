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

import pytest
from pytest_mock import MockerFixture

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.gripper_task.gripper_task import (
    GripperControlTask,
    GripperControlTaskConfig,
    create_task,
)
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.hardware.spec import JointLimits
from dimos.msgs.std_msgs.Float32 import Float32


def _task() -> GripperControlTask:
    return GripperControlTask(
        "tool",
        GripperControlTaskConfig(joint_names=["arm/tool_joint"]),
        limits=[(0.0, 850.0)],
    )


def _state(**positions: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions=positions),
        t_now=0.0,
    )


def test_normalized_and_native_commands_share_the_joint_path() -> None:
    task = _task()
    assert task.set_normalized([0.5])
    output = task.compute(_state())
    assert output is not None
    assert output.positions == [425.0]
    assert output.mode is ControlMode.SERVO_POSITION

    assert task.set_position([612.0])
    output = task.compute(_state())
    assert output is not None
    assert output.positions == [612.0]


@pytest.mark.parametrize("bad", [[9999.0], [float("nan")], []])
def test_invalid_command_does_not_replace_the_latched_target(bad: list[float]) -> None:
    task = _task()
    assert task.set_position([100.0])
    assert task.set_position(bad) is False
    output = task.compute(_state())
    assert output is not None
    assert output.positions == [100.0]


def test_measured_normalized_value_is_not_clamped() -> None:
    task = _task()
    task.compute(_state(**{"arm/tool_joint": 1275.0}))
    assert task.get_normalized() == [1.5]


@pytest.mark.parametrize(("opening", "expected"), [(0.0, 0.0), (0.5, 425.0), (1.0, 850.0)])
def test_stream_input_routes_through_normalized_command(opening: float, expected: float) -> None:
    task = _task()
    assert task.on_gripper_command(Float32(data=opening), 0.0)
    output = task.compute(_state())
    assert output is not None
    assert output.positions == [expected]


@pytest.mark.parametrize("opening", [-0.1, 1.1, float("nan")])
def test_stream_input_rejects_invalid_normalized_opening(opening: float) -> None:
    task = _task()

    assert task.on_gripper_command(Float32(data=opening), 0.0) is False
    assert task.compute(_state()) is None


def _hardware(mocker: MockerFixture, limit_len: int = 7) -> dict[str, ConnectedHardware]:
    component = HardwareComponent(
        hardware_id="robot",
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*make_joints("arm", 6), "arm/tool_joint"],
    )
    adapter = mocker.Mock(spec=ManipulatorAdapter)
    adapter.get_limits.return_value = JointLimits(
        position_lower=[0.0] * limit_len,
        position_upper=[*([3.14] * 6), 0.08][:limit_len],
        velocity_max=[1.0] * limit_len,
    )
    return {"robot": ConnectedHardware(adapter, component)}


def _cfg(joints: list[str]) -> TaskConfig:
    return TaskConfig(name="tool", type="gripper", joint_names=joints, priority=20)


def test_limits_resolve_by_joint_name_in_adapter_order(mocker: MockerFixture) -> None:
    task = create_task(_cfg(["arm/joint2", "arm/tool_joint"]), _hardware(mocker))
    assert task.set_normalized([0.5, 0.5])
    output = task.compute(_state())
    assert output is not None
    assert output.positions == pytest.approx([1.57, 0.04])


def test_limit_resolution_rejects_ambiguous_joint_ownership(mocker: MockerFixture) -> None:
    hardware = _hardware(mocker)
    hardware["duplicate"] = hardware["robot"]

    with pytest.raises(ValueError, match="owned by multiple hardware components"):
        create_task(_cfg(["arm/tool_joint"]), hardware)


def test_limit_resolution_requires_selected_position_limits(mocker: MockerFixture) -> None:
    hardware = _hardware(mocker)
    hardware["robot"].adapter.get_limits.return_value = JointLimits(
        position_lower=[0.0] * 7,
        position_upper=[*([3.14] * 6), None],
        velocity_max=[1.0] * 7,
    )

    with pytest.raises(ValueError, match="has no declared position limits"):
        create_task(_cfg(["arm/tool_joint"]), hardware)


def test_limit_resolution_requires_adapter_limits(mocker: MockerFixture) -> None:
    hardware = _hardware(mocker)
    hardware["robot"].adapter.get_limits.return_value = None

    with pytest.raises(ValueError, match="does not declare joint limits"):
        create_task(_cfg(["arm/tool_joint"]), hardware)


def test_limit_resolution_requires_full_adapter_arrays(mocker: MockerFixture) -> None:
    with pytest.raises(ValueError, match="7 entries"):
        create_task(_cfg(["arm/tool_joint"]), _hardware(mocker, limit_len=6))


@pytest.mark.parametrize("limits", [[(0.0, 0.0)], [(float("-inf"), 1.0)]])
def test_task_requires_finite_ordered_limits(limits: list[tuple[float, float]]) -> None:
    with pytest.raises(ValueError, match="finite ordered limits"):
        GripperControlTask(
            "tool",
            GripperControlTaskConfig(joint_names=["arm/tool_joint"]),
            limits=limits,
        )
