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

from pathlib import Path

import pytest
from pytest_mock import MockerFixture

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.keys import Interface
from dimos.control.coordinator import TaskConfig
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.gripper_task.gripper_task import (
    GripperControlTask,
    GripperControlTaskConfig,
    create_task,
)
from dimos.control.tasks.registry import control_task_registry
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.hardware.spec import JointLimits
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.assets.model import RobotModel


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


def test_configured_hardware_limits_override_adapter_limits(mocker: MockerFixture) -> None:
    hardware = _hardware(mocker)
    configured = JointLimits(
        position_lower=[0.0] * 7,
        position_upper=[*([2.0] * 6), 0.1],
        velocity_max=[1.0] * 7,
    )
    hardware["robot"].component.limits = configured

    task = create_task(_cfg(["arm/tool_joint"]), hardware)
    assert task.set_normalized([0.5])
    output = task.compute(_state())

    assert output is not None
    assert output.positions == pytest.approx([0.05])
    hardware["robot"].adapter.get_limits.assert_not_called()


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


@pytest.fixture
def source(tmp_path: Path):
    def build(robot: str, *, position: bool = True, continuous: bool = False) -> ConnectionSource:
        path = tmp_path / f"{robot}.urdf"
        bounds = "" if continuous else 'lower="0" upper="0.08"'
        kind = "continuous" if continuous else "prismatic"
        path.write_text(
            f'<robot name="{robot}"><link name="base"/><link name="tip"/>'
            f'<joint name="jaw" type="{kind}"><parent link="base"/><child link="tip"/>'
            f'<limit {bounds} velocity="1" effort="10"/></joint></robot>'
        )
        interface = Interface.POSITION if position else Interface.VELOCITY
        return ConnectionSource(
            ConnectionConfig(
                source=robot,
                model=RobotModel.from_file(path),
                joint_commands={"jaw": (interface,)},
            ),
            commands=["position_command" if position else "velocity_command"],
            readings=["joint_state"],
            reading_timeout_s=1.0,
        )

    return build


def test_registry_resolves_each_gripper_joint_from_its_own_source(source) -> None:
    left, right = source("left"), source("right")
    task = control_task_registry.create(
        "gripper", _cfg(["right/jaw", "left/jaw"]), connections={"left": left, "right": right}
    )
    assert isinstance(task, GripperControlTask)
    assert task.set_normalized([0.25, 0.75])
    output = task.compute(_state())
    assert output is not None
    assert output.joint_names == ["right/jaw", "left/jaw"]
    assert output.positions == pytest.approx([0.02, 0.06])
    assert left.position_limit("left/jaw").hi == 0.08
    assert not left.ready_for_control()  # Limit discovery never relies on feedback.


@pytest.mark.parametrize(
    "names,expected",
    [
        ([], "requires at least one joint"),
        (["arm/jaw", "arm/jaw"], "duplicates"),
        (["missing/jaw"], "no connection source"),
        (["arm/unknown"], "no declared position command"),
    ],
)
def test_connection_gripper_rejects_invalid_joint_selection(source, names, expected) -> None:
    with pytest.raises(ValueError, match=expected):
        create_task(_cfg(names), {}, connections={"arm": source("arm")})


@pytest.mark.parametrize("position,continuous", [(False, False), (True, True)])
def test_connection_gripper_requires_a_bounded_position_command(
    source, position, continuous
) -> None:
    connection = source("arm", position=position, continuous=continuous)
    with pytest.raises(ValueError, match="position command|bounded position range"):
        create_task(_cfg(["arm/jaw"]), {}, connections={"arm": connection})


def _gated_task() -> GripperControlTask:
    return GripperControlTask(
        "tool",
        GripperControlTaskConfig(
            joint_names=["arm/tool_joint"], require_feedback=True, accept_broadcast=False
        ),
        limits=[(0.0, 850.0)],
    )


def test_require_feedback_drops_the_latched_target_and_never_replays_it() -> None:
    task = _gated_task()
    assert task.set_normalized([0.5])
    output = task.compute(_state(**{"arm/tool_joint": 100.0}))
    assert output is not None and output.positions == [425.0]

    # A snapshot that lost the joint drops the latched target...
    assert task.compute(_state()) is None
    # ...and feedback returning does not replay it: a new command is required.
    assert task.compute(_state(**{"arm/tool_joint": 100.0})) is None
    assert task.get_normalized() is not None  # the readback itself still works

    assert task.set_normalized([0.5])
    output = task.compute(_state(**{"arm/tool_joint": 100.0}))
    assert output is not None and output.positions == [425.0]


def test_the_measured_snapshot_is_replaced_never_merged() -> None:
    task = _task()  # require_feedback off: the target survives, the readback does not
    task.compute(_state(**{"arm/tool_joint": 425.0}))
    assert task.get_normalized() == [0.5]

    task.compute(_state())
    assert task.get_normalized() is None


def test_a_broadcast_refusing_task_ignores_the_unscoped_stream() -> None:
    task = _gated_task()
    assert task.on_gripper_command(Float32(data=0.5), 0.0) is False
    assert task.compute(_state(**{"arm/tool_joint": 0.0})) is None
    # Addressed directly, the same value works.
    assert task.set_normalized([0.5])


def test_a_command_latched_during_an_outage_goes_out_when_feedback_returns() -> None:
    task = _gated_task()
    # Latched with no live feedback: never sent, so it is held, not dropped —
    # this is the explicit new command a recovered gripper waits for.
    assert task.set_normalized([0.5])
    assert task.compute(_state()) is None
    assert task.compute(_state()) is None
    output = task.compute(_state(**{"arm/tool_joint": 100.0}))
    assert output is not None and output.positions == [425.0]
