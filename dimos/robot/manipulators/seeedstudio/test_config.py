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

from collections.abc import Callable, Iterator
import importlib
from queue import Queue
import time
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.seeedstudio.adapter import (
    GRIPPER_MAX_OPENING_M,
    JOINT2_REST_MARGIN,
    gripper_opening_to_motor,
)
from dimos.hardware.manipulators.seeedstudio.protocol import (
    DISABLE,
    ENABLE,
    Feedback,
    MotorParameters,
)
from dimos.hardware.spec import JointLimits
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.common.blueprints import trajectory_task
from dimos.robot.manipulators.seeedstudio.blueprints import basic
from dimos.robot.manipulators.seeedstudio.config import (
    make_seeedstudio_model_config,
    seeedstudio_gripper_task,
    seeedstudio_hardware,
)

JOINTS = [*(f"joint{i}" for i in range(1, 7)), "arm/gripper"]


@pytest.mark.parametrize(
    "port,simulation,expected",
    [(None, "", "mock"), ("test-only", "", "seeedstudio_b601_dm"), ("test-only", "mock", "mock")],
)
def test_hardware_is_mock_unless_a_can_port_is_given(
    port: str | None, simulation: str, expected: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(global_config, "can_port", port)
    monkeypatch.setattr(global_config, "simulation", simulation)
    hardware = seeedstudio_hardware()
    assert hardware.adapter_type == expected
    assert hardware.joints == JOINTS
    assert hardware.auto_enable
    if expected == "mock":
        assert isinstance(hardware.limits, JointLimits)
        assert hardware.limits.position_upper[1] == JOINT2_REST_MARGIN
        assert hardware.limits.position_upper[-1] == GRIPPER_MAX_OPENING_M
    else:
        assert hardware.address == port
        assert hardware.limits is None


def test_planner_coordinator_drives_arm_and_gripper(monkeypatch: pytest.MonkeyPatch) -> None:
    try:
        monkeypatch.setattr(global_config, "can_port", "test-only")
        monkeypatch.setattr(global_config, "simulation", "")
        blueprint = importlib.reload(basic).seeedstudio_planner_coordinator
        parsed = BlueprintConfigParser(blueprint).parse(environ={})
        coordinator_atom = next(
            a for a in blueprint.active_blueprints if issubclass(a.module, ControlCoordinator)
        )
        planner_atom = next(
            a for a in blueprint.active_blueprints if issubclass(a.module, ManipulationModule)
        )
        config = ControlCoordinatorConfig.model_validate(
            parsed.module_kwargs(coordinator_atom.name)
        )
        assert config.hardware[0].adapter_type == "seeedstudio_b601_dm"
        tasks = {task.type: task for task in config.tasks}
        assert tasks["trajectory"].joint_names == JOINTS
        assert tasks["gripper"].name == "arm_gripper"
        assert tasks["gripper"].joint_names == ["arm/gripper"]
        assert planner_atom.kwargs["model"].gripper_hardware_id == "arm"
    finally:
        monkeypatch.undo()
        importlib.reload(basic)


def test_model_plans_arm_joints_to_the_tool_link() -> None:
    model = make_seeedstudio_model_config()
    assert model.joint_names == JOINTS[:6]
    group = model.planning_groups[0]
    assert (group.base_link, group.tip_link) == ("base_link", "end_link")
    assert model.gripper_hardware_id == "arm"


@pytest.fixture
def coordinator_factory(mocker: MockerFixture) -> Iterator[Callable[..., ControlCoordinator]]:
    # A regression must never open a USB port during software integration tests.
    mocker.patch("serial.Serial", side_effect=AssertionError("Hardware IO forbidden in this test"))
    coordinators: list[ControlCoordinator] = []

    def make(hardware: HardwareComponent) -> ControlCoordinator:
        blueprint = ControlCoordinator.blueprint(
            tick_rate=50.0,
            hardware=[hardware],
            tasks=[trajectory_task(hardware), seeedstudio_gripper_task()],
        )
        parsed = BlueprintConfigParser(blueprint).parse(environ={})
        coordinator = ControlCoordinator(
            **parsed.module_kwargs(blueprint.active_blueprints[0].name)
        )
        coordinators.append(coordinator)
        return coordinator

    yield make
    for coordinator in coordinators:
        coordinator.stop()


def wait_for(states: "Queue[JointState]", predicate: Callable[[JointState], bool]) -> JointState:
    deadline = time.monotonic() + 3.0
    while True:
        state = states.get(timeout=max(0.001, deadline - time.monotonic()))
        if predicate(state):
            return state
        assert time.monotonic() < deadline, "coordinator did not reach the expected state"


def test_mock_coordinator_executes_trajectory_and_gripper_command(
    coordinator_factory: Callable[..., ControlCoordinator],
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "can_port", None)
    coordinator = coordinator_factory(seeedstudio_hardware())
    states: Queue[JointState] = Queue()
    mocker.patch.object(coordinator.coordinator_joint_state, "publish", side_effect=states.put)
    coordinator.start()
    assert wait_for(states, lambda _: True).name == JOINTS

    target = [0.01, -0.01, -0.01, 0.01, 0.01, 0.01, 0.0]
    trajectory = JointTrajectory(
        joint_names=JOINTS[:6],
        points=[
            TrajectoryPoint(positions=[0.0] * 6, velocities=[0.0] * 6, time_from_start=0.0),
            TrajectoryPoint(positions=target[:6], velocities=[0.0] * 6, time_from_start=0.3),
        ],
    )
    assert coordinator.execute_trajectory(trajectory).status == TrajectoryExecutionStatus.ACCEPTED
    wait_for(states, lambda s: list(s.position[:6]) == pytest.approx(target[:6]))

    assert coordinator.task_invoke("arm_gripper", "set_normalized", {"values": [1.0]})
    wait_for(states, lambda s: s.position[6] == pytest.approx(GRIPPER_MAX_OPENING_M))


def test_real_coordinator_enables_on_start_and_disables_on_stop(
    coordinator_factory: Callable[..., ControlCoordinator],
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    monkeypatch.setattr(global_config, "can_port", "test-only")
    monkeypatch.setattr(global_config, "simulation", "")
    bus: MagicMock = mocker.patch(
        "dimos.hardware.manipulators.seeedstudio.adapter.DmSerialTransport"
    ).return_value
    enabled: set[int] = set()
    home = [0.0] * 6 + [gripper_opening_to_motor(0.0)]

    def send(mid: int, payload: bytes, *_: float) -> None:
        if payload == ENABLE:
            enabled.add(mid)
        elif payload == DISABLE:
            enabled.discard(mid)

    def feedback(params: MotorParameters) -> Feedback:
        status = int(params.motor_id in enabled)
        return Feedback(home[params.motor_id - 1], 0.0, 0.0, status, 25, 25, time.monotonic())

    bus.is_open.return_value = True
    bus.parameters.side_effect = lambda mid, fid: MotorParameters(mid, fid, 2, 12.5, 10, 28, 0)
    bus.feedback.side_effect = feedback
    bus.read_register.return_value = 2
    bus.send.side_effect = send

    coordinator = coordinator_factory(seeedstudio_hardware())
    coordinator.start()
    assert enabled == set(range(1, 8))
    coordinator.stop()
    assert enabled == set()
