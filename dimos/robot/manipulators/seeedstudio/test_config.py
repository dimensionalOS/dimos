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
from queue import Queue
import time

import pytest
from pytest_mock import MockerFixture

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.seeedstudio.blueprints import basic
from dimos.robot.manipulators.seeedstudio.config import (
    make_seeedstudio_model_config,
    seeedstudio_gripper_task,
    seeedstudio_hardware,
)

JOINTS = [*(f"joint{i}" for i in range(1, 7)), "arm/gripper"]


@pytest.mark.parametrize(
    "address,simulation,expected",
    [
        (None, "", "mock_whole_body"),
        ("/dev/ttyACM0", "", "seeedstudio_b601_dm"),
        ("can0", "", "seeedstudio_b601_dm"),
        ("/dev/ttyACM0", "mujoco", "mock_whole_body"),
    ],
)
def test_hardware_is_mock_unless_an_address_reaches_it(
    address: str | None,
    simulation: str,
    expected: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", simulation)
    hardware = seeedstudio_hardware()
    assert hardware.hardware_type == HardwareType.WHOLE_BODY
    assert hardware.joints == JOINTS
    assert hardware.auto_enable
    assert hardware.adapter_kwargs["runtime_config"] == DamiaoRuntimeConfig(gravity_comp=False)

    addresses = {"arm": address} if address else {}
    coordinator = ControlCoordinator(addresses=addresses, hardware=[hardware])
    try:
        resolved = coordinator._resolve_component(hardware)
    finally:
        coordinator.stop()
    assert resolved.adapter_type == expected
    if expected == "mock_whole_body":
        assert resolved.limits is not None
        assert resolved.limits.position_upper[-1] == 1.0
    else:
        assert resolved.address == address


def test_planner_coordinator_drives_arm_and_gripper() -> None:
    blueprint = basic.seeedstudio_planner_coordinator
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    coordinator_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, ControlCoordinator)
    )
    planner_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, ManipulationModule)
    )
    config = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator_atom.name))
    assert config.hardware[0].adapter_type == "seeedstudio_b601_dm"
    tasks = {task.type: task for task in config.tasks}
    assert tasks["trajectory"].joint_names == JOINTS
    assert tasks["gripper"].name == "arm_gripper"
    assert tasks["gripper"].joint_names == ["arm/gripper"]
    assert planner_atom.kwargs["model"].gripper_hardware_id == "arm"


def test_model_plans_arm_joints_to_the_tool_link() -> None:
    model = make_seeedstudio_model_config()
    assert model.joint_names == JOINTS[:6]
    group = model.planning_groups[0]
    assert (group.base_link, group.tip_link) == ("base_link", "end_link")
    assert model.gripper_hardware_id == "arm"


@pytest.fixture
def coordinator_factory() -> Iterator[Callable[[HardwareComponent], ControlCoordinator]]:
    coordinators: list[ControlCoordinator] = []

    def make(hardware: HardwareComponent) -> ControlCoordinator:
        blueprint = ControlCoordinator.blueprint(
            tick_rate=50.0,
            hardware=[hardware],
            tasks=[joint_trajectory_task(JOINTS), seeedstudio_gripper_task()],
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
    coordinator_factory: Callable[[HardwareComponent], ControlCoordinator],
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    coordinator = coordinator_factory(seeedstudio_hardware())
    states: Queue[JointState] = Queue()
    mocker.patch.object(coordinator.coordinator_joint_state, "publish", side_effect=states.put)
    coordinator.start()
    assert wait_for(states, lambda _: True).name == JOINTS

    target = [0.01, -0.01, -0.01, 0.01, 0.01, 0.01]
    trajectory = JointTrajectory(
        joint_names=JOINTS[:6],
        points=[
            TrajectoryPoint(positions=[0.0] * 6, velocities=[0.0] * 6, time_from_start=0.0),
            TrajectoryPoint(positions=target, velocities=[0.0] * 6, time_from_start=0.3),
        ],
    )
    assert coordinator.execute_trajectory(trajectory).status == TrajectoryExecutionStatus.ACCEPTED
    wait_for(states, lambda s: list(s.position[:6]) == pytest.approx(target))

    assert coordinator.task_invoke("arm_gripper", "set_normalized", {"values": [1.0]})
    wait_for(states, lambda s: s.position[6] == pytest.approx(1.0))
