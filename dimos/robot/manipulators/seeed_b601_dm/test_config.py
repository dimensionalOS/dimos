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

import pinocchio
import pytest
from pytest_mock import MockerFixture

from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.seeed_b601_dm.blueprints.basic import (
    coordinator_seeed_b601_dm,
    seeed_b601_dm_planner_coordinator,
)
from dimos.robot.manipulators.seeed_b601_dm.config import (
    B601_DM_JOINT2_LIMITS,
    B601_DM_MODEL,
    b601_dm_hardware,
    b601_dm_model_config,
)
from dimos.robot.manipulators.seeed_b601_dm.coordinator import SeeedB601DMCoordinator
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_ARM_JOINTS, B601_DM_JOINTS


def test_hardware_defaults_to_mock_without_a_can_port() -> None:
    hardware = b601_dm_hardware()

    assert hardware.hardware_type == HardwareType.WHOLE_BODY
    assert hardware.adapter_type == "mock_whole_body"
    assert hardware.joints == list(B601_DM_JOINTS)
    assert hardware.adapter_kwargs == {}
    assert hardware.limits is not None
    assert hardware.limits.position_upper[-1] == 1.0


@pytest.mark.parametrize("can_port", ["/dev/ttyACM0", "can0"])
def test_hardware_uses_the_physical_adapter_with_a_can_port(can_port: str) -> None:
    hardware = b601_dm_hardware(can_port=can_port)

    assert hardware.adapter_type == "seeed_b601_dm"
    assert hardware.address is None
    assert hardware.limits is None
    assert hardware.auto_enable
    assert hardware.adapter_kwargs["runtime_config"] == DamiaoRuntimeConfig(
        bus_devices={"b601_dm": can_port}, gravity_comp=True
    )


def test_hardware_rejects_an_empty_can_port() -> None:
    with pytest.raises(ValueError, match="must not be empty"):
        b601_dm_hardware(can_port=" ")


def test_can_port_and_gravity_comp_are_blueprint_cli_options() -> None:
    parsed = BlueprintConfigParser(seeed_b601_dm_planner_coordinator).parse(
        ["--can-port", "/dev/ttyACM0", "--gravity-comp", "false"],
        environ={},
    )

    coordinator = parsed.module_kwargs("ControlCoordinator")
    assert coordinator["can_port"] == "/dev/ttyACM0"
    assert coordinator["gravity_comp"] is False


def test_coordinator_rejects_the_generic_addresses_map() -> None:
    coordinator = SeeedB601DMCoordinator(addresses={"b601_dm": "/dev/ttyACM0"})
    try:
        with pytest.raises(ValueError, match="--can-port"):
            coordinator._setup_from_config()
    finally:
        coordinator.stop()


def test_simulation_runs_mock_hardware_even_with_a_can_port(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    # A path that is never a device, so a regression cannot reach a real arm.
    coordinator = SeeedB601DMCoordinator(can_port="/dev/null/b601-test")
    try:
        coordinator._setup_from_config()
        assert coordinator.config.hardware[0].adapter_type == "mock_whole_body"
    finally:
        coordinator.stop()


def test_planner_coordinator_drives_arm_and_gripper() -> None:
    blueprint = seeed_b601_dm_planner_coordinator
    coordinator_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, SeeedB601DMCoordinator)
    )
    planner_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, ManipulationModule)
    )
    tasks = {task.type: task for task in coordinator_atom.kwargs["tasks"]}

    assert coordinator_atom.kwargs["instance_name"] == "ControlCoordinator"
    assert tasks["trajectory"].joint_names == list(B601_DM_ARM_JOINTS)
    assert tasks["gripper"].name == "b601_dm_gripper"
    assert tasks["gripper"].joint_names == ["arm/gripper"]
    assert planner_atom.kwargs["model"].gripper_hardware_id == "b601_dm"


def test_model_plans_arm_joints_to_the_tool_link() -> None:
    model = b601_dm_model_config()

    assert model.joint_names == list(B601_DM_ARM_JOINTS)
    group = model.planning_groups[0]
    assert (group.base_link, group.tip_link) == ("base_link", "end_link")
    assert model.gripper_hardware_id == "b601_dm"


@pytest.mark.self_hosted
def test_model_fixes_fingers_and_widens_joint2_for_the_folded_rest() -> None:
    loaded = B601_DM_MODEL.load()

    assert {joint.name for joint in loaded.joints if joint.type == "fixed"}.issuperset(
        {"finger_left", "finger_right"}
    )
    model = pinocchio.buildModelFromXML(loaded.xml)
    assert tuple(str(name) for name in model.names[1:]) == B601_DM_ARM_JOINTS
    joint2 = model.getJointId("joint2") - 1
    assert (model.lowerPositionLimit[joint2], model.upperPositionLimit[joint2]) == pytest.approx(
        B601_DM_JOINT2_LIMITS
    )


@pytest.fixture
def mock_coordinator() -> Iterator[ControlCoordinator]:
    parsed = BlueprintConfigParser(coordinator_seeed_b601_dm).parse(environ={})
    coordinator = SeeedB601DMCoordinator(**parsed.module_kwargs("ControlCoordinator"))
    yield coordinator
    coordinator.stop()


def wait_for(states: "Queue[JointState]", predicate: Callable[[JointState], bool]) -> JointState:
    deadline = time.monotonic() + 3.0
    while True:
        state = states.get(timeout=max(0.001, deadline - time.monotonic()))
        if predicate(state):
            return state
        assert time.monotonic() < deadline, "coordinator did not reach the expected state"


def test_mock_coordinator_executes_trajectory_and_gripper_command(
    mock_coordinator: ControlCoordinator,
    mocker: MockerFixture,
) -> None:
    states: Queue[JointState] = Queue()
    mocker.patch.object(mock_coordinator.coordinator_joint_state, "publish", side_effect=states.put)
    mock_coordinator.start()
    assert wait_for(states, lambda _: True).name == list(B601_DM_JOINTS)

    target = [0.01, -0.01, -0.01, 0.01, 0.01, 0.01]
    trajectory = JointTrajectory(
        joint_names=list(B601_DM_ARM_JOINTS),
        points=[
            TrajectoryPoint(positions=[0.0] * 6, velocities=[0.0] * 6, time_from_start=0.0),
            TrajectoryPoint(positions=target, velocities=[0.0] * 6, time_from_start=0.3),
        ],
    )
    result = mock_coordinator.execute_trajectory(trajectory)
    assert result.status == TrajectoryExecutionStatus.ACCEPTED
    wait_for(states, lambda s: list(s.position[:6]) == pytest.approx(target))

    assert mock_coordinator.task_invoke("b601_dm_gripper", "set_normalized", {"values": [1.0]})
    wait_for(states, lambda s: s.position[6] == pytest.approx(1.0))
