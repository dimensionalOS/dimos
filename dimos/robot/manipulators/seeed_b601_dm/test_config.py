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

"""Seeed B601-DM over its direct connection: selectors, mock, tasks and limits."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import ANY

import can_motor_control
from can_motor_control import damiao
import pinocchio
import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.damiao._damiao_test_helpers import mock_connection_rig
from dimos.robot.manipulators.damiao.arm import DamiaoArm
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.seeed_b601_dm.blueprints.basic import (
    B601_DM_PLANNING_MODEL,
    seeed_b601_dm_control,
    seeed_b601_dm_planner_coordinator,
)
from dimos.robot.manipulators.seeed_b601_dm.config import (
    B601_DM,
    B601_DM_JOINT2_LIMITS,
    B601_DM_MODEL,
    GRIPPER_CALIBRATION_TRAVEL_RAD,
    GRIPPER_CURRENT,
    b601_dm_gripper_task,
    b601_dm_model_config,
)
from dimos.robot.manipulators.seeed_b601_dm.connection import B601DMConnection
from dimos.robot.manipulators.seeed_b601_dm.coordinator import SeeedB601DMCoordinator
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_ARM_JOINTS


def test_connection_requires_an_explicit_selector() -> None:
    with pytest.raises(ValueError, match="Field required"):
        B601DMConnection()
    with pytest.raises(ValueError, match="CAN selector must not be empty"):
        B601DMConnection(can_port=" ")


@pytest.mark.parametrize("can_port", ["can0", "/dev/ttyACM0"])
def test_connection_uses_its_explicit_selector(can_port: str) -> None:
    connection = B601DMConnection(can_port=can_port)
    try:
        assert connection.can_ports() == [can_port]
        assert connection.config.source == "b601_dm"
    finally:
        connection.stop()


def test_connection_declares_seeed_gains_and_topology() -> None:
    assert B601_DM.arm_joints == {"arm": list(B601_DM_ARM_JOINTS)}
    assert B601_DM.gripper_joints == {"gripper": "finger_left"}
    assert B601_DM.buses[0].kp == [120.0, 120.0, 120.0, 18.0, 18.0, 18.0]
    assert B601_DM.buses[0].kd == [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
    assert B601_DM.gravity is True
    assert B601_DM.teach_kd is None
    assert B601DMConnection.robot is B601_DM


def test_connection_builds_seeed_motor_table(mocker) -> None:
    class Builder:
        def __init__(self) -> None:
            self.calls: list[tuple[object, ...]] = []

        def add_bus(self, *args):
            self.calls.append(("bus", *args))
            return self

        def add_arm(self, name, **kwargs):
            self.calls.append(("arm", name, kwargs))
            return self

        def add_gripper(self, name, **kwargs):
            self.calls.append(("gripper", name, kwargs))
            return self

        def build(self):
            return self.calls

    builder = Builder()
    mocker.patch.object(can_motor_control, "MotorSpec", side_effect=lambda *args: args)
    # The bus object is transport-specific (SocketCAN, gs_usb, serial); the
    # motor table is what this asserts, so resolve the selector without one.
    mocker.patch.object(DamiaoArm, "_make_can_bus", lambda self, port: port)
    mocker.patch.object(
        can_motor_control, "Robot", SimpleNamespace(builder=lambda: builder), create=True
    )
    arm = DamiaoArm(B601_DM, ["can0"])
    calls = arm._build()

    bus, arm_call, gripper = calls
    assert bus == ("bus", "arm", "can0", ANY)
    assert arm_call[1] == "arm"
    assert [spec[:4] for spec in arm_call[2]["motors"]] == [
        ("joint1", damiao.MotorType.DM4340, 0x01, 0x11),
        ("joint2", damiao.MotorType.DM4340, 0x02, 0x12),
        ("joint3", damiao.MotorType.DM4340, 0x03, 0x13),
        ("joint4", damiao.MotorType.DM4310, 0x04, 0x14),
        ("joint5", damiao.MotorType.DM4310, 0x05, 0x15),
        ("joint6", damiao.MotorType.DM4310, 0x06, 0x16),
    ]
    assert gripper[1] == "gripper"
    options = gripper[2]
    assert options["motor"] == ("gripper_motor", damiao.MotorType.DM4310, 0x07, 0x17)
    assert options["opening_direction"] == "decreasing_position"
    assert options["default_current"] == GRIPPER_CURRENT
    assert options["calibration_travel"] == GRIPPER_CALIBRATION_TRAVEL_RAD


def test_simulation_composes_a_mock_connection(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    blueprint = seeed_b601_dm_control()

    assert not any(issubclass(atom.module, DamiaoConnection) for atom in blueprint.blueprints)
    [mock] = [atom for atom in blueprint.blueprints if atom.module is MockJointConnection]
    assert mock.kwargs["source"] == "arm"
    assert list(mock.kwargs["initial_positions"]) == [*B601_DM_ARM_JOINTS, "finger_left"]
    assert set(mock.kwargs["initial_positions"].values()) == {0.0}
    assert blueprint.remapping_map[(mock.name, "position_command")] == "arm_position_command"


def test_planner_blueprint_requires_a_can_port() -> None:
    with pytest.raises(BlueprintConfigError, match="b601dmconnection.can_port: Field required"):
        BlueprintConfigParser(seeed_b601_dm_planner_coordinator).parse([], environ={})


def test_planner_coordinator_drives_arm_and_gripper() -> None:
    blueprint = seeed_b601_dm_planner_coordinator
    parsed = BlueprintConfigParser(blueprint).parse(["--can-port", "/dev/ttyACM0"], environ={})
    coordinator_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, SeeedB601DMCoordinator)
    )
    planner_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, ManipulationModule)
    )
    connection_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, DamiaoConnection)
    )
    tasks = {task.type: task for task in coordinator_atom.kwargs["tasks"]}

    assert coordinator_atom.kwargs["instance_name"] == "ControlCoordinator"
    assert coordinator_atom.kwargs.get("hardware", []) == []
    assert coordinator_atom.kwargs["zero_stiffness_joints"] == []
    assert connection_atom.module is B601DMConnection
    assert connection_atom.kwargs["source"] == "arm"
    assert parsed.module_kwargs("b601dmconnection")["can_port"] == "/dev/ttyACM0"
    assert tasks["trajectory"].joint_names == [f"arm/{name}" for name in B601_DM_ARM_JOINTS]
    assert tasks["gripper"].name == "b601_dm_gripper"
    assert tasks["gripper"].joint_names == ["arm/finger_left"]

    planner_config = ManipulationModuleConfig.model_validate(
        parsed.module_kwargs(planner_atom.name)
    )
    assert planner_config.gripper_tasks == {"manipulator": "b601_dm_gripper"}
    assert planner_config.model.joint_names == B601_DM_PLANNING_MODEL.joint_names
    assert planner_config.model.joint_names == [f"arm/{name}" for name in B601_DM_ARM_JOINTS]


def test_gripper_task_uses_the_deployed_joint_name() -> None:
    task = b601_dm_gripper_task()

    assert (task.name, task.type, task.joint_names, task.priority, task.params) == (
        "b601_dm_gripper",
        "gripper",
        ["arm/finger_left"],
        20,
        {"open_at": "upper"},
    )


def test_model_plans_arm_joints_to_the_tool_link() -> None:
    model = b601_dm_model_config()

    assert model.joint_names == list(B601_DM_ARM_JOINTS)
    group = model.planning_groups[0]
    assert (group.base_link, group.tip_link) == ("base_link", "end_link")


@pytest.mark.self_hosted
def test_model_moves_fingers_and_widens_joint2_for_the_folded_rest() -> None:
    loaded = B601_DM_MODEL.load()

    assert loaded.get_joint("finger_left").type == "prismatic"
    assert loaded.get_joint("finger_right").mimic == ("finger_left", -1.0, 0.0)
    model = pinocchio.buildModelFromXML(loaded.xml)
    assert tuple(str(name) for name in model.names[1:7]) == B601_DM_ARM_JOINTS
    joint2 = model.getJointId("joint2") - 1
    assert (model.lowerPositionLimit[joint2], model.upperPositionLimit[joint2]) == pytest.approx(
        B601_DM_JOINT2_LIMITS
    )


def test_mock_coordinator_executes_trajectory_and_gripper_command(tmp_path, mocker) -> None:
    clock = mocker.patch("dimos.control.tick_loop.time.perf_counter", return_value=100.0)
    atom = next(
        a
        for a in seeed_b601_dm_planner_coordinator.blueprints
        if issubclass(a.module, ControlCoordinator)
    )
    coordinator = atom.module(publish_joint_state=False, **atom.kwargs)
    with mock_connection_rig(B601DMConnection, coordinator, tmp_path=tmp_path) as rig:
        target = [0.01, -0.01, -0.01, 0.01, 0.01, 0.01]
        trajectory = JointTrajectory(
            joint_names=[f"arm/{name}" for name in B601_DM_ARM_JOINTS],
            points=[
                TrajectoryPoint(positions=[0.0] * 6, velocities=[0.0] * 6, time_from_start=0.0),
                TrajectoryPoint(positions=target, velocities=[0.0] * 6, time_from_start=0.3),
            ],
        )
        result = coordinator.task_invoke("joint_trajectory", "execute", {"trajectory": trajectory})
        assert result.status is TrajectoryExecutionStatus.ACCEPTED
        # The first tick after submission anchors the start point at now.
        rig.tick()
        clock.return_value = 101.0
        rig.tick()
        assert [rig.positions[f"arm/joint{index}"] for index in range(1, 7)] == pytest.approx(
            target
        )

        assert coordinator.task_invoke("b601_dm_gripper", "set_normalized", {"values": [1.0]})
        rig.tick()
        # finger_left is fully open at 0.05 m.
        assert rig.positions["arm/finger_left"] == pytest.approx(0.05)
