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

"""Every retained entry uses one connection and one consistent deployed schema."""

import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.global_config import global_config
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.damiao._damiao_test_helpers import mock_connection_rig
from dimos.robot.manipulators.damiao.connection import (
    DamiaoConnection,
    zero_stiffness_joints,
)
from dimos.robot.manipulators.dual_makermods_metal.blueprints.basic import (
    dual_makermods_metal_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    coordinator_dual_openyam,
    dual_openyam_control,
    dual_openyam_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.makermods_metal.blueprints.basic import coordinator_makermods_metal
from dimos.robot.manipulators.makermods_metal.blueprints.connection import (
    makermods_metal_connection_control,
)
from dimos.robot.manipulators.makermods_metal.blueprints.planning import (
    makermods_metal_planner_coordinator,
)
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.robot.manipulators.openarm.blueprints.basic import (
    coordinator_openarm,
    openarm_planner_coordinator,
)
from dimos.robot.manipulators.openarm.blueprints.connection import openarm_connection_control
from dimos.robot.manipulators.openarm.blueprints.teleop import teleop_webxr_openarm
from dimos.robot.manipulators.openyam.blueprints.basic import (
    coordinator_openyam,
    openyam_control,
    openyam_planner_coordinator,
)
from dimos.robot.manipulators.openyam.blueprints.connection import openyam_connection_control
from dimos.robot.manipulators.openyam.blueprints.learning_collection import (
    OPENYAM_TEACH_JOINTS,
    openyam_teach_collection,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
    keyboard_teleop_openyam_planner,
    teleop_webxr_openyam,
)
from dimos.robot.manipulators.openyam.config import OPENYAM, make_openyam_model_config
from dimos.robot.manipulators.openyam.connection import OpenYamConnection

CASES = [
    ("coordinator-makermods-metal", coordinator_makermods_metal),
    ("makermods-metal-planner-coordinator", makermods_metal_planner_coordinator),
    ("dual-makermods-metal-planner-coordinator", dual_makermods_metal_planner_coordinator),
    ("coordinator-openyam", coordinator_openyam),
    ("openyam-planner-coordinator", openyam_planner_coordinator),
    ("keyboard-teleop-openyam", keyboard_teleop_openyam),
    ("keyboard-teleop-openyam-planner", keyboard_teleop_openyam_planner),
    ("teleop-webxr-openyam", teleop_webxr_openyam),
    ("coordinator-dual-openyam", coordinator_dual_openyam),
    ("dual-openyam-planner-coordinator", dual_openyam_planner_coordinator),
    ("teleop-webxr-dual-openyam", teleop_webxr_dual_openyam),
    ("coordinator-openarm", coordinator_openarm),
    ("openarm-planner-coordinator", openarm_planner_coordinator),
    ("teleop-webxr-openarm", teleop_webxr_openarm),
    ("openyam-connection-control", openyam_connection_control),
    ("openarm-connection-control", openarm_connection_control),
    ("makermods-metal-connection-control", makermods_metal_connection_control),
]


@pytest.mark.parametrize("name,blueprint", CASES, ids=[name for name, _ in CASES])
def test_retained_entry_routes_every_task_through_its_connection(name, blueprint, mocker):
    load = mocker.patch(
        "dimos.robot.assets.model.RobotModel.load", side_effect=AssertionError("must stay lazy")
    )
    assert name in all_blueprints
    connections = [
        atom for atom in blueprint.blueprints if issubclass(atom.module, DamiaoConnection)
    ]
    coordinators = [
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    ]
    assert len(connections) == len(coordinators) == 1
    connection, coordinator = connections[0], coordinators[0]
    assert connection.module.deployment == "python"
    assert connection.kwargs["source"] == "arm"
    assert coordinator.kwargs.get("hardware", []) == []
    joints = {f"arm/{name}" for name in connection.module.robot.joint_names}
    for task in coordinator.kwargs["tasks"]:
        assert set(task.joint_names) <= joints
    load.assert_not_called()


def test_mock_metal_shell_commands_drive_real_coordinator_connection_path(tmp_path, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    clock = mocker.patch("dimos.control.tick_loop.time.perf_counter", return_value=100.0)
    atom = next(
        atom
        for atom in makermods_metal_connection_control.blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    coordinator = atom.module(publish_joint_state=False, **atom.kwargs)
    with mock_connection_rig(MetalConnection, coordinator, tmp_path=tmp_path) as rig:
        # joint7 is closed at 0 m and fully open at -0.05 m.
        assert coordinator.task_invoke("arm_gripper", "set_normalized", {"values": [0.6]})
        rig.tick()
        assert rig.positions == {
            **{f"arm/joint{i}": 0.0 for i in range(1, 7)},
            "arm/joint7": pytest.approx(-0.03),
        }
        request = JointTrajectory(
            joint_names=["arm/joint2"],
            points=[
                TrajectoryPoint(time_from_start=0.0, positions=[0.0]),
                TrajectoryPoint(time_from_start=0.5, positions=[-0.02]),
            ],
        )
        assert coordinator.execute_trajectory(request).status is TrajectoryExecutionStatus.ACCEPTED
        rig.tick()
        clock.return_value = 101.0
        rig.tick()
        assert rig.positions["arm/joint2"] == pytest.approx(-0.02)
        assert rig.positions["arm/joint7"] == pytest.approx(-0.03)


@pytest.mark.self_hosted
def test_simulation_composes_an_independent_dual_arm_mock(mocker):
    mocker.patch.object(global_config, "simulation", "mujoco")
    blueprint = dual_openyam_control()
    assert not any(issubclass(atom.module, DamiaoConnection) for atom in blueprint.blueprints)
    [mock] = [atom for atom in blueprint.blueprints if atom.module is MockJointConnection]
    assert mock.kwargs["source"] == "arm"
    assert list(mock.kwargs["initial_positions"]) == [
        *[f"left_joint{i}" for i in range(1, 7)],
        *[f"right_joint{i}" for i in range(1, 7)],
        "left_gripper",
        "right_gripper",
    ]
    assert mock.kwargs["initial_positions"]["left_gripper"] == 0.0
    assert mock.kwargs["initial_positions"]["right_gripper"] == 0.0
    assert mock.kwargs["initial_positions"]["left_joint1"] == pytest.approx(0.0)
    assert blueprint.remapping_map[(mock.name, "position_command")] == "arm_position_command"


def test_blueprint_composition_is_coordinator_connection_and_remaps():
    blueprint = openyam_control()
    [connection] = [
        atom for atom in blueprint.blueprints if issubclass(atom.module, DamiaoConnection)
    ]
    [coordinator] = [
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    ]
    assert connection.module is OpenYamConnection
    assert connection.kwargs == {"source": "arm", "teach": False}
    assert coordinator.kwargs["model"] == make_openyam_model_config(robot="arm").model
    assert coordinator.kwargs["zero_stiffness_joints"] == []
    assert blueprint.remapping_map == {
        (connection.name, "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }


def test_teaching_blueprint_declares_zero_stiffness_only_when_teaching():
    blueprint = openyam_teach_collection
    [coordinator] = [
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    ]
    [connection] = [
        atom for atom in blueprint.blueprints if issubclass(atom.module, DamiaoConnection)
    ]
    assert connection.kwargs["teach"] is True
    assert coordinator.kwargs["zero_stiffness_joints"] == OPENYAM_TEACH_JOINTS
    assert zero_stiffness_joints(OPENYAM, teach=False) == []
    assert zero_stiffness_joints(DUAL_OPENYAM, teach=True) == [
        *[f"arm/{name}" for name in DUAL_OPENYAM.arm_joint_names],
    ]
