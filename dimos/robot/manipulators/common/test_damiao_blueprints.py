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

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.common._damiao_test_helpers import mock_connection_rig
from dimos.robot.manipulators.common.damiao_connection import DamiaoConnection
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    coordinator_dual_openyam,
    dual_openyam_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.makermods_metal.blueprints.connection import (
    makermods_metal_connection_control,
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
    openyam_planner_coordinator,
)
from dimos.robot.manipulators.openyam.blueprints.connection import openyam_connection_control
from dimos.robot.manipulators.openyam.blueprints.learning_quest_collection import (
    openyam_quest_collection,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
    keyboard_teleop_openyam_planner,
    teleop_webxr_openyam,
)

CASES = [
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
    ("openyam-quest-collection", openyam_quest_collection),
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
    model = connection.module.models()
    for task in coordinator.kwargs["tasks"]:
        assert set(task.joint_names) <= set(model.joints)
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
    with mock_connection_rig(MetalConnection, coordinator, tmp_path=tmp_path, mocker=mocker) as rig:
        assert coordinator.task_invoke("arm_gripper", "set_normalized", {"values": [0.6]})
        rig.tick()
        assert rig.positions == {**{f"arm/joint{i}": 0.0 for i in range(1, 7)}, "arm/gripper": 0.6}
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
        assert rig.positions["arm/gripper"] == pytest.approx(0.6)
