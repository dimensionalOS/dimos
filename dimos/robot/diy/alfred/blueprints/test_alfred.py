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

from typing import Any, cast

from dimos.control.components import HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.diy.alfred.blueprints.alfred import alfred
from dimos.robot.diy.alfred.blueprints.pillar import PILLAR_SERVO_TASK_NAME
from dimos.robot.diy.alfred.pillar_connection import PILLAR_LIFT_JOINT, PillarConnection
from dimos.robot.manipulators.openarm.config import OPENARM_ARM_JOINTS
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _atom(module: type[Any]) -> Any:
    return next(atom for atom in alfred.blueprints if atom.module is module)


def test_alfred_composes_all_hardware_under_one_coordinator() -> None:
    coordinator = _atom(ControlCoordinator)
    hardware = coordinator.kwargs["hardware"]
    tasks = cast("list[TaskConfig]", coordinator.kwargs["tasks"])

    assert [component.hardware_id for component in hardware] == ["base", "pillar", "openarm"]
    assert [component.hardware_type for component in hardware] == [
        HardwareType.BASE,
        HardwareType.WHOLE_BODY,
        HardwareType.WHOLE_BODY,
    ]
    assert tasks == [
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=make_twist_base_joints("base"),
            priority=10,
            params={"timeout": 0.2, "zero_on_timeout": True},
        ),
        TaskConfig(
            name=PILLAR_SERVO_TASK_NAME,
            type="servo",
            joint_names=[PILLAR_LIFT_JOINT],
            priority=10,
            auto_start=True,
        ),
        TaskConfig(
            name="joint_trajectory",
            type="trajectory",
            joint_names=list(OPENARM_ARM_JOINTS),
            priority=10,
            params={"start_position_tolerance": 0.05},
        ),
    ]


def test_alfred_composes_pillar_viser_and_rerun_teleop() -> None:
    assert _atom(PillarConnection)
    assert _atom(ManipulationModule).kwargs["visualization"] == {"backend": "viser"}

    coordinator = _atom(ControlCoordinator)
    rerun_server = _atom(RerunWebSocketServer)
    websocket_vis = _atom(WebsocketVisModule)
    assert alfred.remapping_map[coordinator.name, "twist_command"] == "cmd_vel"
    assert alfred.remapping_map[rerun_server.name, "tele_cmd_vel"] == "cmd_vel"
    assert alfred.remapping_map[websocket_vis.name, "tele_cmd_vel"] == "cmd_vel"


def test_alfred_preserves_pillar_motor_topics() -> None:
    assert alfred.transport_map["motor_command", MotorCommandArray].args[0] == (
        "/pillar/motor_command"
    )
    assert alfred.transport_map["motor_states", JointState].args[0] == "/pillar/motor_states"
