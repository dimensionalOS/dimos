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

"""Complete Alfred hardware and remote-operator blueprint."""

from __future__ import annotations

from dimos.control.blueprints.mobile import _flowbase_twist_base
from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_MOTOR_TRANSPORTS,
    PILLAR_SERVO_TASK_NAME,
)
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_LIFT_JOINT,
    PillarConnection,
    pillar_hardware,
)
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    openarm_bimanual_model_config,
    openarm_hardware,
)
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

_base_joints = make_twist_base_joints("base")
_flowbase_hardware = _flowbase_twist_base()
_pillar_hardware = pillar_hardware()
_openarm_hardware = openarm_hardware()

alfred = (
    autoconnect(
        PillarConnection.blueprint(),
        planner(
            robots=[openarm_bimanual_model_config()],
            visualization={"backend": "viser"},
        ),
        ControlCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[_flowbase_hardware, _pillar_hardware, _openarm_hardware],
            tasks=[
                TaskConfig(
                    name="vel_base",
                    type="velocity",
                    joint_names=_base_joints,
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
                    name=JOINT_TRAJECTORY_TASK_NAME,
                    type="trajectory",
                    joint_names=list(OPENARM_ARM_JOINTS),
                    priority=10,
                    params={"start_position_tolerance": 0.05},
                ),
            ],
        ),
        vis_module("rerun"),
    )
    .transports(dict(PILLAR_MOTOR_TRANSPORTS))
    .remappings(
        [
            (ControlCoordinator, "twist_command", "cmd_vel"),
            (RerunWebSocketServer, "tele_cmd_vel", "cmd_vel"),
            (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
        ]
    )
    .global_config(n_workers=6)
)
