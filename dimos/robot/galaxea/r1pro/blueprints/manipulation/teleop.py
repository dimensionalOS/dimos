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

"""Fake-hardware R1 Pro whole-upper-body teleoperation coordinator."""

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_model_config,
)
from dimos.robot.galaxea.r1pro.teleop_ik import (
    R1PRO_TELEOP_PINK,
    R1ProPinkPoseTargetSolver,
)
from dimos.robot.manipulators.common.blueprints import teleop_ik_task

R1PRO_QUEST_TASK_NAME = "teleop_r1pro"

_r1pro_model = make_r1pro_model_config()
_r1pro_hardware = HardwareComponent(
    hardware_id="r1pro",
    hardware_type=HardwareType.WHOLE_BODY,
    joints=list(R1PRO_UPPER_BODY_PLANNING_JOINTS),
    adapter_type="mock_whole_body",
)

coordinator_teleop_r1pro = autoconnect(
    TeleopControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_r1pro_hardware],
        tasks=[
            teleop_ik_task(
                _r1pro_hardware,
                name=R1PRO_QUEST_TASK_NAME,
                robot_model=_r1pro_model,
                joint_names=R1PRO_UPPER_BODY_PLANNING_JOINTS,
                bindings=[
                    {"hand": "left", "target_frame": "left_gripper_link"},
                    {"hand": "right", "target_frame": "right_gripper_link"},
                ],
                head_target_frame=R1ProPinkPoseTargetSolver.HEAD_FRAME,
                solver_type=R1ProPinkPoseTargetSolver,
                params={"pink": R1PRO_TELEOP_PINK},
            ),
            joint_trajectory_task(list(R1PRO_UPPER_BODY_PLANNING_JOINTS), priority=20),
        ],
    ),
    ManipulationModule.blueprint(
        model=_r1pro_model,
        # Reachable off-robot, like the sibling r1pro blueprint. The mock
        # adapter moves nothing physical, so viser is the only place the
        # operator can see this blueprint working at all.
        visualization=ViserVisualizationConfig(host=global_config.listen_host),
    ),
)
