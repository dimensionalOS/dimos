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

"""Coupled Quest teleoperation for the complete Dual OpenYAM entity."""

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.blueprints import teleop_ik_task
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS,
    dual_openyam_hardware,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.teleop_ik import (
    DualOpenYamPinkPoseTargetSolver,
)
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

DUAL_OPENYAM_QUEST_TASK_NAME = "teleop_dual_openyam"

_dual_openyam_quest_pink = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)
_dual_openyam_quest_hardware = dual_openyam_hardware()
_dual_openyam_quest_model = dual_openyam_model_config()
_dual_openyam_quest_task = teleop_ik_task(
    _dual_openyam_quest_hardware,
    robot_model=_dual_openyam_quest_model,
    name=DUAL_OPENYAM_QUEST_TASK_NAME,
    joint_names=DUAL_OPENYAM_ARM_JOINTS,
    priority=10,
    solver_type=DualOpenYamPinkPoseTargetSolver,
    bindings=[
        {
            "hand": "left",
            "target_frame": "left_grasp_frame",
        },
        {
            "hand": "right",
            "target_frame": "right_grasp_frame",
        },
    ],
    params={
        "pink": _dual_openyam_quest_pink,
        "timeout": 0.5,
        "max_command_tracking_error_deg": 10.0,
        "max_joint_velocity_rad_s": 2.0,
        "joint_command_filter_cutoff_hz": 30.0,
    },
)

teleop_quest_dual_openyam = autoconnect(
    ArmTeleopModule.blueprint(),
    DualOpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[
            _dual_openyam_quest_task,
            TaskConfig(
                name="left_arm_gripper",
                type="gripper",
                joint_names=[DUAL_OPENYAM_GRIPPER_JOINTS[0]],
                priority=20,
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            TaskConfig(
                name="right_arm_gripper",
                type="gripper",
                joint_names=[DUAL_OPENYAM_GRIPPER_JOINTS[1]],
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
            dual_openyam_trajectory_task(priority=20),
        ],
    ),
    ManipulationModule.blueprint(
        model=_dual_openyam_quest_model,
        kinematics=_dual_openyam_quest_pink,
        visualization={"backend": "viser"},
    ),
).remappings(
    [
        (ArmTeleopModule, "left_controller_output", "left_cartesian_command"),
        (ArmTeleopModule, "left_gripper_command", "left_gripper_command"),
        (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
        (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
    ]
)
