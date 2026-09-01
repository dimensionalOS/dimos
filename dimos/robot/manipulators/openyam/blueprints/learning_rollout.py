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

"""OpenYAM ACT rollout with Quest control and wrist observations."""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.imitation.policy.rollout_supervisor import (
    POLICY_GRIPPER_TASK_NAME,
    POLICY_ROLLOUT_TASK_NAME,
    PolicyRolloutSupervisor,
)
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    OPENYAM_QUEST_HARDWARE,
    OPENYAM_QUEST_KINEMATICS,
    OPENYAM_QUEST_MODEL,
    openyam_quest_tasks,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_GRIPPER_JOINT,
)
from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE
from dimos.teleop.quest.action_bindings import QuestActionBindingsModule
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

_policy_arm_task = TaskConfig(
    name=POLICY_ROLLOUT_TASK_NAME,
    type="trajectory",
    joint_names=list(OPENYAM_ARM_JOINTS),
    priority=10,
    params={"requires_activation": True},
    stream_bind={"joint_command": "policy_joint_command"},
)
_policy_gripper_task = TaskConfig(
    name=POLICY_GRIPPER_TASK_NAME,
    type="gripper",
    joint_names=[OPENYAM_GRIPPER_JOINT],
    priority=10,
    params={"hold_duration": 0.1, "requires_activation": True},
    stream_bind={"gripper_command": "policy_gripper_command"},
)

learning_rollout_quest_openyam = (
    autoconnect(
        LeRobotPolicyModule.blueprint(
            joint_names=list(OPENYAM_LEARNING_PROFILE.joint_names),
            gripper_joint_name=OPENYAM_LEARNING_PROFILE.gripper_joint_name,
            fps=OPENYAM_LEARNING_PROFILE.fps,
            robot_type=OPENYAM_LEARNING_PROFILE.robot_type,
        ),
        PolicyRolloutSupervisor.blueprint(),
        QuestActionBindingsModule.blueprint(),
        ArmTeleopModule.blueprint(),
        TeleopControlCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[OPENYAM_QUEST_HARDWARE],
            tasks=openyam_quest_tasks(_policy_arm_task, _policy_gripper_task),
        ),
        CameraModule.blueprint(
            instance_name="WristCamera",
            hardware=WebcamConfig(
                camera_index=0,
                width=OPENYAM_LEARNING_PROFILE.camera_width,
                height=OPENYAM_LEARNING_PROFILE.camera_height,
                fps=OPENYAM_LEARNING_PROFILE.fps,
                frame_id_prefix=OPENYAM_LEARNING_PROFILE.camera_frame_prefix,
            ),
            frame_id=OPENYAM_LEARNING_PROFILE.camera_frame_id,
        ),
        ManipulationModule.blueprint(
            model=OPENYAM_QUEST_MODEL,
            kinematics=OPENYAM_QUEST_KINEMATICS,
            visualization={"backend": "viser"},
        ),
    )
    .remappings(
        [
            (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
            (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
            (LeRobotPolicyModule, "joint_command", "policy_joint_command"),
            (LeRobotPolicyModule, "gripper_command", "policy_gripper_command"),
            (QuestActionBindingsModule, "primary_action", "rollout_toggle"),
        ]
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image",
                default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE,
            )
        }
    )
)
