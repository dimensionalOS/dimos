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

"""OpenYAM policy-rollout builder with optional Quest takeover."""

from __future__ import annotations

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.transport import pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.policy.lerobot.module import (
    POLICY_ROLLOUT_TASK_NAME,
    LeRobotPolicyModule,
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
    OPENYAM_JOINTS,
    openyam_hardware,
)
from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE
from dimos.teleop.quest.quest_extensions import ArmTeleopModule


def build_openyam_rollout(
    *,
    checkpoint: str,
    task: str,
    camera_device: int | str = 0,
    device: str | None = None,
    quest_control: bool = False,
) -> Blueprint:
    """Build an OpenYAM rollout; Quest control is an optional takeover layer."""
    policy_task = TaskConfig(
        name=POLICY_ROLLOUT_TASK_NAME,
        joint_names=list(OPENYAM_JOINTS),
        priority=10,
    )
    policy = LeRobotPolicyModule.blueprint(
        policy_path=checkpoint,
        task=task,
        device=device,
        joint_names=list(OPENYAM_LEARNING_PROFILE.joint_names),
        fps=OPENYAM_LEARNING_PROFILE.fps,
        robot_type=OPENYAM_LEARNING_PROFILE.robot_type,
        image_width=OPENYAM_LEARNING_PROFILE.camera_width,
        image_height=OPENYAM_LEARNING_PROFILE.camera_height,
        trajectory_task_name=POLICY_ROLLOUT_TASK_NAME,
    )
    camera = CameraModule.blueprint(
        instance_name="WristCamera",
        hardware=WebcamConfig(
            camera_index=camera_device,
            width=OPENYAM_LEARNING_PROFILE.camera_width,
            height=OPENYAM_LEARNING_PROFILE.camera_height,
            fps=OPENYAM_LEARNING_PROFILE.fps,
            frame_id_prefix=OPENYAM_LEARNING_PROFILE.camera_frame_prefix,
        ),
        frame_id=OPENYAM_LEARNING_PROFILE.camera_frame_id,
    )

    if quest_control:
        blueprint = autoconnect(
            policy,
            ArmTeleopModule.blueprint(),
            TeleopControlCoordinator.blueprint(
                instance_name="ControlCoordinator",
                hardware=[OPENYAM_QUEST_HARDWARE],
                tasks=openyam_quest_tasks(policy_task),
            ),
            camera,
            ManipulationModule.blueprint(
                model=OPENYAM_QUEST_MODEL,
                kinematics=OPENYAM_QUEST_KINEMATICS,
                visualization={"backend": "viser"},
            ),
        ).remappings(
            [
                (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
                (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
            ]
        )
    else:
        blueprint = autoconnect(
            policy,
            ControlCoordinator.blueprint(
                instance_name="ControlCoordinator",
                hardware=[openyam_hardware()],
                tasks=[policy_task],
            ),
            camera,
        )

    return blueprint.transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image",
                default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE,
            )
        }
    )
