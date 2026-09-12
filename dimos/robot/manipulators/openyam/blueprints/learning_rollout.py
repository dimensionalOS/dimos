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
from dimos.imitation.policy.module import (
    POLICY_ROLLOUT_TASK_NAME,
    PolicyModule,
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
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

_WRIST_WIDTH = 640
_WRIST_HEIGHT = 480
_WRIST_FPS = 30.0


def build_openyam_rollout(
    *,
    checkpoint: str | None = None,
    task: str | None = None,
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
    policy = PolicyModule.blueprint(
        instance_name="policy",
        **({"policy_path": checkpoint} if checkpoint is not None else {}),
        **({"task": task} if task is not None else {}),
        device=device,
        joint_names=list(OPENYAM_JOINTS),
        fps=_WRIST_FPS,
        robot_type="openyam",
        trajectory_task_name=POLICY_ROLLOUT_TASK_NAME,
    )
    camera = CameraModule.blueprint(
        instance_name="WristCamera",
        hardware=WebcamConfig(
            camera_index=camera_device,
            width=_WRIST_WIDTH,
            height=_WRIST_HEIGHT,
            fps=_WRIST_FPS,
            frame_id_prefix="wrist",
        ),
        frame_id="wrist_camera_link",
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


openyam_policy_rollout = autoconnect(build_openyam_rollout())
openyam_policy_quest_rollout = autoconnect(build_openyam_rollout(quest_control=True))
