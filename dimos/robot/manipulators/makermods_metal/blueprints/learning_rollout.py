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

"""MakerMods Metal policy-rollout builder with optional Quest takeover."""

from __future__ import annotations

from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.robot.manipulators.makermods_metal.blueprints.teleop import (
    METAL_QUEST_JOINTS,
    METAL_QUEST_KINEMATICS,
    METAL_QUEST_MODEL,
    metal_quest_tasks,
    metal_teleop_control,
)
from dimos.teleop.webxr.extensions import ArmTeleopModule

_WRIST_WIDTH = 640
_WRIST_HEIGHT = 480
_WRIST_FPS = 30.0


def build_metal_rollout(
    *,
    checkpoint: str | None = None,
    task: str | None = None,
    camera_device: int | str = 0,
    device: str | None = None,
    quest_control: bool = False,
) -> Blueprint:
    """Build the stack that lets a trained LeRobot policy drive one Metal arm.

    Args:
        checkpoint: Pretrained-model directory; ``--policy.policy-path`` when None.
        task: The instruction the policy was trained on; ``--policy.task`` when None.
        camera_device: Wrist webcam index or ``/dev/video*`` path.
        device: Torch device for the policy, e.g. "cuda"; None lets the runtime choose.
        quest_control: Also run the Quest teleop, which takes the arm over at a higher priority.
    """
    policy = LeRobotPolicyModule.blueprint(
        instance_name="policy",
        **({"policy_path": checkpoint} if checkpoint is not None else {}),
        **({"task": task} if task is not None else {}),
        device=device,
        joint_names=list(METAL_QUEST_JOINTS),
        fps=_WRIST_FPS,
        robot_type="makermods_metal",
        image_width=_WRIST_WIDTH,
        image_height=_WRIST_HEIGHT,
    )
    camera = WebcamModule.blueprint(
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
        return autoconnect(
            policy,
            ArmTeleopModule.blueprint(),
            metal_teleop_control(metal_quest_tasks()),
            camera,
            ManipulationModule.blueprint(
                model=METAL_QUEST_MODEL,
                gripper_tasks={"arm": "metal_gripper"},
                kinematics=METAL_QUEST_KINEMATICS,
                visualization={"backend": "viser"},
            ),
        ).remappings(
            [
                (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
                (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
            ]
        )
    return autoconnect(
        policy,
        metal_teleop_control([joint_trajectory_task(METAL_QUEST_JOINTS)], DamiaoCoordinator),
        camera,
    )


metal_lerobot_rollout = autoconnect(build_metal_rollout())
metal_lerobot_quest_rollout = autoconnect(build_metal_rollout(quest_control=True))
