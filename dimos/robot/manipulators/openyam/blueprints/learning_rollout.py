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

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.policy.lerobot.module import (
    LeRobotPolicyModule,
)
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    DamiaoTeleopCoordinator,
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    OPENYAM_QUEST_KINEMATICS,
    OPENYAM_QUEST_MODEL,
    openyam_quest_tasks,
)
from dimos.robot.manipulators.openyam.config import make_openyam_model_config
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.joints import OPENYAM_GRIPPER_JOINT
from dimos.teleop.webxr.extensions import ArmTeleopModule

_WRIST_WIDTH = 640
_WRIST_HEIGHT = 480
_WRIST_FPS = 30.0
_MODEL = make_openyam_model_config(robot="arm")
OPENYAM_ROLLOUT_JOINTS = [*_MODEL.joint_names, OPENYAM_GRIPPER_JOINT]


def _rollout_control(
    tasks: list[TaskConfig],
    coordinator: type[ControlCoordinator] = DamiaoCoordinator,
) -> Blueprint:
    """A trajectory coordinator named ControlCoordinator over the connection."""
    return autoconnect(
        coordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(OpenYamConnection)],
            tasks=list(tasks),
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenYamConnection),
    ).remappings(arm_remaps(OpenYamConnection, coordinator))


def build_openyam_rollout(
    *,
    checkpoint: str | None = None,
    task: str | None = None,
    camera_device: int | str = 0,
    device: str | None = None,
    quest_control: bool = False,
) -> Blueprint:
    """Build an OpenYAM rollout; Quest control is an optional takeover layer."""
    policy = LeRobotPolicyModule.blueprint(
        instance_name="policy",
        **({"policy_path": checkpoint} if checkpoint is not None else {}),
        **({"task": task} if task is not None else {}),
        device=device,
        joint_names=list(OPENYAM_ROLLOUT_JOINTS),
        fps=_WRIST_FPS,
        robot_type="openyam",
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
        blueprint = autoconnect(
            policy,
            ArmTeleopModule.blueprint(),
            _rollout_control(openyam_quest_tasks(), DamiaoTeleopCoordinator),
            camera,
            ManipulationModule.blueprint(
                gripper_tasks={"manipulator": "arm_gripper"},
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
            _rollout_control([joint_trajectory_task(OPENYAM_ROLLOUT_JOINTS)]),
            camera,
        )

    return blueprint


openyam_lerobot_rollout = autoconnect(build_openyam_rollout())
openyam_lerobot_quest_rollout = autoconnect(build_openyam_rollout(quest_control=True))
