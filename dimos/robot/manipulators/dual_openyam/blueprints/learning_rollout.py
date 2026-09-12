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

"""Dual-YAM policy rollout with three RGB views and optional Quest controls."""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.transport import pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.policy.module import POLICY_ROLLOUT_TASK_NAME, policy_module
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    dual_openyam_quest_tasks,
    teleop_quest_dual_openyam,
)
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_GRIPPER_JOINTS,
    DUAL_OPENYAM_JOINTS,
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)

ABC_JOINT_NAMES = [
    *DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS[0],
    *DUAL_OPENYAM_RIGHT_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS[1],
]


def build_dual_openyam_rollout(*, quest_control: bool = False) -> Blueprint:
    """Keep hardware wiring fixed when selecting a backend through module config."""
    policy_task = TaskConfig(
        name=POLICY_ROLLOUT_TASK_NAME,
        type="trajectory",
        priority=5,
        joint_names=list(DUAL_OPENYAM_JOINTS),
        params={"start_position_tolerance": 0.05},
    )
    coordinator = DualOpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=dual_openyam_quest_tasks(policy_task) if quest_control else [policy_task],
    )
    robot = autoconnect(teleop_quest_dual_openyam, coordinator) if quest_control else coordinator
    mapping = {"left_wrist_image": "left", "right_wrist_image": "right", "overhead_image": "top"}
    blueprint = autoconnect(
        robot,
        policy_module(
            instance_name="policy",
            backend="abc",
            image_mapping=mapping,
            joint_names=list(DUAL_OPENYAM_JOINTS),
            policy_joint_names=ABC_JOINT_NAMES,
            task="throw plastic bottles in bin",
            robot_type="dual_openyam",
        ),
        *[
            RealSenseCamera.blueprint(
                instance_name=f"{side}_wrist",
                width=640,
                height=480,
                fps=30,
                frame_id=f"{side}_wrist_camera_link",
                enable_depth=False,
                align_depth_to_color=False,
                enable_pointcloud=False,
            )
            for side in ("left", "right")
        ],
        CameraModule.blueprint(
            instance_name="overhead",
            hardware=WebcamConfig(camera_index=0, width=640, height=480, fps=30),
            frame_id="overhead_camera_link",
        ),
    ).remappings(
        [
            (camera, port, f"{camera}_{'image' if port == 'color_image' else port}")
            for camera in ("left_wrist", "right_wrist", "overhead")
            for port in ("color_image", "camera_info", "tf")
        ]
    )
    return blueprint.transports(
        {
            (port, Image): pSHMTransport.spec(
                f"/{port}", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            )
            for port in mapping
        }
    )


dual_openyam_policy_rollout = autoconnect(build_dual_openyam_rollout())
dual_openyam_policy_quest_rollout = autoconnect(build_dual_openyam_rollout(quest_control=True))
