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

from dataclasses import replace

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.transport import pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.policy.module import policy_module
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    dual_openyam_webxr_tasks,
    teleop_webxr_dual_openyam,
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
    tasks = (
        dual_openyam_webxr_tasks()
        if quest_control
        else [joint_trajectory_task(DUAL_OPENYAM_JOINTS)]
    )
    # Policy chunks include grippers as well as arm joints.
    tasks = [
        replace(task, joint_names=list(DUAL_OPENYAM_JOINTS)) if task.type == "trajectory" else task
        for task in tasks
    ]
    coordinator = DualOpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=tasks,
    )
    robot = autoconnect(teleop_webxr_dual_openyam, coordinator) if quest_control else coordinator
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
