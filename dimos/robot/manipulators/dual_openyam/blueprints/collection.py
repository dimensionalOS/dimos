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

"""What a dual OpenYAM WebXR collection runs: teleop pacing, cameras, recorded streams."""

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    build_dual_openyam_webxr,
    dual_openyam_webxr_tasks,
)
from dimos.robot.manipulators.dual_openyam.collection import DualOpenYamCollectionTeleopModule
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_SIDES
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)

DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S = 1.2
DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S = 0.6
# B and Y belong to the episode monitor; A sends both arms to the zero pose.
DUAL_OPENYAM_COLLECTION_ZERO_BUTTON = "A"

DUAL_OPENYAM_COLLECTION_STREAMS = [
    "color_image",
    "coordinator_joint_state",
    "coordinator_joint_command",
    "status",
    "left_wrist_color_image",
    "right_wrist_color_image",
    "left_cartesian_command",
    "right_cartesian_command",
    "left_gripper_command",
    "right_gripper_command",
    "teleop_buttons",
]


def dual_openyam_collection_teleop_tasks() -> list[TaskConfig]:
    """The stock per-arm teleop tasks at the collection speed ceilings."""
    return dual_openyam_webxr_tasks(
        max_joint_velocity_rad_s=DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S,
        joint_velocity_limits_rad_s={
            DUAL_OPENYAM_LEFT_ARM_JOINTS[0]: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
            DUAL_OPENYAM_RIGHT_ARM_JOINTS[0]: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
        },
    )


def dual_openyam_wrist_camera(side: str) -> Blueprint:
    """A wrist D405 with ``{side}_wrist_`` stream names; colour only, the D405s share USB 2 hubs."""
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    name = f"{side}_wrist_camera"
    camera = RealSenseCamera.blueprint(
        instance_name=name,
        frame_id_prefix=f"{side}_wrist",
        width=640,
        fps=30,
        enable_depth=False,
        align_depth_to_color=False,
    )
    streams = [stream.name for stream in camera.blueprints[0].streams]
    return camera.remappings([(name, stream, f"{side}_wrist_{stream}") for stream in streams])


teleop_webxr_dual_openyam_collection = autoconnect(
    build_dual_openyam_webxr(
        teleop_tasks=dual_openyam_collection_teleop_tasks(),
        teleop_module=DualOpenYamCollectionTeleopModule,
        zero_button=DUAL_OPENYAM_COLLECTION_ZERO_BUTTON,
    )
)
