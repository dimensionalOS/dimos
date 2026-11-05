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

"""Shared pieces of dual OpenYAM data collection: teleop pacing, cameras, the profile."""

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    build_dual_openyam_webxr,
    dual_openyam_webxr_tasks,
)
from dimos.robot.manipulators.dual_openyam.collection import DualOpenYamCollectionTeleopModule
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_JOINTS,
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)

DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S = 1.2
DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S = 0.6
# B and Y belong to the episode monitor; A sends both arms to the zero pose.
DUAL_OPENYAM_COLLECTION_ZERO_BUTTON = "A"
DUAL_OPENYAM_CAMERAS = ("overhead", "left_wrist", "right_wrist")


def dual_openyam_collection_teleop_tasks() -> list[TaskConfig]:
    """The stock per-arm teleop tasks at the collection speed ceilings."""
    return dual_openyam_webxr_tasks(
        max_joint_velocity_rad_s=DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S,
        joint_velocity_limits_rad_s={
            DUAL_OPENYAM_LEFT_ARM_JOINTS[0]: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
            DUAL_OPENYAM_RIGHT_ARM_JOINTS[0]: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
        },
    )


teleop_webxr_dual_openyam_collection = autoconnect(
    build_dual_openyam_webxr(
        teleop_tasks=dual_openyam_collection_teleop_tasks(),
        teleop_module=DualOpenYamCollectionTeleopModule,
        zero_button=DUAL_OPENYAM_COLLECTION_ZERO_BUTTON,
    )
)


def dual_openyam_collection_camera(name: str) -> Blueprint:
    """A D405 whose streams carry a ``{name}_`` prefix; colour only, the D405s share USB 2 hubs."""
    if name not in DUAL_OPENYAM_CAMERAS:
        raise ValueError(f"camera must be one of {DUAL_OPENYAM_CAMERAS}, got {name!r}")
    instance = f"{name}_camera"
    camera = RealSenseCamera.blueprint(
        instance_name=instance,
        frame_id_prefix=name,
        width=640,
        fps=30,
        enable_depth=False,
        align_depth_to_color=False,
    )
    streams = [stream.name for stream in camera.blueprints[0].streams]
    return camera.remappings([(instance, stream, f"{name}_{stream}") for stream in streams])


def dual_openyam_collection_cameras() -> tuple[Blueprint, ...]:
    return tuple(dual_openyam_collection_camera(name) for name in DUAL_OPENYAM_CAMERAS)


_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(DUAL_OPENYAM_JOINTS),),
    names=list(DUAL_OPENYAM_JOINTS),
)


def dual_openyam_collection_profile(name: str) -> CollectionProfile:
    """Three colour images, the 14 measured joints, and the coordinator's joint targets."""
    return CollectionProfile(
        observations={
            **{
                f"observation.images.{camera}": FeatureSpec(
                    stream=f"{camera}_color_image",
                    message_type=Image,
                    field="data",
                    dtype="video",
                    shape=(480, 640, 3),
                    names=["height", "width", "channels"],
                )
                for camera in DUAL_OPENYAM_CAMERAS
            },
            "observation.state": _state,
        },
        actions={
            "action": _state.model_copy(
                update={
                    "stream": "coordinator_joint_command",
                    "source_kind": "joint_position_updates",
                },
                deep=True,
            )
        },
        metadata={"name": name, "robot_type": "dual_openyam"},
    )
