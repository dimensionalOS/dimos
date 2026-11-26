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

"""Record dual OpenYAM demonstrations driven by the YAM leader arms."""

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.blueprints.leader_teleop import (
    dual_openyam_leader_teleop,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

DUAL_OPENYAM_CAMERAS = ("overhead", "left_wrist", "right_wrist")
# The connection publishes every joint under its robot name, "arm".
DUAL_OPENYAM_COLLECTION_JOINTS = [f"arm/{name}" for name in DUAL_OPENYAM_JOINTS]


def dual_openyam_collection_camera(name: str) -> Blueprint:
    """A D405 color stream named after its place on the rig; the serial comes from the run config."""
    if name not in DUAL_OPENYAM_CAMERAS:
        raise ValueError(f"unknown dual OpenYAM camera {name!r}")
    return RealSenseCamera.blueprint(
        instance_name=f"{name}_camera",
        frame_id_prefix=name,
        width=640,
        fps=30,
        enable_depth=False,
        align_depth_to_color=False,
    ).remappings(
        [
            (f"{name}_camera", stream, f"{name}_{stream}")
            for stream in ("color_image", "camera_info", "tf")
        ]
    )


def dual_openyam_leader_collection_profile() -> CollectionProfile:
    state = FeatureSpec(
        stream="coordinator_joint_state",
        message_type=JointState,
        field="position",
        dtype="float32",
        shape=(len(DUAL_OPENYAM_COLLECTION_JOINTS),),
        names=DUAL_OPENYAM_COLLECTION_JOINTS,
    )
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
            "observation.state": state,
        },
        actions={
            "action": state.model_copy(
                update={
                    # The coordinator's command port to the connection; it does
                    # not publish coordinator_joint_command for connection robots.
                    "stream": "arm_position_command",
                    "source_kind": "joint_position_updates",
                },
                deep=True,
            )
        },
        metadata={"name": "dual-openyam-leader", "robot_type": "dual_openyam"},
    )


# The leaders have no buttons; episodes are marked with dimos imitation control.
learning_collect_leader_dual_openyam = autoconnect(
    collect(profile=dual_openyam_leader_collection_profile()),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    dual_openyam_leader_teleop,
    *[dual_openyam_collection_camera(name) for name in DUAL_OPENYAM_CAMERAS],
)
