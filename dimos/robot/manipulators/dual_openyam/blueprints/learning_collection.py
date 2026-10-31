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

"""Dual OpenYAM collection using ordinary blueprint configuration."""

from copy import deepcopy

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import build_dual_openyam_webxr
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(DUAL_OPENYAM_JOINTS),),
    names=list(DUAL_OPENYAM_JOINTS),
)
_profile = CollectionProfile(
    observations={
        "observation.images.left_wrist": FeatureSpec(
            stream="left_wrist_image",
            message_type=Image,
            field="data",
            dtype="video",
            shape=(480, 640, 3),
            names=["height", "width", "channels"],
        ),
        "observation.images.right_wrist": FeatureSpec(
            stream="right_wrist_image",
            message_type=Image,
            field="data",
            dtype="video",
            shape=(480, 640, 3),
            names=["height", "width", "channels"],
        ),
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
    metadata={"name": "dual-openyam-quest", "robot_type": "dual_openyam"},
)


def build_dual_openyam_collection(*, overhead: bool = False) -> Blueprint:
    """Collect wrist views, optionally adding the Dreamlabs overhead view."""
    profile = deepcopy(_profile)
    cameras = [("left_wrist", 0), ("right_wrist", 1)]
    if overhead:
        cameras.append(("overhead", 2))
        profile.observations["observation.images.overhead"] = FeatureSpec(
            stream="overhead_image",
            message_type=Image,
            field="data",
            dtype="video",
            shape=(480, 640, 3),
            names=["height", "width", "channels"],
        )
        profile.metadata["name"] = "dual-openyam-dreamlabs"
    return autoconnect(
        collect(profile=profile),
        EpisodeMonitorModule.blueprint(instance_name="episodes"),
        build_dual_openyam_webxr(visualization=ViserVisualizationConfig(host="0.0.0.0")),
        *[
            WebcamModule.blueprint(
                instance_name=name,
                hardware=WebcamConfig(camera_index=index, fps=30, frame_id_prefix=f"{name}_image"),
                frame_id=f"{name}_camera_link",
            )
            for name, index in cameras
        ],
    ).remappings(
        [
            (name, output, f"{name}_{suffix}")
            for name, _ in cameras
            for output, suffix in (
                ("color_image", "image"),
                ("camera_info", "camera_info"),
                ("tf", "tf"),
            )
        ]
    )


dual_openyam_quest_collection = autoconnect(build_dual_openyam_collection())
dual_openyam_dreamlabs_collection = autoconnect(build_dual_openyam_collection(overhead=True))
