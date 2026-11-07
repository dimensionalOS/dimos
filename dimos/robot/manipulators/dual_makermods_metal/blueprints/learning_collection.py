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

"""Quest-guided bimanual Metal collection: two wrist cameras, measured and commanded joints."""

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_makermods_metal.blueprints.teleop import (
    DUAL_METAL_QUEST_JOINTS,
    teleop_webxr_dual_metal,
)


def _wrist_image(side: str) -> FeatureSpec:
    return FeatureSpec(
        stream=f"{side}_wrist_image",
        message_type=Image,
        field="data",
        dtype="video",
        shape=(480, 640, 3),
        names=["height", "width", "channels"],
    )


_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(DUAL_METAL_QUEST_JOINTS),),
    names=list(DUAL_METAL_QUEST_JOINTS),
)
DUAL_METAL_QUEST_PROFILE = CollectionProfile(
    observations={
        "observation.images.left_wrist": _wrist_image("left"),
        "observation.images.right_wrist": _wrist_image("right"),
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
    metadata={"name": "dual-metal-quest", "robot_type": "dual_makermods_metal"},
)


def _wrist_camera(side: str, camera_index: int) -> Blueprint:
    return WebcamModule.blueprint(
        instance_name=f"{side}_wrist",
        hardware=WebcamConfig(
            camera_index=camera_index,
            width=640,
            height=480,
            fps=30,
            frame_id_prefix=f"{side}_wrist_image",
        ),
        frame_id=f"{side}_wrist_camera_link",
    )


# The recorder comes first so it stops after the producers it records.
dual_metal_quest_collection = autoconnect(
    collect(profile=DUAL_METAL_QUEST_PROFILE),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    teleop_webxr_dual_metal,
    _wrist_camera("left", 0),
    _wrist_camera("right", 1),
).remappings(
    [
        ("left_wrist", "color_image", "left_wrist_image"),
        ("left_wrist", "camera_info", "left_wrist_camera_info"),
        ("left_wrist", "tf", "left_wrist_tf"),
        ("right_wrist", "color_image", "right_wrist_image"),
        ("right_wrist", "camera_info", "right_wrist_camera_info"),
        ("right_wrist", "tf", "right_wrist_tf"),
    ]
)
