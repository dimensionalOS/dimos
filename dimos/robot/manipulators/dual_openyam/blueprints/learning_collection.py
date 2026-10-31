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

from dimos.core.coordination.blueprints import autoconnect
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

dual_openyam_quest_collection = autoconnect(
    collect(profile=_profile),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    build_dual_openyam_webxr(visualization=ViserVisualizationConfig(host="0.0.0.0")),
    WebcamModule.blueprint(
        instance_name="left_wrist",
        hardware=WebcamConfig(camera_index=0, fps=30, frame_id_prefix="left_wrist_image"),
        frame_id="left_wrist_camera_link",
    ),
    WebcamModule.blueprint(
        instance_name="right_wrist",
        hardware=WebcamConfig(camera_index=1, fps=30, frame_id_prefix="right_wrist_image"),
        frame_id="right_wrist_camera_link",
    ),
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
