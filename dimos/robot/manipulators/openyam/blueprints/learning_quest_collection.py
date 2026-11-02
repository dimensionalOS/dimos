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

"""Quest-guided OpenYAM collection."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.blueprints.teleop import teleop_webxr_openyam
from dimos.robot.manipulators.openyam.joints import OPENYAM_JOINTS

_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(OPENYAM_JOINTS),),
    names=list(OPENYAM_JOINTS),
)
_profile = CollectionProfile(
    observations={
        "observation.images.wrist": FeatureSpec(
            stream="wrist_image",
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
    metadata={"name": "openyam-quest", "robot_type": "openyam"},
)

openyam_quest_collection = autoconnect(
    collect(profile=_profile),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    teleop_webxr_openyam,
    WebcamModule.blueprint(
        instance_name="wrist",
        hardware=WebcamConfig(camera_index=0, fps=30, frame_id_prefix="wrist_image"),
        frame_id="wrist_camera_link",
    ),
).remappings(
    [
        ("wrist", "color_image", "wrist_image"),
        ("wrist", "camera_info", "wrist_camera_info"),
        ("wrist", "tf", "wrist_tf"),
    ]
)
