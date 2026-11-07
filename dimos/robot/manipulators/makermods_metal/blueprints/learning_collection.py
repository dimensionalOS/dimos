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

"""Quest-guided MakerMods Metal collection: wrist camera, measured and commanded joints."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.makermods_metal.blueprints.teleop import (
    METAL_QUEST_JOINTS,
    teleop_webxr_metal,
)

_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(METAL_QUEST_JOINTS),),
    names=list(METAL_QUEST_JOINTS),
)
METAL_QUEST_PROFILE = CollectionProfile(
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
    metadata={"name": "metal-quest", "robot_type": "makermods_metal"},
)

# The recorder comes first so it stops after the producers it records.
metal_quest_collection = autoconnect(
    collect(profile=METAL_QUEST_PROFILE),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    teleop_webxr_metal,
    WebcamModule.blueprint(
        instance_name="wrist",
        hardware=WebcamConfig(
            camera_index=0, width=640, height=480, fps=30, frame_id_prefix="wrist_image"
        ),
        frame_id="wrist_camera_link",
    ),
    # The recorder's wrist image is the webcam, which keeps its name.
).remappings([("recorder", "wrist_image", "color_image")])
