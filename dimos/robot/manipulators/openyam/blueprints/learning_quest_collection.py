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

"""Quest-guided OpenYAM collection with explicit robot and camera composition."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import collection_recorder
from dimos.robot.manipulators.openyam.blueprints.teleop import teleop_webxr_openyam
from dimos.robot.manipulators.openyam.collection import OPENYAM_QUEST_COLLECTION

openyam_quest_collection = autoconnect(
    teleop_webxr_openyam,
    CameraModule.blueprint(
        instance_name="wrist",
        hardware=WebcamConfig(
            camera_index=0, width=640, height=480, fps=30, frame_id_prefix="wrist_image"
        ),
        frame_id="wrist_camera_link",
    ),
    collection_recorder(profile=OPENYAM_QUEST_COLLECTION),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
).remappings(
    [
        ("wrist", "color_image", "wrist_image"),
        ("wrist", "camera_info", "wrist_camera_info"),
        ("wrist", "tf", "wrist_tf"),
    ]
)
