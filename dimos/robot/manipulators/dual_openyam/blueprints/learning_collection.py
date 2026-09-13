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
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import collection_recorder
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_COLLECTION

dual_openyam_quest_collection = autoconnect(
    teleop_webxr_dual_openyam,
    CameraModule.blueprint(
        instance_name="left_wrist",
        hardware=WebcamConfig(
            camera_index=0, width=640, height=480, fps=30, frame_id_prefix="left_wrist_image"
        ),
        frame_id="left_wrist_camera_link",
    ),
    CameraModule.blueprint(
        instance_name="right_wrist",
        hardware=WebcamConfig(
            camera_index=1, width=640, height=480, fps=30, frame_id_prefix="right_wrist_image"
        ),
        frame_id="right_wrist_camera_link",
    ),
    collection_recorder(profile=DUAL_OPENYAM_COLLECTION),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
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
