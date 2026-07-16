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

"""Native Booster SDK RGB-D camera bridge."""

from __future__ import annotations

import os

from pydantic import Field

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.spec import perception

_BUILD_COMMAND = "nix build .#booster-camera-native"
_DEFAULT_COLOR_TOPIC = "rt/booster_video_stream"
_DEFAULT_DEPTH_TOPIC = "rt/boostercamera/head/depth"
_DEFAULT_COLOR_CAMERA_INFO_TOPIC = "rt/boostercamera/head/rgb/camera_info"
_DEFAULT_DEPTH_CAMERA_INFO_TOPIC = "rt/boostercamera/head/depth/camera_info"


class BoosterCameraConfig(NativeModuleConfig):
    """Configuration for Booster's SDK camera catalog and DDS streams."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/booster_camera_native"
    build_command: str | None = _BUILD_COMMAND
    log_format: LogFormat = LogFormat.TEXT
    network_interface: str | None = Field(default_factory=lambda: os.environ.get("ROBOT_INTERFACE"))
    depth_scale: float = Field(default=0.001, gt=0.0)
    image_reliable: bool = True
    color_compressed: bool = True
    color_topic: str = _DEFAULT_COLOR_TOPIC
    depth_topic: str = _DEFAULT_DEPTH_TOPIC
    color_camera_info_topic: str = _DEFAULT_COLOR_CAMERA_INFO_TOPIC
    depth_camera_info_topic: str = _DEFAULT_DEPTH_CAMERA_INFO_TOPIC


class BoosterCamera(NativeModule, perception.DepthCamera):
    """RGB-D camera streamed through the Booster C++ SDK."""

    config: BoosterCameraConfig

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
