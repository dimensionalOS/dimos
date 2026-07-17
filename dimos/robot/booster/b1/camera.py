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

from pydantic import Field

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.booster.b1.camera_config import (
    DEFAULT_COLOR_CAMERA_INFO_TOPIC,
    DEFAULT_COLOR_TOPIC,
    DEFAULT_DEPTH_CAMERA_INFO_TOPIC,
    DEFAULT_DEPTH_SCALE,
    DEFAULT_DEPTH_TOPIC,
    camera_network_interface_from_env,
)
from dimos.spec import perception

_BUILD_COMMAND = "nix build .#booster-camera-native"


class BoosterCameraConfig(NativeModuleConfig):
    """Configuration for Booster's SDK camera catalog and DDS streams."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/booster_camera_native"
    build_command: str | None = _BUILD_COMMAND
    auto_build: bool = True
    log_format: LogFormat = LogFormat.TEXT
    network_interface: str | None = Field(default_factory=camera_network_interface_from_env)
    depth_scale: float = Field(default=DEFAULT_DEPTH_SCALE, gt=0.0)
    publish_rate_hz: float | None = Field(default=None, gt=0.0)
    image_reliable: bool = False
    color_compressed: bool = True
    depth_enabled: bool = False
    depth_compressed: bool = True
    color_topic: str = DEFAULT_COLOR_TOPIC
    depth_topic: str = DEFAULT_DEPTH_TOPIC
    color_camera_info_topic: str = DEFAULT_COLOR_CAMERA_INFO_TOPIC
    depth_camera_info_topic: str = DEFAULT_DEPTH_CAMERA_INFO_TOPIC


class BoosterCamera(NativeModule, perception.DepthCamera):
    """RGB-D camera streamed through the Booster C++ SDK."""

    config: BoosterCameraConfig

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
