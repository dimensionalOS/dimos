# Copyright 2025-2026 Dimensional Inc.
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

"""A V4L2 camera published as JPEG, by the native ``v4l2_camera`` module.

Frames are stamped with the driver's capture time, converted exactly to wall-clock time. On a Jetson with the
Multimedia API they never touch the CPU: the VIC converts them and NVJPG encodes them. Anywhere else, or if that
path will not start, the raw frames are encoded with libjpeg-turbo on the CPU. The device is opened lazily and
reopened on any failure, so a camera held by another process logs and waits.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from pydantic import Field, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage


class V4L2CameraConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/hardware/sensors/camera/v4l2/rust"
    executable: str = "result/bin/v4l2_camera"
    # A Jetson builds path:.#jetson, the hardware JPEG path (see below).
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # /dev/videoN, or better a /dev/v4l/by-path link that survives the nodes being renumbered.
    device: str = "/dev/video0"
    width: int = Field(default=848, ge=1, le=16384)
    height: int = Field(default=480, ge=1, le=16384)
    # V4L2 pixel format to request; the CPU encoder takes packed 4:2:2 (UYVY, YUYV, VYUY, YVYU).
    fourcc: str = Field(default="YUYV", min_length=4, max_length=4)
    frame_id: str = "camera_optical"
    jpeg_quality: int = Field(default=90, ge=1, le=100)
    # Use the Jetson's hardware encoder when present; the CPU path covers everything else.
    hardware: bool = True
    retry_s: float = Field(default=3.0, ge=0.1, le=60.0)

    @model_validator(mode="after")
    def _jetson_build(self) -> V4L2CameraConfig:
        if self.build_command == "nix build -L path:." and Path("/etc/nv_tegra_release").exists():
            self.build_command = "nix build -L path:.#jetson"
        return self


class V4L2Camera(NativeModule):
    """Publish a V4L2 camera as JPEG ``CompressedImage`` frames stamped with the driver's capture time."""

    config: V4L2CameraConfig

    jpeg_out: Out[CompressedImage]


if TYPE_CHECKING:
    V4L2Camera()
