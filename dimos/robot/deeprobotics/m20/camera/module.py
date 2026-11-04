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

"""The M20's two vendor RTSP cameras relayed as H.265, every frame, by the native ``m20_camera`` module.

Each ``CompressedVideo`` is one Annex-B access unit, so a stream holds P-frames and decodes only in
order. Keyframes carry their VPS/SPS/PPS in band (the vendor sends them only in the SDP): a late
subscriber starts decoding at the next keyframe, about a second away. A lost message corrupts the
picture until the next keyframe, so these topics keep zenoh's reliable delivery rather than
``QOS_LATEST_WINS``.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.deeprobotics.m20.constants import (
    FRONT_CAMERA_RTSP_URL,
    REAR_CAMERA_RTSP_URL,
)


class M20CameraRelayConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/camera/rust"
    executable: str = "result/bin/m20_camera"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    front_rtsp_url: str = FRONT_CAMERA_RTSP_URL
    rear_rtsp_url: str = REAR_CAMERA_RTSP_URL
    # Camera centers in base_link from the Deep Robotics docs, and their turn about z.
    front_camera_xyz: tuple[float, float, float] = (0.37646, 0.0, 0.03738)
    front_camera_yaw_rad: float = 0.0
    rear_camera_xyz: tuple[float, float, float] = (-0.37646, 0.0, 0.03738)
    rear_camera_yaw_rad: float = math.pi
    # The provisional M20/Whale focal length at 1280 px wide, scaled to each stream's width. The
    # vendor publishes no factory calibration.
    focal_length_px: float = Field(default=607.0, gt=0.0)
    focal_length_width_px: int = Field(default=1280, ge=1)
    retry_s: float = Field(default=2.0, ge=0.1, le=60.0)
    # Without an answer or a frame for this long, the stream is dropped and reopened.
    stall_timeout_s: float = Field(default=2.0, ge=0.1, le=60.0)
    # Period of the static transforms and CameraInfo.
    metadata_period_s: float = Field(default=1.0, ge=0.01, le=60.0)


class M20CameraRelay(NativeModule):
    """Relay both vendor RTSP cameras as H.265 ``CompressedVideo``, every frame."""

    config: M20CameraRelayConfig

    front_camera: Out[CompressedVideo]
    rear_camera: Out[CompressedVideo]
    front_camera_info: Out[CameraInfo]
    rear_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    M20CameraRelay()
