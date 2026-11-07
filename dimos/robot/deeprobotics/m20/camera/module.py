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

"""The M20's two cameras captured off V4L2 (MJPEG) and hardware-encoded to H.264, every frame, by the native ``m20_camera`` module — the direct path that replaces the vendor RTSP server.

Each ``CompressedVideo`` is one H.264 Annex-B access unit, so a stream holds P-frames and decodes only
in order. ``h264parse config-interval=-1`` puts the SPS/PPS in front of every keyframe, so a late
subscriber starts decoding at the next keyframe, about a GOP away. A lost message corrupts the picture
until the next keyframe, so these topics keep zenoh's reliable delivery rather than ``QOS_LATEST_WINS``.

The Desktop gateway passes an H.264 source straight through to the browser (no decode/re-encode), so
encoding H.264 here frees the robot's big cores that the old H.265 transcode used.
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


class M20CameraRelayConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/camera/rust"
    executable: str = "result/bin/m20_camera"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # V4L2 device nodes of the two cameras on the vendor AOS board (MJPEG up to 1080p30)
    front_device: str = "/dev/video0"
    rear_device: str = "/dev/video2"
    # capture + encode: the pipeline forces these caps and the Rockchip H.264 encoder's bitrate and GOP
    width: int = Field(default=1280, ge=1)
    height: int = Field(default=720, ge=1)
    fps: int = Field(default=30, ge=1, le=120)
    bitrate_bps: int = Field(default=4_000_000, ge=100_000, le=50_000_000)
    gop: int = Field(default=30, ge=1, le=600)
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
    # Period of the per-camera fps / keyframe / bitrate log line.
    stats_period_s: float = Field(default=10.0, ge=0.1, le=3600.0)


class M20CameraRelay(NativeModule):
    """Relay both M20 cameras as H.264 ``CompressedVideo`` (direct V4L2 capture), every frame."""

    config: M20CameraRelayConfig

    front_camera: Out[CompressedVideo]
    rear_camera: Out[CompressedVideo]
    front_camera_info: Out[CameraInfo]
    rear_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    M20CameraRelay()
