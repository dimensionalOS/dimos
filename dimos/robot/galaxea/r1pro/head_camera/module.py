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

"""The R1 Pro's two head cameras captured off V4L2 (raw UYVY) and hardware-encoded to H.264 with the Jetson's NVENC,
every frame, by the native ``r1_head_camera`` module — the R1 twin of the M20 direct relay.

Each ``CompressedVideo`` is one H.264 Annex-B access unit; ``h264parse config-interval=-1`` puts the SPS/PPS in front
of every keyframe, so a late subscriber starts decoding at the next keyframe. The Desktop gateway passes an H.264
source straight through to the browser (no decode/re-encode), so the Controller matches the native capture rate.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.robot.galaxea.r1pro.head_cameras import (
    HEAD_FPS,
    HEAD_HEIGHT,
    HEAD_LEFT_FRAME,
    HEAD_LEFT_V4L2,
    HEAD_RIGHT_FRAME,
    HEAD_RIGHT_V4L2,
    HEAD_WIDTH,
)


class R1HeadCameraRelayConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/galaxea/r1pro/head_camera/rust"
    executable: str = "result/bin/r1_head_camera"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    left_device: str = HEAD_LEFT_V4L2
    right_device: str = HEAD_RIGHT_V4L2
    left_frame_id: str = HEAD_LEFT_FRAME
    right_frame_id: str = HEAD_RIGHT_FRAME
    width: int = Field(default=HEAD_WIDTH, ge=1)
    height: int = Field(default=HEAD_HEIGHT, ge=1)
    fps: int = Field(default=int(HEAD_FPS), ge=1, le=120)
    # 1920x1536 H.264 needs more than 720p; NVENC CBR.
    bitrate_bps: int = Field(default=8_000_000, ge=100_000, le=50_000_000)
    gop: int = Field(default=30, ge=1, le=600)
    retry_s: float = Field(default=2.0, ge=0.1, le=60.0)
    stall_timeout_s: float = Field(default=2.0, ge=0.1, le=60.0)
    stats_period_s: float = Field(default=10.0, ge=0.1, le=3600.0)


class R1HeadCameraRelay(NativeModule):
    """Relay both R1 Pro head cameras as H.264 ``CompressedVideo`` (direct V4L2 capture + NVENC), every frame."""

    config: R1HeadCameraRelayConfig

    head_left_camera: Out[CompressedVideo]
    head_right_camera: Out[CompressedVideo]


if TYPE_CHECKING:
    R1HeadCameraRelay()
