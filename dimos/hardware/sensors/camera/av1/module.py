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

"""A V4L2 camera published as AV1 ``CompressedVideo``, by the native ``av1_camera`` module.

AV1 is the one video codec every current browser decodes without a plugin, so a stream published this way
reaches a page through the gateway with no transcode. Nothing in this path is hardware: the RK3588 and every
other chip we run on can decode AV1 but none of them encode it, so the frames are encoded with SVT-AV1 on the
CPU, which measures about one Cortex-A76 core per 720p30 camera at ``preset`` 12-13. Capture is MJPEG (what USB
UVC cameras deliver) decoded with libjpeg-turbo, or a packed 4:2:2 format converted directly.

The encoder runs in low-delay mode, where each frame predicts only from the one before it: that is what keeps
the added latency near a frame time instead of the ~100 ms a lookahead costs. The cost is that there are no
B-frames to spend bits on, so at a fixed bitrate the picture is slightly worse than a lookahead encode would be.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo


class Av1CameraConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/hardware/sensors/camera/av1/rust"
    executable: str = "result/bin/av1_camera"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # /dev/videoN, or better a /dev/v4l/by-path link that survives the nodes being renumbered.
    device: str = "/dev/video0"
    # SVT-AV1 refuses anything under 64 on either side.
    width: int = Field(default=1280, ge=64, le=16384)
    height: int = Field(default=720, ge=64, le=16384)
    # V4L2 pixel format to request: MJPG, or a packed 4:2:2 format (UYVY, YUYV, VYUY, YVYU).
    fourcc: str = Field(default="MJPG", min_length=4, max_length=4)
    frame_id: str = "camera_optical"
    # The rate the device is asked for. It is also what the encoder's bitrate is spread over, so a camera that
    # delivers fewer frames than this spends fewer bits, not more per frame.
    fps: int = Field(default=30, ge=1, le=240)
    bitrate_bps: int = Field(default=4_000_000, ge=100_000, le=100_000_000)
    # Frames between keyframes. A keyframe is far larger than a predicted frame, so at a fixed bitrate a short
    # interval spends a visible share of each second on one frame and arrives as a burst; a long one risks a
    # lost frame corrupting the picture for up to gop/fps seconds, and nothing here answers a decoder's request
    # for a fresh keyframe.
    # 1 would make every frame a keyframe, which SVT-AV1 will not encode at a fixed bitrate.
    gop: int = Field(default=60, ge=2, le=600)
    # SVT-AV1 speed preset: 0 is slowest and best, 13 fastest. Only 11-13 reach 30 fps on an A76 core.
    preset: int = Field(default=12, ge=0, le=13)
    # Cores the encoder may use. Low-delay prediction leaves no frame-level parallelism, so more than one buys
    # very little (measured on the M20: 38.6 fps on one core, 37.7 on four).
    threads: int = Field(default=1, ge=1, le=64)
    retry_s: float = Field(default=3.0, ge=0.1, le=60.0)


class Av1Camera(NativeModule):
    """Publish a V4L2 camera as AV1 ``CompressedVideo`` stamped with the driver's capture time."""

    config: Av1CameraConfig

    video: Out[CompressedVideo]


if TYPE_CHECKING:
    Av1Camera()
