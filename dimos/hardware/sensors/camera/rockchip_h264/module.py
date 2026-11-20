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

"""A front and a rear camera on a Rockchip RK3588, captured off V4L2 (MJPEG) and hardware-encoded to H.264, every frame, by the native ``rockchip_h264_cameras`` module (on the M20, the direct path that replaces the vendor RTSP server).

Each ``CompressedVideo`` is one H.264 Annex-B access unit, so a stream holds P-frames and decodes only
in order. The SPS/PPS go in front of every keyframe, so a late
subscriber starts decoding at the next keyframe, about a GOP away. A lost message corrupts the picture
until the next keyframe, so these topics keep zenoh's reliable delivery rather than ``QOS_LATEST_WINS``.

The Desktop gateway passes an H.264 source straight through to the browser (no decode/re-encode), so
encoding H.264 here frees the robot's big cores that the old H.265 transcode used.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Literal

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class RockchipH264CamerasConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/hardware/sensors/camera/rockchip_h264/rust"
    executable: str = "result/bin/rockchip_h264_cameras"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # V4L2 device nodes of the two cameras (MJPEG)
    front_device: str = "/dev/video0"
    rear_device: str = "/dev/video2"
    # capture + encode: the pipeline forces these caps and the Rockchip H.264 encoder's bitrate and GOP
    width: int = Field(default=1280, ge=1)
    height: int = Field(default=720, ge=1)
    fps: int = Field(default=30, ge=1, le=120)
    bitrate_bps: int = Field(default=4_000_000, ge=100_000, le=50_000_000)
    gop: int = Field(default=30, ge=1, le=600)
    # "rust" (default): in-process, the rockchip-h264 crate driving the RK3588's JPEG decoder and H.264 encoder directly.
    # "gstreamer" (fallback): a gst-launch child per camera (the vendor mpp plugin, which leaks a dmabuf fd per frame).
    encoder: Literal["rust", "gstreamer"] = "rust"
    # Camera centers in base_link, and their turn about z (the rear one looks back).
    front_camera_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    front_camera_yaw_rad: float = 0.0
    rear_camera_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rear_camera_yaw_rad: float = math.pi
    # The lenses' focal length at focal_length_width_px wide, scaled to each stream's width.
    focal_length_px: float = Field(default=607.0, gt=0.0)
    focal_length_width_px: int = Field(default=1280, ge=1)
    # The lenses' distortion as CameraInfo names it: "plumb_bob" (k1, k2, p1, p2, k3), "rational_polynomial"
    # (k1, k2, p1, p2, k3, k4, k5, k6) or "equidistant" (fisheye, k1..k4; also "fisheye" / "kannala_brandt"), at the
    # focal length above. Uncalibrated: the lenses are fisheyes, and a plumb-line fit to one rear
    # frame gives equidistant, focal 486 px at 1280 wide, k1..k4 = 0 (an estimate, not a calibration).
    distortion_model: Literal[
        "plumb_bob", "rational_polynomial", "equidistant", "fisheye", "kannala_brandt"
    ] = "plumb_bob"
    distortion_coefficients: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])
    # Undistort every frame on the robot before encoding (rust encoder only). CameraInfo then describes the
    # deskewed frames: their new pinhole matrix, no distortion, the same frame_id.
    deskew: bool = False
    # "pinhole" keeps straight lines straight; "cylindrical" / "equirectangular" keep vertical lines straight and fit
    # the fisheyes' whole width without the black wedges a pinhole needs.
    deskew_projection: Literal["pinhole", "cylindrical", "equirectangular"] = "pinhole"
    # Deskewed frame size; 0 keeps the capture's.
    deskew_width: int = Field(default=0, ge=0)
    deskew_height: int = Field(default=0, ge=0)
    # 0 keeps only what every edge of the frame saw (no black borders, crops the most), 1 the whole field of view
    # (40% black on the M20's fisheyes). 0.5 keeps most of the view with black only past the lens' own edge (16%).
    deskew_balance: float = Field(default=0.5, ge=0.0, le=1.0)
    deskew_interpolation: Literal["bilinear", "nearest"] = "bilinear"
    retry_s: float = Field(default=2.0, ge=0.1, le=60.0)
    # Without an answer or a frame for this long, the stream is dropped and reopened.
    stall_timeout_s: float = Field(default=2.0, ge=0.1, le=60.0)
    # Period of the static transforms and CameraInfo.
    metadata_period_s: float = Field(default=1.0, ge=0.01, le=60.0)
    # Period of the per-camera fps / keyframe / bitrate log line.
    stats_period_s: float = Field(default=10.0, ge=0.1, le=3600.0)


class RockchipH264Cameras(NativeModule):
    """Both cameras as H.264 ``CompressedVideo`` (direct V4L2 capture), every frame."""

    config: RockchipH264CamerasConfig

    front_camera: Out[CompressedVideo]
    rear_camera: Out[CompressedVideo]
    front_camera_info: Out[CameraInfo]
    rear_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    RockchipH264Cameras()
