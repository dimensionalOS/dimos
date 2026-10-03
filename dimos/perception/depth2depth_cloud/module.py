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

"""Native colour camera + lidar -> dense camera-frame PointCloud2 (Depth2DepthCloud) module."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from pydantic import Field, field_validator

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import get_data


class Depth2DepthCloudConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "depth2depth_cloud")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    # frame_id is also a NativeModuleConfig field; listed so it still crosses to the Rust config.
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # Depth Anything V2 small, metric (Hypersim), as the depth2depth crate's two safetensors files; empty fetches
    # the bundled ones (184 MB) when the module starts, not whenever a config is built.
    weights_dir: str = ""
    # On a Jetson the model runs through TensorRT from this ONNX export (fixed 364x448 input); the engine is
    # built on first run (minutes) and cached per machine.
    onnx_file: str = "da2_metric_hypersim_vits_364x448.onnx"
    engine_cache_dir: str = str(Path.home() / ".cache" / "dimos" / "depth2depth_cloud")
    # Model input for candle (Mac, CPU); multiples of 14, smaller is faster.
    model_height: int = 364
    model_width: int = 448
    # JPEG decoded at 1/decode_scale of full size, then resampled to this pinhole image for the model;
    # 0 takes the decoded size and the CameraInfo's focal length at that scale.
    decode_scale: int = Field(default=2, ge=1, le=8)
    undistorted_width: int = Field(default=0, ge=0, le=4096)
    undistorted_height: int = Field(default=0, ge=0, le=4096)
    undistorted_focal_px: float = Field(default=0.0, ge=0.0, le=10000.0)
    # Scans are kept this long in world_frame, so ground the lidar saw a moment ago still anchors the frame.
    world_frame: str = "odom"
    lidar_history_s: float = Field(default=2.0, ge=0.0, le=30.0)
    max_anchor_range_m: float = Field(default=12.0, ge=0.1, le=200.0)
    tf_tolerance_s: float = Field(default=0.1, ge=0.0, le=5.0)
    # Frames and scans wait this long for their transform before they are dropped.
    max_tf_lag_s: float = Field(default=0.5, ge=0.0, le=2.0)
    # Calibration (depth2depth::CalibrationConfig): edge-aware spread near lidar, a smooth fit away from it.
    sigma_px: float = Field(default=40.0, ge=1.0, le=1000.0)
    sigma_log_depth: float = Field(default=0.15, ge=0.001, le=10.0)
    neighbours: int = Field(default=16, ge=1, le=256)
    grid_step: int = Field(default=6, ge=1, le=64)
    reach: float = Field(default=1.0, ge=0.01, le=100.0)
    shape_ema: float = Field(default=0.1, ge=0.0, le=1.0)
    min_anchors: int = Field(default=100, ge=1, le=1000000)
    # Leave out pixels leaning on nearby lidar less than this (0..1); 0 keeps every pixel.
    min_support: float = Field(default=0.0, ge=0.0, le=1.0)
    # Cloud: range crop, then one pixel per decimation x decimation block, then a point budget (0 = none).
    min_range_m: float = Field(default=0.3, ge=0.0, le=1000.0)
    max_range_m: float = Field(default=6.0, ge=0.0, le=1000.0)
    decimation: int = Field(default=4, ge=1, le=64)
    max_points: int = Field(default=0, ge=0, le=10000000)
    frame_id: str = ""
    # Keep only points whose height in height_frame (e.g. base_link) is within [min, max]; "" keeps all.
    height_frame: str = ""
    min_height_m: float = -1000.0
    max_height_m: float = 1000.0

    @field_validator("model_height", "model_width")
    @classmethod
    def _patch_multiple(cls, size: int) -> int:
        if size % 14 or not 56 <= size <= 1036:
            raise ValueError(
                f"{size} must be a multiple of 14 in [56, 1036], the model's patch grid"
            )
        return size

    @field_validator("decode_scale")
    @classmethod
    def _jpeg_scale(cls, scale: int) -> int:
        if scale not in (1, 2, 4, 8):
            raise ValueError(f"decode_scale={scale}: JPEG decodes scale by 1, 2, 4 or 8 only")
        return scale


class Depth2DepthCloud(NativeModule):
    """Colour frames + lidar -> camera-frame PointCloud2: Depth Anything calibrated per pixel to the recent lidar."""

    config: Depth2DepthCloudConfig

    image: In[CompressedImage]
    camera_info: In[CameraInfo]
    lidar: In[PointCloud2]
    tf: In[TFMessage]
    depth_cloud: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        if not self.config.weights_dir:
            self.config.weights_dir = str(get_data("depth2depth_vits_hypersim"))
        super().start()


if TYPE_CHECKING:
    Depth2DepthCloud()
