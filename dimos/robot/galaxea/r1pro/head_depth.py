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

"""The R1 Pro's head depth: Depth Anything on the head cameras, calibrated to the chassis lidar, with the floor prior.

bin/native-modules only opens files with a line like the next one, so this says it for R1ProHeadDepthConfig:
class R1ProHeadDepthConfig(NativeModuleConfig)
"""

from __future__ import annotations

from pydantic import Field

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.depth2depth_cloud.module import Depth2DepthCloudConfig
from dimos.robot.galaxea.r1pro.config import R1PRO_CHASSIS_WIDTH_M
from dimos.robot.galaxea.r1pro.lio import R1ProLioConfig

BASE_FRAME = R1ProLioConfig.model_fields["base_frame"].default


class R1ProHeadDepthConfig(Depth2DepthCloudConfig, NativeModuleConfig):
    source_dir: str | None = "dimos/robot/galaxea/r1pro/floor_prior/rust"
    executable: str = "result/bin/r1pro_head_depth"
    build_command: str | None = "nix build -L path:."
    # Which head cameras get depth; each one runs its own model.
    head_left: bool = True
    head_right: bool = False
    # Its origin is where the wheels touch the floor, so the floor is its z = 0 however the torso and head move.
    height_frame: str = BASE_FRAME
    # Just under the floor to just above the chassis lidar's plane, what the lidar sees worst; deeper is a drop, kept out.
    min_height_m: float = -0.15
    max_height_m: float = 0.35
    # Floor prior: probe one pixel per floor_anchor_step block (8: one per cloud pixel at decimation 8), anchor those
    # from min_height_m to floor_tolerance_m about the floor, in floor_anchor_rounds rounds.
    floor_anchor_step: int = Field(default=8, ge=1, le=256)
    floor_tolerance_m: float = Field(default=0.05, ge=0.0, le=10.0)
    floor_anchor_rounds: int = Field(default=3, ge=1, le=16)
    # A cloud point from min_height_m to this about the floor is slid along its ray onto it.
    floor_snap_tolerance_m: float = Field(default=0.05, ge=0.0, le=10.0)
    # The floor under the robot, which it stands on and no sensor sees, as points this far apart; 0 radius adds none.
    footprint_radius_m: float = Field(default=0.0, ge=0.0, le=10.0)
    footprint_spacing_m: float = Field(default=0.05, ge=0.001, le=10.0)
    # Unseen floor past the footprint, per bearing out to its nearest point within view_fill_reach_m if that is floor.
    view_fill_reach_m: float = Field(default=0.0, ge=0.0, le=100.0)
    view_fill_bin_deg: float = Field(default=5.0, ge=0.1, le=180.0)


class R1ProHeadDepth(NativeModule):
    """Depth2DepthCloud per enabled head camera, with the floor prior: the chassis lidar never sees the near floor, which unanchored reads 2-5x short."""

    config: R1ProHeadDepthConfig

    head_left_color: In[CompressedImage]
    head_left_info: In[CameraInfo]
    head_right_color: In[CompressedImage]
    head_right_info: In[CameraInfo]
    lidar: In[PointCloud2]
    tf: In[TFMessage]
    # Every enabled camera's clouds, each in its own camera's frame, with the floor snapped and filled in.
    head_cloud: Out[PointCloud2]


def r1pro_head_depth(tf_tolerance_s: float, **overrides: object) -> Blueprint:
    """Orin-tuned head depth anchored on Point-LIO; keywords override R1ProHeadDepth's config."""
    return R1ProHeadDepth.blueprint(
        **{
            "tf_tolerance_s": tf_tolerance_s,
            # Past this the per-pixel calibration has few lidar anchors to lean on.
            "max_range_m": 6.0,
            # lidar_history_s stays at d2d's 2 s (20 sweeps): at 6 s a person who walked off stayed in the anchors, a 6 s ghost.
            # 8x8 / 3000 over 11x11 / 1500 raised near floor coverage from ~45% to 51% on r1_live_20260930_1332.
            "decimation": 8,
            "max_points": 3000,
            # JPEG decoded at 1/4 size: calibration then works on a quarter of the pixels (3 -> 9.5 fps on the Orin).
            "decode_scale": 4,
            # One lidar anchor per 4x4 px block: ~31k anchors per frame was most of the calibration time.
            "anchor_step": 4,
            "footprint_radius_m": R1PRO_CHASSIS_WIDTH_M / 2,
            # The head's lowest ray meets the floor ~0.74 m out (farther with the head raised); unfilled, the start is an island.
            "view_fill_reach_m": 1.5,
            **overrides,
        }
    )
