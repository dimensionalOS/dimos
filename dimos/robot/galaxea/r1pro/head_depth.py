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

"""The R1 Pro's head depth: Depth Anything on the left head camera, calibrated to the chassis lidar."""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint
from dimos.perception.depth2depth_cloud.module import Depth2DepthCloud
from dimos.robot.galaxea.r1pro.lio import R1ProLioConfig


def r1pro_head_depth(tf_tolerance_s: float, **overrides: object) -> Blueprint:
    """Head depth anchored on Point-LIO's scans, tuned for the Orin; keywords override the Depth2DepthCloud config."""
    return Depth2DepthCloud.blueprint(
        **{
            "tf_tolerance_s": tf_tolerance_s,
            # Past this the per-pixel calibration has few lidar anchors to lean on.
            "max_range_m": 6.0,
            # The chassis lidar never sees the floor near the robot; floor it swept a few seconds ago anchors it.
            "lidar_history_s": 6.0,
            # Kept only from the floor to just above the lidar's plane, what the lidar sees worst; dropping (not
            # snapping) anything under the floor, since nothing can be there.
            "height_frame": R1ProLioConfig.model_fields["base_frame"].default,
            "min_height_m": -0.15,
            "max_height_m": 0.35,
            # The ray tracer shares the Orin with everything else; one point per 11x11 pixels, at most 1500, keeps it
            # near one Point-LIO's worth of work with every lidar ray traced.
            "decimation": 11,
            "max_points": 1500,
            # JPEG decoded at 1/4 size: calibration then works on a quarter of the pixels (3 -> 9.5 fps on the Orin).
            "decode_scale": 4,
            # One lidar anchor per 4x4 px block: ~31k anchors per frame was most of the calibration time.
            "anchor_step": 4,
            **overrides,
        }
    ).remappings(
        [
            (Depth2DepthCloud, "image", "head_left_color"),
            (Depth2DepthCloud, "camera_info", "head_left_info"),
            (Depth2DepthCloud, "depth_cloud", "head_cloud"),
        ]
    )
