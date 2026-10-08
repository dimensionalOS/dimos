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

# Past this the per-pixel calibration has few lidar anchors to lean on.
MAX_RANGE_M = 6.0

# The lidar mount tf comes at 5 Hz and Point-LIO's at 10 with jitter; 0.1 s dropped frames (as for the ray tracer).
TF_TOLERANCE_S = 0.25
# The chassis lidar never sees the floor near the robot; floor it swept a few seconds ago anchors it.
LIDAR_HISTORY_S = 6.0


def r1pro_head_depth(
    *,
    min_height_m: float | None = None,
    max_height_m: float | None = None,
    lidar_history_s: float = LIDAR_HISTORY_S,
    **cloud: object,
) -> Blueprint:
    """Head depth anchored on Point-LIO's scans; height bounds (in base_link) gate only the cloud.

    Other keywords are Depth2DepthCloud config (e.g. decimation, max_points).
    """
    options: dict[str, object] = dict(cloud)
    if min_height_m is not None or max_height_m is not None:
        options["height_frame"] = R1ProLioConfig.model_fields["base_frame"].default
    if min_height_m is not None:
        options["min_height_m"] = min_height_m
    if max_height_m is not None:
        options["max_height_m"] = max_height_m

    return Depth2DepthCloud.blueprint(
        max_range_m=MAX_RANGE_M,
        tf_tolerance_s=TF_TOLERANCE_S,
        lidar_history_s=lidar_history_s,
        **options,
    ).remappings(
        [
            (Depth2DepthCloud, "image", "head_left_color"),
            (Depth2DepthCloud, "camera_info", "head_left_info"),
            (Depth2DepthCloud, "lidar", "lidar"),
            (Depth2DepthCloud, "depth_cloud", "head_cloud"),
        ]
    )
