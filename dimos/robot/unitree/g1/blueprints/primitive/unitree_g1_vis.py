#!/usr/bin/env python3
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

from __future__ import annotations

from typing import Any

import rerun as rr

from dimos.core.global_config import global_config
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.nav_stack.main import nav_stack_rerun_config
from dimos.robot.unitree.g1.g1_rerun import g1_costmap, g1_odometry_tf_override, g1_static_robot
from dimos.visualization.vis_module import vis_module

_PATH_Z_LIFT = 0.3
_PATH_COLOR_RGBA = (0, 255, 128, 255)
_PATH_RADIUS_METERS = 0.05


def _g1_path_colors(path: Path) -> Any:
    # Empty geometry instead of None so the stale path actually clears.
    if not path.poses:
        return rr.LineStrips3D([])

    points = [[pose.x, pose.y, pose.z + _PATH_Z_LIFT] for pose in path.poses]
    return rr.LineStrips3D([points], colors=[_PATH_COLOR_RGBA], radii=_PATH_RADIUS_METERS)


unitree_g1_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=nav_stack_rerun_config(
        {
            "visual_override": {
                "world/odometry": g1_odometry_tf_override,
                "world/lidar": None,
                "world/local_map": None,
                "world/global_map_fastlio": None,
                "world/global_costmap": g1_costmap,
                "world/path": _g1_path_colors,
            },
            "static": {"world/tf/robot": g1_static_robot},
            "memory_limit": "1GB",
        },
        vis_throttle=0.5,
    ),
)

__all__ = ["unitree_g1_vis"]
