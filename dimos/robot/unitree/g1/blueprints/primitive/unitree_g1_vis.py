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

import numpy as np
import rerun as rr

from dimos.core.global_config import global_config
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.nav_stack.main import nav_stack_rerun_config
from dimos.robot.unitree.g1.g1_rerun import g1_odometry_tf_override, g1_static_robot
from dimos.visualization.vis_module import vis_module

_COSTMAP_Z_LIFT = 0.02
_COSTMAP_MAX_TEXTURE_SIZE = 256

_PATH_Z_LIFT = 0.3
_PATH_COLOR_RGBA = (0, 255, 128, 255)
_PATH_RADIUS_METERS = 0.05

# make obstacles red so I can see them
_COLOR_UNKNOWN = (0, 0, 0, 0)
_COLOR_FREE = (72, 73, 129, 255)
_COLOR_OCCUPIED = (255, 140, 0, 255)
_COLOR_LETHAL = (220, 30, 30, 255)

_COST_TO_RGBA = np.empty((102, 4), dtype=np.uint8)
_COST_TO_RGBA[0] = _COLOR_UNKNOWN
_COST_TO_RGBA[1] = _COLOR_FREE
_COST_TO_RGBA[2:101] = _COLOR_OCCUPIED
_COST_TO_RGBA[101] = _COLOR_LETHAL


def _g1_global_costmap_colors(occupancy_grid: OccupancyGrid) -> Any:
    if occupancy_grid.grid.size == 0:
        return rr.Mesh3D(vertex_positions=[])

    grid = occupancy_grid.grid[::-1]
    if grid.shape[0] > _COSTMAP_MAX_TEXTURE_SIZE or grid.shape[1] > _COSTMAP_MAX_TEXTURE_SIZE:
        row_stride = max(1, grid.shape[0] // _COSTMAP_MAX_TEXTURE_SIZE)
        column_stride = max(1, grid.shape[1] // _COSTMAP_MAX_TEXTURE_SIZE)
        grid = grid[::row_stride, ::column_stride]

    rgba_texture = np.ascontiguousarray(_COST_TO_RGBA[np.clip(grid + 1, 0, 101)])

    origin_x = occupancy_grid.origin.position.x
    origin_y = occupancy_grid.origin.position.y
    width_meters = occupancy_grid.width * occupancy_grid.resolution
    height_meters = occupancy_grid.height * occupancy_grid.resolution

    vertices = np.array(
        [
            [origin_x, origin_y, _COSTMAP_Z_LIFT],
            [origin_x + width_meters, origin_y, _COSTMAP_Z_LIFT],
            [origin_x + width_meters, origin_y + height_meters, _COSTMAP_Z_LIFT],
            [origin_x, origin_y + height_meters, _COSTMAP_Z_LIFT],
        ],
        dtype=np.float32,
    )
    triangle_indices = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
    texture_coords = np.array(
        [[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
        dtype=np.float32,
    )

    return rr.Mesh3D(
        vertex_positions=vertices,
        triangle_indices=triangle_indices,
        vertex_texcoords=texture_coords,
        albedo_texture=rgba_texture,
    )


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
                "world/global_costmap": _g1_global_costmap_colors,
                "world/path": _g1_path_colors,
            },
            "static": {"world/tf/robot": g1_static_robot},
            "memory_limit": "1GB",
        },
        vis_throttle=0.5,
    ),
)

__all__ = ["unitree_g1_vis"]
