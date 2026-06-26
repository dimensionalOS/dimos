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

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.dannav.rotation_clearance import can_rotate_in_place


def _pose_at(x: float, y: float) -> PoseStamped:
    return PoseStamped(frame_id="map", position=[x, y, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])


def test_can_rotate_in_place_on_open_map() -> None:
    grid = np.zeros((80, 80), dtype=np.int8)
    costmap = OccupancyGrid(
        grid=grid,
        resolution=0.05,
        origin=Pose(-2.0, -2.0, 0.0),
        frame_id="map",
        ts=1.0,
    )

    assert can_rotate_in_place(costmap, _pose_at(0.0, 0.0), rotation_radius_m=0.3)


def test_can_rotate_in_place_rejects_narrow_corridor() -> None:
    grid = np.zeros((80, 80), dtype=np.int8)
    # Corridor width 0.30 m (robot can translate, 0.6 m rotation disk cannot fit).
    for row in range(grid.shape[0]):
        world_y = -2.0 + row * 0.05
        if abs(world_y) > 0.15:
            grid[row, :] = CostValues.OCCUPIED

    costmap = OccupancyGrid(
        grid=grid,
        resolution=0.05,
        origin=Pose(-2.0, -2.0, 0.0),
        frame_id="map",
        ts=1.0,
    )

    assert not can_rotate_in_place(costmap, _pose_at(0.0, 0.0), rotation_radius_m=0.3)


def test_can_rotate_in_place_rejects_unknown_cells() -> None:
    grid = np.zeros((40, 40), dtype=np.int8)
    grid[20, 21] = CostValues.UNKNOWN
    costmap = OccupancyGrid(
        grid=grid,
        resolution=0.05,
        origin=Pose(-1.0, -1.0, 0.0),
        frame_id="map",
        ts=1.0,
    )

    assert not can_rotate_in_place(costmap, _pose_at(0.0, 0.0), rotation_radius_m=0.1)
