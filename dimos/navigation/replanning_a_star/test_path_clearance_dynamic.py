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

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star import path_clearance as path_clearance_module
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlannerConfig
from dimos.navigation.replanning_a_star.path_clearance import (
    PathClearance,
    obstacle_lookahead_distance_m,
)


@pytest.mark.parametrize(
    ("speed_m_s", "expected_m"),
    [
        (0.0, 2.0),
        (0.5, 2.0),
        (1.0, 3.0),
        (-1.0, 3.0),
        (3.0, 6.0),
    ],
)
def test_obstacle_lookahead_uses_minimum_horizon_and_cap(
    speed_m_s: float, expected_m: float
) -> None:
    assert obstacle_lookahead_distance_m(
        speed_m_s,
        min_distance_m=2.0,
        time_horizon_s=3.0,
        max_distance_m=6.0,
    ) == pytest.approx(expected_m)


def test_obstacle_lookahead_config_rejects_inverted_bounds() -> None:
    with pytest.raises(ValueError, match="obstacle_lookahead_max_distance_m"):
        ReplanningAStarPlannerConfig(
            obstacle_lookahead_min_distance_m=3.0,
            obstacle_lookahead_max_distance_m=2.0,
        )


def test_speed_change_rebuilds_path_mask_with_new_lookahead(monkeypatch) -> None:
    used_distances: list[float] = []

    def fake_make_path_mask(*args, max_length: float, **kwargs):
        used_distances.append(max_length)
        return np.zeros((20, 20), dtype=np.bool_)

    monkeypatch.setattr(path_clearance_module, "make_path_mask", fake_make_path_mask)
    path = Path(
        poses=[
            PoseStamped(position=[0.0, 0.0, 0.0]),
            PoseStamped(position=[5.0, 0.0, 0.0]),
        ]
    )
    clearance = PathClearance(
        GlobalConfig(robot_width=0.9),
        path,
        min_lookup_distance_m=2.0,
        lookup_time_horizon_s=3.0,
        max_lookup_distance_m=6.0,
    )
    clearance.update_costmap(
        OccupancyGrid(
            grid=np.zeros((20, 20), dtype=np.int8),
            resolution=0.5,
            origin=Pose(position=[-1.0, -1.0, 0.0]),
        )
    )

    _ = clearance.mask
    clearance.update_command_speed(1.0)
    _ = clearance.mask

    assert used_distances == [2.0, 3.0]
