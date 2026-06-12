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

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("dimos_mls_planner")

from dimos.memory2.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.mls_planner.transformer import MLSPlan


def _obs(
    points: np.ndarray,
    pose: tuple[float, float, float],
    region_bounds: tuple[float, float, float, float, float],
) -> Observation[PointCloud2]:
    return Observation(
        id=0,
        ts=0.0,
        pose=pose,
        tags={"region_bounds": region_bounds},
        _data=PointCloud2.from_numpy(points),
    )


def test_start_z_is_dropped_by_robot_height() -> None:
    obs = _obs(
        np.zeros((1, 3), dtype=np.float32),
        pose=(1.0, 2.0, 3.0),
        region_bounds=(1.0, 2.0, 5.0, -1.0, 5.0),
    )

    [out] = list(MLSPlan(goal=(10.0, 10.0, 0.0), robot_height=0.4)(iter([obs])))

    assert out.tags["start"] == (1.0, 2.0, 3.0 - 0.4)


def test_no_route_yields_empty_path_with_planned_false() -> None:
    # Random points form no traversable surface, so plan() returns None.
    rng = np.random.default_rng(0)
    obs = _obs(
        rng.random((50, 3)).astype(np.float32),
        pose=(0.0, 0.0, 0.0),
        region_bounds=(0.5, 0.5, 2.0, -1.0, 2.0),
    )

    [out] = list(MLSPlan(goal=(100.0, 100.0, 100.0))(iter([obs])))

    assert out.tags["planned"] is False
    assert out.data.poses == []
