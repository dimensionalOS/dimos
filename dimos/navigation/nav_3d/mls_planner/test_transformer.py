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
from numpy.typing import NDArray
import pytest

pytest.importorskip("dimos_mls_planner")

from dimos.memory2.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.mls_planner.transformer import MLSPlan


def _obs(points: NDArray[np.float32], pose: tuple[float, float, float]) -> Observation[PointCloud2]:
    return Observation(id=0, ts=0.0, pose=pose, _data=PointCloud2.from_numpy(points))


def _flat_floor(half_extent: float = 3.0, spacing: float = 0.1) -> NDArray[np.float32]:
    coords = np.arange(-half_extent, half_extent, spacing, dtype=np.float32)
    xs, ys = np.meshgrid(coords, coords)
    zs = np.zeros_like(xs)
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)


def test_flat_floor_yields_populated_path_and_planned_true() -> None:
    obs = _obs(_flat_floor(), pose=(-2.0, -2.0, 1.0))

    [out] = list(MLSPlan(goal=(2.0, 2.0, 0.0), voxel_size=0.2, robot_height=1.0)(iter([obs])))

    assert out.tags["planned"] is True
    assert len(out.data.poses) >= 2
    assert out.tags["voxel_map"] is obs.data
    assert out.tags["nodes"].shape[1] == 3
    assert out.tags["surface_map"].shape[1] == 3


def test_no_route_yields_empty_path_with_planned_false() -> None:
    rng = np.random.default_rng(0)
    obs = _obs(rng.random((50, 3)).astype(np.float32), pose=(0.0, 0.0, 0.0))

    [out] = list(MLSPlan(goal=(100.0, 100.0, 100.0))(iter([obs])))

    assert out.tags["planned"] is False
    assert out.data.poses == []


def test_poseless_obs_is_skipped_and_following_posed_obs_plans() -> None:
    poseless = Observation(id=1, ts=0.0, pose=None, _data=PointCloud2.from_numpy(_flat_floor()))
    posed = _obs(_flat_floor(), pose=(-2.0, -2.0, 1.0))

    results = list(
        MLSPlan(goal=(2.0, 2.0, 0.0), voxel_size=0.2, robot_height=1.0)(iter([poseless, posed]))
    )

    assert len(results) == 1
    assert results[0].tags["planned"] is True
