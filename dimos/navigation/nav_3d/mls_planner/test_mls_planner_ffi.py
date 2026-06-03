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

"""Smoke tests for the pyo3-bound MLSPlanner."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner


def _floor_cloud(half_extent: float = 2.0, step: float = 0.05) -> np.ndarray:
    xs = np.arange(-half_extent, half_extent + step, step, dtype=np.float32)
    ys = np.arange(-half_extent, half_extent + step, step, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    zz = np.zeros_like(xx)
    return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)


def test_round_trip_builds_graph_and_plans() -> None:
    planner = MLSPlanner(voxel_size=0.1, robot_height=0.5)
    planner.update_global_map(_floor_cloud())

    assert planner.surface_map().shape[1] == 3
    assert planner.nodes().shape[1] == 3
    assert planner.node_edges().shape[1] == 7

    path = planner.plan((-1.5, -1.5, 0.1), (1.5, 1.5, 0.1))
    assert path is not None
    assert path.dtype == np.float32
    assert path.shape[1] == 3
    assert path.shape[0] >= 2


def test_update_global_map_rejects_wrong_shape() -> None:
    planner = MLSPlanner(voxel_size=0.1, robot_height=0.5)
    bad = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    with pytest.raises(ValueError, match="N, 3"):
        planner.update_global_map(bad)
