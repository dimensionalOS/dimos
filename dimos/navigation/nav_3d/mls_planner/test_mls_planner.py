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

pytest.importorskip("dimos_mls_planner")

from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner


def make_planner() -> MLSPlanner:
    return MLSPlanner(voxel_size=0.2, robot_height=1.0)


def flat_floor(half_extent: float = 3.0, spacing: float = 0.1) -> np.ndarray:
    coords = np.arange(-half_extent, half_extent, spacing, dtype=np.float32)
    xs, ys = np.meshgrid(coords, coords)
    zs = np.zeros_like(xs)
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)


def test_update_global_map_builds_surfaces_and_nodes() -> None:
    planner = make_planner()
    planner.update_global_map(flat_floor())

    surface = planner.surface_map()
    assert surface.shape[1] == 3
    assert surface.dtype == np.float32
    assert len(surface) > 0

    nodes = planner.nodes()
    assert nodes.shape[1] == 3
    assert len(nodes) > 0

    edges = planner.node_edges()
    assert edges.shape[1] == 7
    assert len(edges) > 0


def test_plan_returns_path_on_flat_floor() -> None:
    planner = make_planner()
    planner.update_global_map(flat_floor())

    path = planner.plan((-2.0, -2.0, 0.0), (2.0, 2.0, 0.0))
    assert path is not None
    assert path.shape[1] == 3
    assert path.dtype == np.float32
    assert len(path) >= 2
    np.testing.assert_allclose(path[0][:2], [-2.0, -2.0], atol=0.5)
    np.testing.assert_allclose(path[-1][:2], [2.0, 2.0], atol=0.5)


def test_plan_before_update_returns_none() -> None:
    planner = make_planner()
    assert planner.plan((0.0, 0.0, 0.0), (1.0, 1.0, 0.0)) is None


def test_plan_to_unreachable_goal_returns_none() -> None:
    planner = make_planner()
    planner.update_global_map(flat_floor())

    assert planner.plan((0.0, 0.0, 0.0), (100.0, 100.0, 0.0)) is None


def test_wrong_shape_is_rejected() -> None:
    planner = make_planner()
    with pytest.raises(ValueError):
        planner.update_global_map(np.zeros((4, 2), dtype=np.float32))


def test_clear_drops_graph() -> None:
    planner = make_planner()
    planner.update_global_map(flat_floor())
    assert planner.plan((-2.0, -2.0, 0.0), (2.0, 2.0, 0.0)) is not None

    planner.clear()
    assert len(planner.nodes()) == 0
    assert planner.plan((-2.0, -2.0, 0.0), (2.0, 2.0, 0.0)) is None
