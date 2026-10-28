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
from scipy.spatial.transform import Rotation

o3d = pytest.importorskip("open3d")

from dimos.mapping.relocalization.go2 import relocalize as strategy


def _room() -> np.ndarray:
    """Walls, floor and two pillars, off the voxel grid like real surfaces."""
    xs, ys, zs = (np.arange(0, n + 1e-9, 0.05) for n in (10.0, 6.0, 2.5))
    parts = []
    for x in (0.0, 10.0):
        y, z = np.meshgrid(ys, zs)
        parts.append(np.stack([np.full(y.size, x), y.ravel(), z.ravel()], 1))
    for y in (0.0, 6.0):
        x, z = np.meshgrid(xs, zs)
        parts.append(np.stack([x.ravel(), np.full(x.size, y), z.ravel()], 1))
    x, y = np.meshgrid(xs, ys)
    parts.append(np.stack([x.ravel(), y.ravel(), np.zeros(x.size)], 1))
    for px, py in ((3.0, 1.5), (7.0, 4.0)):
        z, a = np.meshgrid(zs, np.linspace(0, 2 * np.pi, 40))
        parts.append(
            np.stack([px + 0.3 * np.cos(a.ravel()), py + 0.3 * np.sin(a.ravel()), z.ravel()], 1)
        )
    return np.concatenate(parts) + 0.0123


def _cloud(pts: np.ndarray):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def _pose(x: float, y: float, yaw_deg: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[:2, 3] = x, y
    return m


@pytest.fixture
def quick(monkeypatch: pytest.MonkeyPatch) -> None:
    # the same strategy on a smaller budget, so it runs in seconds
    monkeypatch.setattr(strategy, "SCALE_PLAN", [(0.2, 3), (0.3, 3), (0.8, 1)])
    monkeypatch.setattr(strategy, "RANSAC_ITERS", 50_000)


def test_relocalize_places_a_view_of_the_room_back_where_it_was_taken(quick: None) -> None:
    room = _room()
    target = strategy.prepare(_cloud(room))
    # what the robot sees from where it stands, in a frame turned and moved away from the map's
    view = room[np.linalg.norm(room[:, :2] - [6.0, 3.0], axis=1) < 6.0]
    seen_from = _pose(2.0, -1.5, 35.0)
    local = view @ np.linalg.inv(seen_from)[:3, :3].T + np.linalg.inv(seen_from)[:3, 3]

    placement, fitness = strategy.relocalize(target, _cloud(local))

    np.testing.assert_allclose(placement[:3, 3], seen_from[:3, 3], atol=0.05)
    angle = Rotation.from_matrix(placement[:3, :3] @ seen_from[:3, :3].T).magnitude()
    assert np.degrees(angle) < 1.0
    assert fitness > 0.8
    assert strategy.wall_fitness(target, _cloud(local), placement) > 0.8
    assert strategy.wall_fitness(target, _cloud(local), _pose(-1.0, 2.0, 120.0)) < 0.3
