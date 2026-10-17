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

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

VOXEL = 0.05


def _wall(x, ts):
    """A 1 m x 1 m wall of voxel centers in the plane at `x`."""
    ys, zs = np.meshgrid(np.arange(0.025, 1, VOXEL), np.arange(0.025, 1, VOXEL))
    pts = np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1)
    return PointCloud2.from_numpy(pts, timestamp=ts)


def _voxels(grid):
    return {tuple(v) for v in np.floor(grid.get_global_pointcloud2().points_f32() / VOXEL)}


def _drifted_walls():
    """The same wall seen twice, the first time 0.5 m off."""
    grid = VoxelGrid(stamped=True, show_startup_log=False)
    grid.add_frame(_wall(1.025, 1.0))
    grid.add_frame(_wall(1.525, 2.0))
    return grid


def _undrift(cloud):
    shift = np.where(cloud.stamps_f64()[:, None] == 1.0, [0.5, 0.0, 0.0], 0.0)
    return PointCloud2.from_numpy(cloud.points_f32() + shift)


def test_stamped_holds_the_same_voxels():
    plain = VoxelGrid(device="CPU:0", show_startup_log=False)
    stamped = VoxelGrid(stamped=True, show_startup_log=False)
    for cloud in (_wall(1.025, 1.0), _wall(2.025, 2.0), _wall(1.025, 3.0)):
        plain.add_frame(cloud)
        stamped.add_frame(cloud)
    assert _voxels(stamped) == _voxels(plain)


def test_identity_reproject_changes_nothing():
    grid = _drifted_walls()
    before = _voxels(grid)
    grid.reproject(lambda cloud: cloud)
    assert _voxels(grid) == before


def test_reproject_moves_voxels_by_their_frame():
    grid = _drifted_walls()
    assert len(grid) == 800  # double wall

    grid.reproject(_undrift)
    single = _voxels(grid)
    assert len(single) == 400
    assert {x for x, _, _ in single} == {30}  # both walls now at x = 1.5 m

    # re-placing starts from the original positions, so it does not compound
    grid.reproject(_undrift)
    assert _voxels(grid) == single


def test_reproject_needs_stamps():
    with pytest.raises(RuntimeError):
        VoxelGrid(device="CPU:0", show_startup_log=False).reproject(lambda cloud: cloud)
