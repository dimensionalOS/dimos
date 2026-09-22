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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
import numpy as np
import open3d.core as o3c
import pytest

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.mapping.voxels.impl.o3d import O3dVoxels
from dimos.mapping.voxels.impl.packed import PackedVoxels
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz


@pytest.mark.parametrize("backend", ["packed", "open3d"])
@pytest.mark.parametrize("carve", [False, True])
def test_generated_cloud_union_and_column_carving(backend, carve):
    grid = (
        PackedVoxels(0.5, carve)
        if backend == "packed"
        else O3dVoxels(0.5, 1000, carve, o3c.Device("CPU:0"))
    )
    try:
        for points in [[[0.1, 0.1, 0.1], [0.1, 0.1, 1.1], [1.1, 1.1, 0.1]], [[0.1, 0.1, 0.6]]]:
            grid.add_frame(pointcloud_from_xyz(np.array(points), header=Header(frame_id="map")))
        actual = {tuple(point) for point in grid.points()}
        expected = {(0.25, 0.25, 0.75), (1.25, 1.25, 0.25)}
        if not carve:
            expected |= {(0.25, 0.25, 0.25), (0.25, 0.25, 1.25)}
        assert actual == expected
        grid.add_frame(
            pointcloud_from_xyz(np.array([[np.nan, 0, 0], [0, np.inf, 0]]), header=Header())
        )
        assert {tuple(point) for point in grid.points()} == expected
    finally:
        grid.dispose()


def test_voxel_output_retains_exact_latest_stamp_and_invalidates_cache():
    grid = VoxelGrid(voxel_size=0.5, device="CPU:0", frame_id="map", show_startup_log=False)
    try:
        empty = grid.get_global_pointcloud2()
        assert empty.width == 0 and empty.header.stamp == Time()
        header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
        source = pointcloud_from_xyz(np.array([[0.1, 0.2, 0.3]]), header=header)
        grid.add_frame(source)
        source.header.stamp.nanosec = 1
        first = grid.get_global_pointcloud2()
        assert first.header.stamp == header.stamp
        np.testing.assert_array_equal(pointcloud_xyz(first), [[0.25, 0.25, 0.25]])
        assert grid.get_global_pointcloud2() is first
        header.stamp.nanosec += 1
        grid.add_frame(pointcloud_from_xyz(np.array([[1.1, 1.1, 1.1]]), header=header))
        second = grid.get_global_pointcloud2()
        assert second is not first
        assert second.header == header and second.width == 2
        assert first.width == 1
        grid.add_frame(pointcloud_from_xyz(np.empty((0, 3)), header=Header(frame_id="map")))
        assert grid.get_global_pointcloud2().header.stamp == Time()
        assert grid.get_global_pointcloud2().width == 2
    finally:
        grid.dispose()
    with pytest.raises(RuntimeError, match="disposed"):
        grid.get_global_pointcloud2()
