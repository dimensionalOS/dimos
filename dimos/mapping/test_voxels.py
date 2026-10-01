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

"""Deterministic CDR replacement for the retired typed-pickle lidar fixture."""

from collections.abc import Generator

from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz


@pytest.fixture
def grid() -> Generator[VoxelGrid, None, None]:
    value = VoxelGrid(device="CPU:0", show_startup_log=False)
    try:
        yield value
    finally:
        value.dispose()


@pytest.fixture
def lidar_frame() -> PointCloud2:
    axis = np.arange(20) * 0.05 + 0.025
    points = np.stack(np.meshgrid(axis, axis, axis, indexing="ij"), axis=-1).reshape(-1, 3)
    message = pointcloud_from_xyz(points, header=Header(frame_id="world"))
    return PointCloud2.decode(message.encode())


def test_ingest_a_few(grid: VoxelGrid) -> None:
    for offset in (0.0, 1.0, 2.0):
        points = np.array([[offset + 0.025, 0.025, 0.025], [offset + 0.075, 0.025, 0.025]])
        frame = pointcloud_from_xyz(points, header=Header(frame_id="world"))
        grid.add_frame(PointCloud2.decode(frame.encode()))
    assert grid.get_global_pointcloud2().width == 6
    np.testing.assert_allclose(
        np.sort(pointcloud_xyz(grid.get_global_pointcloud2())[:, 0]),
        [0.025, 0.075, 1.025, 1.075, 2.025, 2.075],
        atol=1e-6,
    )


@pytest.mark.parametrize("voxel_size,expected_points", [(0.5, 8), (0.1, 1000), (0.05, 8000)])
def test_roundtrip(lidar_frame: PointCloud2, voxel_size: float, expected_points: int) -> None:
    grid = VoxelGrid(voxel_size=voxel_size, device="CPU:0", show_startup_log=False)
    try:
        grid.add_frame(lidar_frame)
        first = grid.get_global_pointcloud2()
        assert first.width == expected_points
        if voxel_size == 0.05:
            assert first.width == lidar_frame.width
            np.testing.assert_allclose(
                np.sort(pointcloud_xyz(first), axis=0),
                np.sort(pointcloud_xyz(lidar_frame), axis=0),
                atol=1e-6,
            )
        grid.add_frame(PointCloud2.decode(first.encode()))
        assert grid.get_global_pointcloud2().width == expected_points
        np.testing.assert_array_equal(
            pointcloud_xyz(grid.get_global_pointcloud2()), pointcloud_xyz(first)
        )
    finally:
        grid.dispose()


def test_roundtrip_range_preserved(grid: VoxelGrid, lidar_frame: PointCloud2) -> None:
    inputs = pointcloud_xyz(lidar_frame)
    grid.add_frame(lidar_frame)
    outputs = np.asarray(grid.get_global_pointcloud().to_legacy().points)
    for axis in range(3):
        assert abs(inputs[:, axis].min() - outputs[:, axis].min()) < grid._voxel_size
        assert abs(inputs[:, axis].max() - outputs[:, axis].max()) < grid._voxel_size
