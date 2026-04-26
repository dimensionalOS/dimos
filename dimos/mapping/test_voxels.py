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

from collections.abc import Callable, Generator
import time

import numpy as np
import pytest

from dimos.core.transport import LCMTransport
from dimos.mapping.voxels import VoxelGrid
from dimos.memory.timeseries.legacy import LegacyPickleStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data
from dimos.utils.testing.moment import OutputMoment
from dimos.utils.testing.test_moment import Go2Moment


@pytest.fixture
def grid() -> Generator[VoxelGrid, None, None]:
    g = VoxelGrid()
    yield g
    g.dispose()


class Go2MapperMoment(Go2Moment):
    global_map: OutputMoment[PointCloud2] = OutputMoment(LCMTransport("/global_map", PointCloud2))


MomentFactory = Callable[[float, bool], Go2MapperMoment]


@pytest.fixture
def moment() -> Generator[MomentFactory, None, None]:
    instances: list[Go2MapperMoment] = []

    def get_moment(ts: float, publish: bool = True) -> Go2MapperMoment:
        m = Go2MapperMoment()
        m.seek(ts)
        if publish:
            m.publish()
        instances.append(m)
        return m

    yield get_moment
    for m in instances:
        m.stop()


@pytest.fixture
def moment1(moment: MomentFactory) -> Go2MapperMoment:
    return moment(10, False)


@pytest.fixture
def moment2(moment: MomentFactory) -> Go2MapperMoment:
    return moment(85, False)


@pytest.mark.tool
def two_perspectives_loop(moment: MomentFactory) -> None:
    while True:
        moment(10, True)
        time.sleep(1)
        moment(85, True)
        time.sleep(1)


@pytest.mark.tool
def test_carving(grid: VoxelGrid, moment1: Go2MapperMoment, moment2: Go2MapperMoment) -> None:
    lidar_frame1 = moment1.lidar.value
    assert lidar_frame1 is not None

    lidar_frame2 = moment2.lidar.value
    assert lidar_frame2 is not None

    # Carving grid (default, carve_columns=True)
    grid.add_frame(lidar_frame1)
    grid.add_frame(lidar_frame2)
    count_carving = grid.size()

    voxel_size = grid._voxel_size
    pts1 = np.asarray(lidar_frame1.pointcloud.points)
    pts2 = np.asarray(lidar_frame2.pointcloud.points)
    combined_vox = np.floor(np.vstack([pts1, pts2]) / voxel_size).astype(np.int64)
    count_additive = np.unique(combined_vox, axis=0).shape[0]

    print("\n=== Carving comparison ===")
    print(f"Additive (no carving): {count_additive}")
    print(f"With carving: {count_carving}")
    print(f"Voxels carved: {count_additive - count_carving}")

    # Carving should result in fewer voxels
    assert count_carving < count_additive, (
        f"Carving should remove some voxels. Additive: {count_additive}, Carving: {count_carving}"
    )


def test_ingest_a_few(grid: VoxelGrid) -> None:
    data_dir = get_data("unitree_go2_office_walk2")
    lidar_store = LegacyPickleStore(f"{data_dir}/lidar")

    for i in [1, 4, 8]:
        frame = lidar_store.find_closest_seek(i)
        assert frame is not None
        print("add", frame)
        grid.add_frame(frame)

    assert len(grid.get_global_pointcloud2()) == 30136


@pytest.mark.parametrize(
    "voxel_size, expected_points",
    [
        (0.5, 277),
        (0.1, 7290),
        (0.05, 28199),
    ],
)
def test_roundtrip(moment1: Go2MapperMoment, voxel_size: float, expected_points: int) -> None:
    lidar_frame = moment1.lidar.value
    assert lidar_frame is not None

    grid = VoxelGrid(voxel_size=voxel_size)
    grid.add_frame(lidar_frame)

    global1 = grid.get_global_pointcloud2()
    assert len(global1) == expected_points

    # loseless roundtrip
    if voxel_size == 0.05:
        assert len(global1) == len(lidar_frame)
        # TODO: we want __eq__ on PointCloud2 - should actually compare
        # all points in both frames

    grid.add_frame(global1)
    # no new information, no global map change
    assert len(grid.get_global_pointcloud2()) == len(global1)

    moment1.publish()
    grid.dispose()


def test_roundtrip_range_preserved(grid: VoxelGrid) -> None:
    """Test that input coordinate ranges are preserved in output."""
    data_dir = get_data("unitree_go2_office_walk2")
    lidar_store = LegacyPickleStore(f"{data_dir}/lidar")

    frame = lidar_store.find_closest_seek(1.0)
    assert frame is not None
    input_pts = np.asarray(frame.pointcloud.points)

    grid.add_frame(frame)

    out_pcd = grid.get_global_pointcloud().to_legacy()
    out_pts = np.asarray(out_pcd.points)

    voxel_size = grid._voxel_size
    tolerance = voxel_size  # Allow one voxel of difference at boundaries

    # TODO: we want __eq__ on PointCloud2 - should actually compare
    # all points in both frames

    for axis, name in enumerate(["X", "Y", "Z"]):
        in_min, in_max = input_pts[:, axis].min(), input_pts[:, axis].max()
        out_min, out_max = out_pts[:, axis].min(), out_pts[:, axis].max()

        assert abs(in_min - out_min) < tolerance, f"{name} min mismatch: in={in_min}, out={out_min}"
        assert abs(in_max - out_max) < tolerance, f"{name} max mismatch: in={in_max}, out={out_max}"
