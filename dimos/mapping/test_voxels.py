# Copyright 2025 Dimensional Inc.
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

from collections.abc import Generator
import time

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import pytest

from dimos.core import LCMTransport, Transport
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.data import get_data
from dimos.utils.testing.moment import OutputMoment
from dimos.utils.testing.replay import TimedSensorReplay
from dimos.utils.testing.test_moment import Go2Moment


@pytest.fixture
def mapper() -> Generator[VoxelGridMapper, None, None]:
    mapper = VoxelGridMapper()
    yield mapper
    mapper.stop()


class Go2MapperMoment(Go2Moment):
    global_map: OutputMoment[PointCloud2] = OutputMoment(LCMTransport("/global_map", PointCloud2))
    costmap: OutputMoment[OccupancyGrid] = OutputMoment(LCMTransport("/costmap", OccupancyGrid))


@pytest.fixture
def moment():
    moment = Go2MapperMoment()

    def get_moment(ts: float, publish: bool = True) -> Go2Moment:
        moment.seek(ts)
        if publish:
            moment.publish()
        return moment

    yield get_moment
    moment.stop()


@pytest.fixture
def moment1(moment):
    return moment(10, False)


@pytest.fixture
def moment2(moment):
    return moment(10, False)


@pytest.mark.tool
def two_perspectives_loop(moment):
    while True:
        moment(10)
        time.sleep(1)
        moment(85)
        time.sleep(1)


def test_carving(mapper, moment1: Go2MapperMoment, moment2: Go2MapperMoment):
    lidar_frame1 = moment1.lidar.value
    lidar_frame1_transport = LCMTransport("/prev_lidar", PointCloud2)
    lidar_frame1_transport.publish(lidar_frame1)
    lidar_frame1_transport.stop()

    lidar_frame2 = moment2.lidar.value

    # Carving mapper (default, carve_columns=True)
    mapper.add_frame(lidar_frame1)
    mapper.add_frame(lidar_frame2)

    moment2.global_map.set(mapper.get_global_pointcloud2())
    moment2.costmap.set(mapper.get_global_occupancygrid())
    moment2.publish()

    count_carving = mapper.size()
    # Additive mapper (carve_columns=False)
    additive_mapper = VoxelGridMapper(carve_columns=False)
    additive_mapper.add_frame(lidar_frame1)
    additive_mapper.add_frame(lidar_frame2)
    count_additive = additive_mapper.size()

    print("\n=== Carving comparison ===")
    print(f"Additive (no carving): {count_additive}")
    print(f"With carving: {count_carving}")
    print(f"Voxels carved: {count_additive - count_carving}")

    # Carving should result in fewer voxels
    assert count_carving < count_additive, (
        f"Carving should remove some voxels. Additive: {count_additive}, Carving: {count_carving}"
    )

    additive_global_map = LCMTransport("additive_global_map", PointCloud2)
    additive_global_map.publish(additive_mapper.get_global_pointcloud2())
    additive_global_map.stop()
    additive_mapper.stop()


def test_injest_a_few(mapper: VoxelGridMapper) -> None:
    data_dir = get_data("unitree_go2_office_walk2")
    lidar_store = TimedSensorReplay(f"{data_dir}/lidar")  # type: ignore[var-annotated]

    for i in [1, 4, 8]:
        frame = lidar_store.find_closest_seek(i)
        assert frame is not None
        print("add", frame)
        mapper.add_frame(frame)

    assert len(mapper.get_global_pointcloud2()) == 30136


@pytest.mark.parametrize(
    "voxel_size, expected_points",
    [
        (0.5, 277),
        (0.1, 7290),
        (0.05, 28199),
    ],
)
def test_roundtrip(moment1: Go2MapperMoment, voxel_size, expected_points) -> None:
    mapper = VoxelGridMapper(voxel_size=voxel_size)
    mapper.add_frame(moment1.lidar.value)

    global1 = mapper.get_global_pointcloud2()
    assert len(global1) == expected_points

    # loseless roundtrip
    if voxel_size == 0.05:
        assert len(global1) == len(moment1.lidar.value)
        # TODO: we want __eq__ on PointCloud2 - should actually compare
        # all points in both frames

    mapper.add_frame(global1)
    # no new information, no global map change
    assert len(mapper.get_global_pointcloud2()) == len(global1)

    moment1.publish()
    mapper.stop()


def test_roundtrip_range_preserved(mapper: VoxelGridMapper) -> None:
    """Test that input coordinate ranges are preserved in output."""
    data_dir = get_data("unitree_go2_office_walk2")
    lidar_store = TimedSensorReplay(f"{data_dir}/lidar")  # type: ignore[var-annotated]

    frame = lidar_store.find_closest_seek(1.0)
    assert frame is not None
    input_pts = np.asarray(frame.pointcloud.points)

    mapper.add_frame(frame)

    out_pcd = mapper.get_global_pointcloud().to_legacy()
    out_pts = np.asarray(out_pcd.points)

    voxel_size = mapper.config.voxel_size
    tolerance = voxel_size  # Allow one voxel of difference at boundaries

    # TODO: we want __eq__ on PointCloud2 - should actually compare
    # all points in both frames

    for axis, name in enumerate(["X", "Y", "Z"]):
        in_min, in_max = input_pts[:, axis].min(), input_pts[:, axis].max()
        out_min, out_max = out_pts[:, axis].min(), out_pts[:, axis].max()

        assert abs(in_min - out_min) < tolerance, f"{name} min mismatch: in={in_min}, out={out_min}"
        assert abs(in_max - out_max) < tolerance, f"{name} max mismatch: in={in_max}, out={out_max}"
