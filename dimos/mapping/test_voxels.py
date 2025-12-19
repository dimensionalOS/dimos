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
from dimos.mapping.voxels import SparseVoxelGridMapper
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.data import get_data
from dimos.utils.testing.moment import OutputMoment
from dimos.utils.testing.test_moment import Go2Moment


@pytest.fixture
def mapper() -> Generator[SparseVoxelGridMapper, None, None]:
    mapper = SparseVoxelGridMapper()
    yield mapper
    mapper.stop()


class Go2MapperMoment(Go2Moment):
    global_map: OutputMoment[PointCloud2] = OutputMoment(LCMTransport("global_map", PointCloud2))


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


@pytest.mark.tool
def two_perspectives_loop(moment):
    while True:
        moment(10)
        time.sleep(1)
        moment(85)
        time.sleep(1)


def test_carving(mapper, moment):
    moment1 = moment(10, False)

    lidar_frame1 = moment1.lidar.value
    lidar_frame1_transport = LCMTransport("prev_lidar", PointCloud2)
    lidar_frame1_transport.publish(lidar_frame1)
    lidar_frame1_transport.stop()

    moment2 = moment(85, False)

    lidar_frame2 = moment2.lidar.value

    # Carving mapper (default, carve_columns=True)
    mapper.add_frame(lidar_frame1)
    mapper.add_frame(lidar_frame2)

    moment2.global_map.set(mapper.get_global_pointcloud2())
    moment2.publish()

    count_carving = mapper._hm.size()
    # Additive mapper (carve_columns=False)
    additive_mapper = SparseVoxelGridMapper(carve_columns=False)
    additive_mapper.add_frame(lidar_frame1)
    additive_mapper.add_frame(lidar_frame2)
    count_additive = additive_mapper._hm.size()

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


def test_injest_a_few(mapper: SparseVoxelGridMapper) -> None:
    data_dir = get_data("unitree_go2_office_walk2")
    lidar_store = TimedSensorReplay(f"{data_dir}/lidar")  # type: ignore[var-annotated]

    frame = lidar_store.find_closest_seek(1.0)
    assert frame is not None
    print("add", frame)
    mapper.add_frame(frame)

    print(mapper.get_global_pointcloud2())


def test_roundtrip_coordinates(mapper: SparseVoxelGridMapper) -> None:
    """Test that voxelization preserves point coordinates within voxel resolution."""
    voxel_size = mapper.config.voxel_size

    # Create synthetic points at known voxel centers
    # Points at voxel centers should round-trip exactly
    input_pts = np.array(
        [
            [0.0, 0.0, 0.0],
            [voxel_size, 0.0, 0.0],
            [0.0, voxel_size, 0.0],
            [0.0, 0.0, voxel_size],
            [-voxel_size, -voxel_size, -voxel_size],
            [1.0, 2.0, 3.0],
            [-1.5, 0.5, -0.25],
        ],
        dtype=np.float32,
    )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(input_pts)
    frame = LidarMessage(pointcloud=pcd, ts=0.0)

    mapper.add_frame(frame)

    out_pcd = mapper.get_global_pointcloud().to_legacy()
    out_pts = np.asarray(out_pcd.points)

    # Same number of unique voxels
    assert len(out_pts) == len(input_pts), f"Expected {len(input_pts)} voxels, got {len(out_pts)}"

    # Each output point should be within half a voxel of an input point
    # (output is voxel center, input could be anywhere in voxel)
    for out_pt in out_pts:
        dists = np.linalg.norm(input_pts - out_pt, axis=1)
        min_dist = dists.min()
        assert min_dist < voxel_size, (
            f"Output point {out_pt} too far from any input (dist={min_dist})"
        )


def test_roundtrip_range_preserved(mapper: SparseVoxelGridMapper) -> None:
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

    for axis, name in enumerate(["X", "Y", "Z"]):
        in_min, in_max = input_pts[:, axis].min(), input_pts[:, axis].max()
        out_min, out_max = out_pts[:, axis].min(), out_pts[:, axis].max()

        assert abs(in_min - out_min) < tolerance, f"{name} min mismatch: in={in_min}, out={out_min}"
        assert abs(in_max - out_max) < tolerance, f"{name} max mismatch: in={in_max}, out={out_max}"
