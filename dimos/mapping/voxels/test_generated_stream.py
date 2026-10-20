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

from contextlib import ExitStack
from copy import deepcopy
from threading import Event

from dimos_cdrgen.registry import decode, encode
import numpy as np
import open3d.core as o3c
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import SimpleOccupancyConfig
from dimos.mapping.voxels.grid import VoxelGrid
from dimos.mapping.voxels.impl.o3d import O3dVoxels
from dimos.mapping.voxels.impl.packed import PackedVoxels
from dimos.mapping.voxels.module import VoxelGridMapper, VoxelMapTransformer
from dimos.memory.type.observation import Observation
from dimos.messages.nav_msgs.msg import OccupancyGrid
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as LegacyCloud
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def keys(points):
    return {tuple(row) for row in np.floor(np.asarray(points) / np.float32(0.05)).astype(np.int64)}


@pytest.mark.parametrize("carve", [False, True])
def test_array_input_matches_legacy_voxel_ingestion(carve):
    with ExitStack() as cleanup:
        old = VoxelGrid(device="CPU:0", carve_columns=carve, show_startup_log=False)
        new = VoxelGrid(device="CPU:0", carve_columns=carve, show_startup_log=False)
        cleanup.callback(old.dispose)
        cleanup.callback(new.dispose)
        for index, points in enumerate(
            ([[0.025, 0.025, 0.025], [0.075, 0.025, 0.025]], [[0.025, 0.025, 0.125]])
        ):
            values = np.array(points, dtype=np.float32)
            old.add_frame(LegacyCloud.from_numpy(values, timestamp=index))
            new.add_points(values, timestamp=index)
        assert keys(old.get_global_pointcloud().point["positions"].numpy()) == keys(
            new.get_global_pointcloud().point["positions"].numpy()
        )
        assert len(old) == len(new) == (2 if carve else 3)


@pytest.mark.parametrize("carve", [False, True])
def test_actual_open3d_cpu_array_entry_matches_packed_store(carve):
    with ExitStack() as cleanup:
        packed = PackedVoxels(0.05, carve)
        tensor = O3dVoxels(0.05, 128, carve, o3c.Device("CPU:0"))
        cleanup.callback(packed.dispose)
        cleanup.callback(tensor.dispose)
        for points in ([[0.025, 0.025, 0.025], [0.075, 0.025, 0.025]], [[0.025, 0.025, 0.125]]):
            values = np.array(points, dtype=np.float32)
            packed.add_points(values, 0)
            tensor.add_points(values, 0)
        assert keys(packed.points()) == keys(tensor.points())
        assert packed.size() == tensor.size() == (2 if carve else 3)


@pytest.mark.parametrize("points", [np.zeros((3,)), np.zeros((1, 2)), np.array([[np.nan, 0, 0]])])
def test_array_entry_rejects_invalid_coordinates(points):
    grid = VoxelGrid(device="CPU:0", show_startup_log=False)
    try:
        with pytest.raises(ValueError, match="Voxel points"):
            grid.add_points(points)
    finally:
        grid.dispose()


@pytest.mark.parametrize("base", [0, -1, 1700000000123456789])
@pytest.mark.parametrize("emit_every,indices", [(0, [2]), (1, [0, 1, 2]), (2, [1, 2])])
def test_generated_stream_retains_exact_headers_batch_tail_and_observation_metadata(
    base, emit_every, indices
):
    frames = []
    for index in range(3):
        cloud = pointcloud_from_xyz(
            np.array([[0.025 + 0.05 * index, 0.025, 0.025]], dtype=np.float32),
            header=Header(frame_id="world", stamp=time_from_nanoseconds(base + index)),
        )
        frames.append(
            Observation(ts=42 + index, data_type=PointCloud2, _data=cloud, tags={"source": "lidar"})
        )
    original = deepcopy(frames[-1].data.header)
    transform = VoxelMapTransformer(
        emit_every=emit_every,
        device="CPU:0",
        voxel_size=0.05,
        frame_id="map",
        show_startup_log=False,
    )

    result = list(transform(iter(frames)))

    assert [to_nanoseconds(item.data.header.stamp) for item in result] == [
        base + index for index in indices
    ]
    assert [item.tags["frame_count"] for item in result] == [index + 1 for index in indices]
    assert [item.ts for item in result] == [42 + index for index in indices]
    assert all(item.tags["source"] == "lidar" and item.pose is None for item in result)
    assert all(item.data.header.frame_id == "map" for item in result)
    received = decode(encode(result[-1].data), PointCloud2)
    assert len(pointcloud_xyz(received)) == 3
    assert frames[-1].data.header == original


def test_generated_empty_and_missing_points_keep_zero_source_stamp():
    header = Header(frame_id="world", stamp=time_from_nanoseconds(0))
    cloud = pointcloud_from_xyz(np.array([[np.nan, 0, 0]], dtype=np.float32), header=header)
    transform = VoxelMapTransformer(device="CPU:0", show_startup_log=False)

    result = list(transform(iter([Observation(ts=123, _data=cloud, data_type=PointCloud2)])))

    assert pointcloud_xyz(result[0].data).shape == (0, 3)
    assert to_nanoseconds(result[0].data.header.stamp) == 0
    assert result[0].ts == 123


def test_voxel_mapper_real_generated_ports_accumulate_over_cdr_memory_transport():
    with ExitStack() as cleanup:
        incoming = CdrTransport("voxel_input", PointCloud2, backend="lcm", url="memq://")
        outgoing = CdrTransport("voxel_map", PointCloud2, backend="lcm", url="memq://")
        cleanup.callback(incoming.stop)
        cleanup.callback(outgoing.stop)
        module = VoxelGridMapper(rpc_transport=MemoryRPC, device="CPU:0", voxel_size=0.05)
        cleanup.callback(module.stop)
        assert module.lidar.type is PointCloud2
        assert module.global_map.type is PointCloud2
        module.lidar.transport = incoming
        module.global_map.transport = outgoing
        received = []
        ready = Event()

        def receive(value):
            received.append(value)
            ready.set()

        cleanup.callback(outgoing.subscribe(receive))
        module.start()
        source = pointcloud_from_xyz(
            np.array([[0.025, 0.025, 0.025]], dtype=np.float32),
            header=Header(frame_id="world", stamp=time_from_nanoseconds(1700000000123456789)),
        )

        incoming.broadcast(None, source)

        assert ready.wait(5)
        assert len(received) == 1
        assert received[0].header == source.header
        assert keys(pointcloud_xyz(received[0])) == {(0, 0, 0)}


def test_actual_voxel_to_costmapper_chain_preserves_source_header_over_cdr():
    with ExitStack() as cleanup:
        source_wire = CdrTransport("voxel_chain_source", PointCloud2, backend="lcm", url="memq://")
        map_wire = CdrTransport("voxel_chain_map", PointCloud2, backend="lcm", url="memq://")
        merged_wire = CdrTransport("voxel_chain_merged", PointCloud2, backend="lcm", url="memq://")
        cost_wire = CdrTransport("voxel_chain_cost", OccupancyGrid, backend="lcm", url="memq://")
        for wire in (source_wire, map_wire, merged_wire, cost_wire):
            cleanup.callback(wire.stop)
        voxel = VoxelGridMapper(rpc_transport=MemoryRPC, device="CPU:0", voxel_size=0.05)
        cost = CostMapper(
            rpc_transport=MemoryRPC, algo="simple", config=SimpleOccupancyConfig(resolution=0.25)
        )
        cleanup.callback(voxel.stop)
        cleanup.callback(cost.stop)
        voxel.lidar.transport = source_wire
        voxel.global_map.transport = map_wire
        cost.global_map.transport = map_wire
        cost.merged_map.transport = merged_wire
        cost.global_costmap.transport = cost_wire
        received = []
        ready = Event()

        def receive(value):
            received.append(value)
            ready.set()

        cleanup.callback(cost_wire.subscribe(receive))
        cost.start()
        voxel.start()
        cloud = pointcloud_from_xyz(
            np.array([[0.025, 0.025, 0.225], [1.025, 1.025, 0.025]], dtype=np.float32),
            header=Header(frame_id="world", stamp=time_from_nanoseconds(1700000000123456789)),
        )

        source_wire.broadcast(None, cloud)

        assert ready.wait(5)
        assert len(received) == 1
        assert received[0].header == cloud.header
        assert received[0].info.map_load_time == cloud.header.stamp
        assert 100 in received[0].data
