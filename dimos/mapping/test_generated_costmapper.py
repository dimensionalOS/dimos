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

import cv2
from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.occupancy.visualizations import rainbow_image
from dimos.mapping.pointclouds.occupancy import (
    SimpleOccupancyConfig,
    general_occupancy,
    height_cost_occupancy,
    simple_occupancy,
)
from dimos.mapping.pointclouds.util import read_pointcloud
from dimos.message_utils.occupancy import occupancy_from_array, occupancy_view
from dimos.message_utils.pointcloud import pointcloud_from_xyz
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.nav_msgs.msg import OccupancyGrid
from dimos.messages.sensor_msgs.msg import PointCloud2, PointField
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid as LegacyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as LegacyCloud
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.data import get_data
from dimos.utils.threadpool import get_max_workers


@pytest.fixture(params=[(0, 0), (-1, 999999999), (1700000000, 123456789)])
def source_header(request):
    return Header(frame_id="lidar", stamp=Time(sec=request.param[0], nanosec=request.param[1]))


@pytest.mark.parametrize("algorithm", [simple_occupancy, general_occupancy, height_cost_occupancy])
def test_generated_costmaps_match_existing_math_and_keep_exact_header(source_header, algorithm):
    points = np.array(
        [[-1, -1, 0], [-0.5, 0, 0.4], [0, 0, 0.4], [0.5, 0.5, 0], [1, 1, 0.2]], dtype=np.float32
    )
    cloud = pointcloud_from_xyz(points, header=source_header)
    legacy = LegacyCloud.from_numpy(points, frame_id="lidar", timestamp=0)

    actual = algorithm(cloud, resolution=0.25)
    original = algorithm(legacy, resolution=0.25)
    received = decode(encode(actual), OccupancyGrid)

    assert type(original) is LegacyGrid
    assert type(actual) is OccupancyGrid
    assert received.header == source_header
    assert received.info.map_load_time == source_header.stamp
    assert original.ts == legacy.ts
    assert received.info.resolution == original.resolution
    assert (received.info.origin.position.x, received.info.origin.position.y) == (
        original.origin.position.x,
        original.origin.position.y,
    )
    np.testing.assert_array_equal(occupancy_view(received), original.grid)
    actual.header.stamp.nanosec = 0
    assert cloud.header == source_header


@pytest.mark.parametrize("algorithm", [simple_occupancy, general_occupancy, height_cost_occupancy])
def test_empty_generated_cloud_retains_source_metadata_and_unknown_map(source_header, algorithm):
    cloud = pointcloud_from_xyz(np.empty((0, 3), dtype=np.float32), header=source_header)

    actual = algorithm(cloud, resolution=0.25, frame_id="map")

    assert actual.header.frame_id == "map"
    assert actual.header.stamp == source_header.stamp
    assert actual.info.map_load_time == source_header.stamp
    assert occupancy_view(actual).tolist() == [[-1]]


def test_generated_missing_points_are_ignored_before_rasterization():
    header = Header(frame_id="lidar", stamp=Time(sec=0, nanosec=0))
    points = np.array([[0, 0, 0.4], [1, 1, 0], [np.nan, 0, 0], [0, np.inf, 0]], dtype=np.float32)

    actual = simple_occupancy(pointcloud_from_xyz(points, header=header), resolution=0.25)
    expected = simple_occupancy(pointcloud_from_xyz(points[:2], header=header), resolution=0.25)

    np.testing.assert_array_equal(actual.data, expected.data)
    assert actual.header == header


class MemoryRPC(LCMRPC):
    """Use the real RPC implementation with an in-memory transport."""

    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


@pytest.fixture
def mapper_factory():
    with ExitStack() as cleanup:

        def create(**kwargs):
            module = CostMapper(rpc_transport=MemoryRPC, **kwargs)
            cleanup.callback(module.stop)
            return module

        yield create, cleanup


def test_initial_safe_radius_uses_owned_cells_without_changing_metadata(mapper_factory):
    create, _ = mapper_factory
    module = create(initial_safe_radius_meters=0.25)
    header = Header(frame_id="map", stamp=Time(sec=1700000000, nanosec=123456789))
    grid = occupancy_from_array(np.full((3, 3), 100, dtype=np.int8), header=header, resolution=0.25)
    original = deepcopy(grid.header)

    module._apply_initial_safe_radius(grid)

    assert occupancy_view(grid).tolist() == [[0, 0, 100], [0, 0, 100], [100, 100, 100]]
    assert grid.header == original
    assert grid.info.map_load_time == original.stamp


def test_costmapper_actual_generated_ports_deliver_cdr_without_network(
    mapper_factory, owned_scheduler
):
    create, cleanup = mapper_factory
    global_wire = LCMTransport("costmap_global", PointCloud2, url="memq://")
    merged_wire = LCMTransport("costmap_merged", PointCloud2, url="memq://")
    output_wire = LCMTransport("costmap_output", OccupancyGrid, url="memq://")
    for wire in (global_wire, merged_wire, output_wire):
        cleanup.callback(wire.stop)
    module = create(algo="simple", config=SimpleOccupancyConfig(resolution=0.25))
    assert module.global_map.type is PointCloud2
    assert module.global_costmap.type is OccupancyGrid
    module.global_map.transport = global_wire
    module.merged_map.transport = merged_wire
    module.global_costmap.transport = output_wire
    received = []
    ready = Event()

    def receive(value):
        received.append(value)
        ready.set()

    cleanup.callback(output_wire.subscribe(receive))
    header = Header(frame_id="lidar", stamp=Time(sec=1700000000, nanosec=123456789))
    cloud = pointcloud_from_xyz(np.array([[0, 0, 0.4], [1, 1, 0]], dtype=np.float32), header=header)
    module.start()

    global_wire.broadcast(None, cloud)

    assert ready.wait(5)
    assert len(received) == 1
    assert received[0].header == header
    assert 100 in received[0].data

    ready.clear()
    merged_header = Header(frame_id="merged_map", stamp=Time(sec=1700000000, nanosec=123456790))
    merged = pointcloud_from_xyz(
        np.array([[0, 0, 0.4], [1, 1, 0]], dtype=np.float32), header=merged_header
    )
    merged_wire.broadcast(None, merged)
    assert ready.wait(5)
    assert len(received) == 2
    assert received[-1].header == merged_header


@pytest.mark.self_hosted
@pytest.mark.parametrize(
    "source_name,algorithm,expected_name,grayscale",
    [
        ("apartment", simple_occupancy, "occupancy_simple.png", True),
        ("big_office.ply", height_cost_occupancy, "big_office_height_cost_occupancy.png", False),
        ("big_office.ply", simple_occupancy, "big_office_simple_occupancy.png", False),
    ],
)
def test_generated_real_pointcloud_costmap_matches_existing_golden(
    source_name, algorithm, expected_name, grayscale
):
    source = get_data(source_name)
    if source_name == "apartment":
        source = source / "sum.ply"
    points = np.asarray(read_pointcloud(source).points).astype("<f8")
    header = Header(frame_id="map", stamp=Time(sec=1700000000, nanosec=123456789))
    # The original offline goldens use the PLY's float64 coordinates. Preserve
    # that legal ROS PointCloud2 layout instead of quantizing fixture input.
    cloud = PointCloud2(
        header=header,
        height=1,
        width=len(points),
        fields=[
            PointField(name=name, offset=index * 8, datatype=PointField.FLOAT64, count=1)
            for index, name in enumerate(("x", "y", "z"))
        ],
        is_bigendian=False,
        point_step=24,
        row_step=len(points) * 24,
        data=np.frombuffer(points.tobytes(), dtype=np.uint8).copy(),
        is_dense=True,
    )

    actual = algorithm(decode(encode(cloud), PointCloud2))
    cells = occupancy_view(decode(encode(actual), OccupancyGrid))
    if grayscale:
        expected = cv2.imread(str(get_data(expected_name)), cv2.IMREAD_GRAYSCALE)
        pixels = (cells + 1).astype(np.uint8)
    else:
        expected = cv2.imread(str(get_data(expected_name)), cv2.IMREAD_COLOR)
        pixels = rainbow_image(cells)

    np.testing.assert_array_equal(pixels, expected)
    assert actual.header == header


@pytest.fixture
def owned_scheduler(monkeypatch):
    scheduler = ThreadPoolScheduler(max_workers=get_max_workers())
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    monkeypatch.setattr("dimos.utils.threadpool.get_scheduler", lambda: scheduler)
    yield scheduler
    scheduler.executor.shutdown(wait=True)
