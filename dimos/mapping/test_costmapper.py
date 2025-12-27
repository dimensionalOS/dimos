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

from dataclasses import asdict
import pickle
import time

import pytest

from dimos.core import In, LCMTransport, Module, Out, rpc, start
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import OCCUPANCY_ALGOS, SimpleOccupancyConfig
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.data import _get_data_dir, get_data
from dimos.utils.testing import TimedSensorReplay


def test_costmap_direct_no_deploy():
    """Test costmap calculation directly without dask deployment.

    This isolates whether the delay is in the algorithm or the messaging layer.
    """
    seekt = 200.0
    seed = seed_map(seekt)

    # Create mapper and costmapper as plain objects (no deployment)
    mapper = VoxelGridMapper(publish_interval=-1)
    mapper.add_frame(seed)

    # Get the costmap function directly
    costmap_fn = OCCUPANCY_ALGOS["simple"]
    cfg = SimpleOccupancyConfig()

    frame_count = 0
    total_mapper_time = 0.0
    total_costmap_time = 0.0

    print("\n=== Direct (no deploy) timing test ===")

    for _ts, frame in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_duration(seek=seekt):
        if frame_count >= 100:  # Test 100 frames
            break

        # Time the mapper
        t0 = time.perf_counter()
        mapper.add_frame(frame)
        global_pc = mapper.get_global_pointcloud2()
        t1 = time.perf_counter()

        # Time the costmap calculation
        costmap_fn(global_pc, **asdict(cfg))
        t2 = time.perf_counter()

        mapper_time = (t1 - t0) * 1000
        costmap_time = (t2 - t1) * 1000
        total_mapper_time += mapper_time
        total_costmap_time += costmap_time

        if frame_count % 20 == 0:
            print(
                f"Frame {frame_count}: mapper={mapper_time:.1f}ms, costmap={costmap_time:.1f}ms, total={mapper_time + costmap_time:.1f}ms"
            )

        frame_count += 1

    mapper.stop()

    print(f"\n=== Summary ({frame_count} frames) ===")
    print(f"Avg mapper time: {total_mapper_time / frame_count:.1f}ms")
    print(f"Avg costmap time: {total_costmap_time / frame_count:.1f}ms")
    print(f"Avg total time: {(total_mapper_time + total_costmap_time) / frame_count:.1f}ms")


def test_costmap_with_reactive_no_deploy():
    """Test with reactive pipeline but no dask deployment.

    This isolates whether the delay is in RxPy/backpressure or cross-process comm.
    Runs 300+ messages to observe delay buildup over time.
    """
    import reactivex as rx
    from reactivex import operators as ops

    from dimos.utils.reactive import backpressure

    seekt = 200.0
    seed = seed_map(seekt)

    mapper = VoxelGridMapper(publish_interval=-1)
    mapper.add_frame(seed)

    costmap_fn = OCCUPANCY_ALGOS["simple"]
    cfg = SimpleOccupancyConfig()

    received_costmaps = []
    send_times = {}  # msg_ts -> wall time when sent

    print("\n=== Reactive pipeline (no deploy) timing test ===")

    from reactivex.subject import Subject

    frame_subject = Subject()

    def process_frame(frame):
        mapper.add_frame(frame)
        global_pc = mapper.get_global_pointcloud2()
        return global_pc

    def calc_costmap(pc):
        costmap = costmap_fn(pc, **asdict(cfg))
        return costmap

    # Simulate what CostMapper does: backpressure on input, then map
    bp_pipeline = backpressure(
        frame_subject.pipe(
            ops.map(process_frame),
        )
    ).pipe(
        ops.map(calc_costmap),
    )

    def on_costmap(costmap):
        received_costmaps.append((costmap.ts, time.perf_counter()))

    bp_pipeline.subscribe(on_costmap)

    frame_count = 0
    for _ts, frame in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_duration(seek=seekt):
        if frame_count >= 300:
            break
        send_times[frame.ts] = time.perf_counter()
        frame_subject.on_next(frame)
        frame_count += 1

    # Wait for pipeline to drain
    time.sleep(1.0)

    print(f"Sent {frame_count} frames, received {len(received_costmaps)} costmaps")

    # Calculate delays and show progression
    if received_costmaps:
        delays = []
        for i, (costmap_ts, recv_wall) in enumerate(received_costmaps):
            if costmap_ts in send_times:
                delay = (recv_wall - send_times[costmap_ts]) * 1000
                delays.append(delay)
                if i % 10 == 0:
                    print(f"  costmap #{i}: delay={delay:.1f}ms")

        if delays:
            print(
                f"\nPipeline delays: min={min(delays):.1f}ms, max={max(delays):.1f}ms, avg={sum(delays) / len(delays):.1f}ms"
            )

    mapper.stop()


def test_costmap_with_lcm_no_deploy():
    """Test with LCM transport but no dask deployment.

    This isolates whether the delay is in LCM serialization or cross-process comm.
    """
    from dimos.protocol.pubsub.lcmpubsub import LCM, Topic

    seekt = 200.0
    seed = seed_map(seekt)

    mapper = VoxelGridMapper(publish_interval=-1)
    mapper.add_frame(seed)

    OCCUPANCY_ALGOS["simple"]
    SimpleOccupancyConfig()

    print("\n=== LCM transport (no deploy) timing test ===")

    # Create LCM pubsub
    lcm = LCM()
    lcm.start()

    topic = Topic("/test_global_map", PointCloud2)

    received_pcs = []
    publish_times = {}

    def on_pc(pc, _topic):
        recv_time = time.perf_counter()
        received_pcs.append((pc, recv_time))

    lcm.subscribe(topic, on_pc)

    # Publish frames through LCM and measure round-trip
    frame_count = 0
    for _ts, frame in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_duration(seek=seekt):
        if frame_count >= 50:
            break

        mapper.add_frame(frame)
        global_pc = mapper.get_global_pointcloud2()

        pub_time = time.perf_counter()
        publish_times[global_pc.ts] = pub_time
        lcm.publish(topic, global_pc)

        frame_count += 1
        time.sleep(0.01)  # Small delay to let messages be received

    time.sleep(0.5)  # Wait for last messages

    print(f"Published {frame_count} frames, received {len(received_pcs)} frames")

    # Calculate LCM delays
    delays = []
    for pc, recv_time in received_pcs:
        if pc.ts in publish_times:
            delay = (recv_time - publish_times[pc.ts]) * 1000
            delays.append(delay)

    if delays:
        print(
            f"LCM delays: min={min(delays):.1f}ms, max={max(delays):.1f}ms, avg={sum(delays) / len(delays):.1f}ms"
        )

    lcm.stop()
    mapper.stop()


def seed_map(target: float = 200.0):
    mapper = VoxelGridMapper(publish_interval=-1)
    print("seeding map up to time:", target)
    for ts, frame in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_duration():
        # print(ts, frame)
        if ts > target:
            break
        mapper.add_frame(frame)

    global_pc = mapper.get_global_pointcloud2()
    mapper.stop()
    print("done")
    return global_pc


def test_costmap_calc():
    seekt = 200.0
    seed = seed_map(seekt)

    dimos = start(2)
    mapper = dimos.deploy(VoxelGridMapper, publish_interval=0)
    costmapper = dimos.deploy(CostMapper)

    mapper.add_frame(seed)

    mapper.global_map.transport = LCMTransport("/global_map", PointCloud2)
    mapper.lidar.transport = LCMTransport("/lidar", PointCloud2)

    costmapper.global_map.connect(mapper.global_map)
    costmapper.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)

    mapper.start()
    costmapper.start()

    # Track wall clock times for latency measurement
    map_wall_times = {}  # data_ts -> wall_time when map received
    costmap_count = 0
    latencies = []

    def on_costmap(costmap):
        nonlocal costmap_count
        recv_time = time.perf_counter()
        costmap_count += 1

        # Find matching map by data timestamp
        if costmap.ts in map_wall_times:
            latency_ms = (recv_time - map_wall_times[costmap.ts]) * 1000
            latencies.append(latency_ms)
            print(f"costmap #{costmap_count}: {costmap} | latency={latency_ms:.1f}ms")
        else:
            print(f"costmap #{costmap_count}: {costmap} | no matching map ts")

    def on_map(pc):
        map_wall_times[pc.ts] = time.perf_counter()

    costmapper.global_costmap.subscribe(on_costmap)
    mapper.global_map.subscribe(on_map)

    for msg in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_realtime(
        seek=seekt, duration=30.0
    ):
        mapper.lidar.transport.publish(msg)

    print("closing")

    if latencies:
        print(f"\n=== Latency Summary ({len(latencies)} samples) ===")
        print(
            f"Min: {min(latencies):.1f}ms, Max: {max(latencies):.1f}ms, Avg: {sum(latencies) / len(latencies):.1f}ms"
        )

    mapper.stop()
    costmapper.stop()
    dimos.stop()
