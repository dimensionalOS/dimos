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


def test_load_old_pickled_pcd():
    """Test that old pickled LidarMessage/PointCloud2 objects load correctly."""
    # Load one frame from replay (old pickle format)
    for _ts, frame in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_duration():
        print(f"Frame type: {type(frame)}")
        print(f"Frame __dict__ keys: {list(frame.__dict__.keys())}")

        # Check if pointcloud is in __dict__ (old format)
        if "pointcloud" in frame.__dict__:
            old_pcd = frame.__dict__["pointcloud"]
            print(f"Old pointcloud in __dict__: {type(old_pcd)}, points: {len(old_pcd.points)}")

        # Try accessing via property
        print("Accessing .pointcloud property...")
        pcd = frame.pointcloud
        print(f"Pointcloud: {type(pcd)}, points: {len(pcd.points)}")

        # Try as_numpy
        print("Accessing .as_numpy()...")
        pts = frame.as_numpy()
        print(f"Numpy points shape: {pts.shape}")

        break  # Just test one frame


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


def test_lcm_raw_vs_decode():
    """Test raw LCM bytes transport vs PointCloud2 decode time."""
    from dimos.protocol.pubsub.lcmpubsub import LCM, LCMPubSubBase, Topic

    seekt = 200.0
    seed = seed_map(seekt)

    mapper = VoxelGridMapper(publish_interval=-1)
    mapper.add_frame(seed)

    print("\n=== Raw LCM bytes vs PointCloud2 decode ===")

    # Get a sample global pointcloud and encode it
    global_pc = mapper.get_global_pointcloud2()
    encoded = global_pc.lcm_encode()
    print(f"Encoded size: {len(encoded) / 1024:.1f} KB")

    # Time just the decode
    import statistics

    decode_times = []
    for _ in range(20):
        t0 = time.perf_counter()
        PointCloud2.lcm_decode(encoded)
        decode_times.append((time.perf_counter() - t0) * 1000)

    print(
        f"PointCloud2.lcm_decode(): avg={statistics.mean(decode_times):.1f}ms, stdev={statistics.stdev(decode_times):.1f}ms"
    )

    # Time just the encode
    encode_times = []
    for _ in range(20):
        t0 = time.perf_counter()
        global_pc.lcm_encode()
        encode_times.append((time.perf_counter() - t0) * 1000)

    print(
        f"PointCloud2.lcm_encode(): avg={statistics.mean(encode_times):.1f}ms, stdev={statistics.stdev(encode_times):.1f}ms"
    )

    # Now test raw LCM transport with raw bytes (no encode/decode)
    lcm_raw = LCMPubSubBase()
    lcm_raw.start()

    raw_topic = Topic("/test_raw_bytes")
    received_raw = []
    raw_publish_times = {}

    def on_raw(data, _topic):
        recv_time = time.perf_counter()
        received_raw.append((id(data), recv_time, len(data)))

    lcm_raw.subscribe(raw_topic, on_raw)

    # Publish raw bytes 100 times
    for i in range(100):
        pub_time = time.perf_counter()
        raw_publish_times[i] = pub_time
        lcm_raw.publish(raw_topic, encoded)  # same encoded bytes each time

    time.sleep(0.5)

    print(f"\nRaw bytes transport: published 100, received {len(received_raw)}")
    if received_raw:
        # Calculate transport-only delays (no decode in callback)
        print("  (callback just stores bytes, no decode)")

    lcm_raw.stop()
    mapper.stop()


def test_costmap_with_lcm_no_deploy():
    """Test with LCM transport but no dask deployment.

    This isolates whether the delay is in LCM serialization or cross-process comm.
    Publishes global pointclouds at realistic rates (no artificial sleep).
    """
    from dimos.protocol.pubsub.lcmpubsub import LCM, Topic

    seekt = 200.0
    seed = seed_map(seekt)

    mapper = VoxelGridMapper(publish_interval=-1)
    mapper.add_frame(seed)

    print("\n=== LCM transport latency test (300 frames, no sleep) ===")

    # Create LCM pubsub
    lcm = LCM()
    lcm.start()

    topic = Topic("/test_global_map", PointCloud2)

    received_pcs = []
    publish_times = {}

    decode_times = []

    def on_pc(pc, _topic):
        recv_time = time.perf_counter()
        received_pcs.append((pc.ts, recv_time))
        # Track how long since last callback (shows if callbacks are slow)
        if len(received_pcs) > 1:
            decode_times.append(recv_time - received_pcs[-2][1])

    lcm.subscribe(topic, on_pc)

    # Publish frames through LCM as fast as possible (no sleep)
    frame_count = 0
    pc_size_bytes = 0
    for _ts, frame in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_duration(seek=seekt):
        if frame_count >= 300:
            break

        mapper.add_frame(frame)
        global_pc = mapper.get_global_pointcloud2()

        # Track size of first pointcloud
        if frame_count == 0:
            import numpy as np

            pc_size_bytes = len(global_pc.lcm_encode())
            num_points = len(np.asarray(global_pc.pointcloud.points))
            print(f"Pointcloud size: {pc_size_bytes / 1024:.1f} KB, {num_points} points")

        pub_time = time.perf_counter()
        publish_times[global_pc.ts] = pub_time
        lcm.publish(topic, global_pc)

        frame_count += 1

    # Wait for pipeline to drain
    time.sleep(1.0)

    print(f"Published {frame_count} frames, received {len(received_pcs)} frames")

    # Calculate LCM delays and show progression
    delays = []
    for i, (pc_ts, recv_time) in enumerate(received_pcs):
        if pc_ts in publish_times:
            delay = (recv_time - publish_times[pc_ts]) * 1000
            delays.append(delay)
            if i % 30 == 0:
                print(f"  msg #{i}: delay={delay:.1f}ms")

    if delays:
        print(f"\nLCM delays over {len(delays)} messages:")
        print(
            f"  min={min(delays):.1f}ms, max={max(delays):.1f}ms, avg={sum(delays) / len(delays):.1f}ms"
        )
        print(f"  first 10 avg: {sum(delays[:10]) / 10:.1f}ms")
        print(f"  last 10 avg: {sum(delays[-10:]) / 10:.1f}ms")

    if decode_times:
        print("\nTime between callbacks (decode time):")
        print(
            f"  min={min(decode_times) * 1000:.1f}ms, max={max(decode_times) * 1000:.1f}ms, avg={sum(decode_times) / len(decode_times) * 1000:.1f}ms"
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
