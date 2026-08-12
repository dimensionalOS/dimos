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

"""CodSpeed simulation benchmark: the go2 replay compute pipeline, one process.

The single-process complement to test_replay_benchmark.py. Simulation mode
counts instructions under Valgrind in one process, so this drives the
pipeline's compute directly and fully synchronously: decode the replay
window's frames (JPEG images, lidar pointclouds, odometry) and pull the lidar
through VoxelMapTransformer -> CostMapper._calculate_costmap — the same work
the e2e blueprint performs (its planner/patrol/movement modules idle without
goals), minus worker processes, transports, and schedulers.

Deliberately no Module/rx machinery in the measured region: the timed replay's
shared wall-clock anchor skips late subscribers at high speed (fatal under
Valgrind's dilation), and module RPC/loop threads pollute the count. Pure
iterator pulls make completion exact by construction. Also runs as a plain
self_hosted test for coverage.

Local smoke (small bundled recording):

    DIMOS_SIM_BENCH_DURATION=5 \
        uv run pytest dimos/robot/unitree/go2/test_replay_pipeline_benchmark.py \
        -m self_hosted --no-cov -v
"""

from collections.abc import Iterator
import os
from typing import Any

import pytest
from pytest_codspeed import BenchmarkFixture

from dimos.core.global_config import global_config
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.unitree.go2.test_replay_benchmark import _expected_counts

REPLAY_DB = os.environ.get("DIMOS_SIM_BENCH_REPLAY_DB", "go2_short")
DURATION = float(os.environ.get("DIMOS_SIM_BENCH_DURATION", 10))


class _NoRpc(RPCSpec):
    """Disables the module RPC service (ModuleBase catches this ValueError).

    The benchmark only needs CostMapper for its configured compute; an RPC
    service would add zenoh/LCM threads (and hangs zenoh setup on macOS).
    """

    def __init__(self, **_kwargs: Any) -> None:
        raise ValueError("module RPC disabled for the benchmark")


@pytest.mark.self_hosted
@pytest.mark.timeout(600)
def test_go2_pipeline_simulation(benchmark: BenchmarkFixture) -> None:
    """Decode a replay window and pull it through voxel map -> costmap."""
    from dimos.mapping.costmapper import CostMapper
    from dimos.mapping.voxels.module import VoxelMapTransformer
    from dimos.memory2.replay import resolve_db_path
    from dimos.memory2.store.sqlite import SqliteStore

    saved = global_config.model_dump()
    global_config.update(viewer="none")
    cost = None
    store = None
    try:
        db_path = str(resolve_db_path(REPLAY_DB))  # LFS pull/extract on miss
        expected = _expected_counts(db_path, duration=DURATION)
        assert all(count > 0 for count in expected.values()), f"empty window: {expected}"

        store = SqliteStore(path=db_path, must_exist=True)
        store.start()
        replay = store.replay(duration=DURATION)
        available = replay.list_streams()
        lidar_name = "go2_lidar" if "go2_lidar" in available else "lidar"
        odom_name = "go2_odom" if "go2_odom" in available else "odom"
        window_end = replay.first_ts() + DURATION  # type: ignore[operator]

        def windowed(name: str) -> Any:
            # The duration-only window of Replay._base_stream: everything
            # before recording start + duration, in timestamp order.
            return store.stream(name).before(window_end).order_by("ts")

        # Only the config'd compute is used (never started as a module).
        cost = CostMapper(g=global_config, rpc_transport=_NoRpc)
        # Suppressed rpc setup never assigns the attribute; _close_rpc guards
        # on truthiness, so seed it for a clean _close_module in the finally.
        cost.rpc = None

        counts = {"odom": 0, "lidar": 0, "color_image": 0}
        produced = {"global_map": 0, "global_costmap": 0}

        def counted(observations: Iterator[Any], key: str) -> Iterator[Any]:
            for obs in observations:
                counts[key] += 1
                yield obs

        def drain() -> None:
            # The blueprint's voxel pipeline: accumulate every lidar frame,
            # emit the global map every 5th, costmap each emitted map.
            transformer = VoxelMapTransformer(
                emit_every=5,
                voxel_size=0.05,
                block_count=2_000_000,
                device="CPU:0",
                carve_columns=True,
                frame_id="world",
            )
            for map_obs in transformer(counted(iter(windowed(lidar_name)), "lidar")):
                produced["global_map"] += 1
                cost._calculate_costmap(map_obs.data)
                produced["global_costmap"] += 1
            # Decode odom and camera frames like GO2Connection does; their
            # consumers idle in the e2e too.
            for obs in counted(iter(windowed(odom_name)), "odom"):
                _ = obs.data
            for obs in counted(iter(windowed("color_image")), "color_image"):
                _ = obs.data
            if counts != expected or produced["global_costmap"] == 0:
                pytest.fail(f"incomplete drain: counts={counts}, expected={expected}, {produced}")

        def teardown() -> None:
            for key in counts:
                counts[key] = 0
            for key in produced:
                produced[key] = 0

        benchmark.pedantic(drain, teardown=teardown, rounds=1, warmup_rounds=0)
    finally:
        if cost is not None:
            cost._close_module()
        if store is not None:
            store.stop()
        global_config.update(**saved)
