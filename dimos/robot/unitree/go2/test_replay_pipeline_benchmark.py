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
counts instructions in one process under Valgrind, so this runs the pipeline's
computational core in-process: GO2Connection (replay decode: JPEG frames,
lidar, odom) -> VoxelGridMapper -> CostMapper. The planners/patrol/movement
modules are omitted — with no goals published they are idle in the e2e run
too, so this covers the same work that actually executes there, minus worker
processes and UDP transports.

Modules are constructed directly (no coordinator, no workers) and wired with
a synchronous in-test LocalTransport shared per topic, so delivery is
lossless and the end condition is exact: the run completes when every frame
of the replay window has been observed. Deterministic enough for instruction
counting; also runs as a plain self_hosted test for coverage.

Local smoke (small bundled recording):

    DIMOS_SIM_BENCH_REPLAY_DB=go2_short DIMOS_SIM_BENCH_DURATION=5 \
        uv run pytest dimos/robot/unitree/go2/test_replay_pipeline_benchmark.py \
        -m self_hosted --no-cov -v
"""

from collections.abc import Callable
import os
import threading
import time
from typing import Any

import pytest
from pytest_codspeed import BenchmarkFixture

from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.robot.unitree.go2.test_replay_benchmark import _expected_counts

REPLAY_DB = os.environ.get("DIMOS_SIM_BENCH_REPLAY_DB", "go2_short")
DURATION = float(os.environ.get("DIMOS_SIM_BENCH_DURATION", 10))
SPEED = 1000.0  # every emission delay clamps to 0: a pure CPU-bound drain
DRAIN_TIMEOUT = 420.0  # start+drain deadline, inside the test timeout


class LocalTransport(PubSubTransport[Any]):
    """Synchronous in-process pub/sub: broadcast() calls subscribers inline.

    Lossless and deterministic — no sockets, no queues, no threads of its
    own — which is what makes exact-count completion and instruction
    counting under Valgrind meaningful.
    """

    def __init__(self, topic: str) -> None:
        super().__init__(topic)
        self._subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, stream: Any, msg: Any) -> None:
        for callback in list(self._subscribers):
            callback(msg)

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Any = None
    ) -> Callable[[], None]:
        self._subscribers.append(callback)
        return lambda: self._subscribers.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._subscribers.clear()


@pytest.mark.self_hosted
@pytest.mark.timeout(600)
def test_go2_pipeline_simulation(benchmark: BenchmarkFixture) -> None:
    """Drain a replay window through decode -> voxel map -> costmap, in-process."""
    from dimos.mapping.costmapper import CostMapper
    from dimos.mapping.voxels.module import VoxelGridMapper
    from dimos.memory2.replay import resolve_db_path
    from dimos.robot.unitree.go2.connection import GO2Connection

    saved = global_config.model_dump()
    global_config.update(
        replay=True,
        replay_db=REPLAY_DB,
        replay_speed=SPEED,
        replay_duration=DURATION,
        viewer="none",
    )
    modules: list[Any] = []
    try:
        db_path = str(resolve_db_path(REPLAY_DB))  # LFS pull/extract on miss
        expected = _expected_counts(db_path, duration=DURATION)
        assert all(count > 0 for count in expected.values()), f"empty window: {expected}"

        go2 = GO2Connection(g=global_config)
        voxel = VoxelGridMapper(g=global_config, emit_every=5, device="CPU:0")
        cost = CostMapper(g=global_config)
        modules = [go2, voxel, cost]

        # One shared transport per topic; producer Out and consumer In point
        # at the same object, exactly like the transport-pinning pattern in
        # test_basic_deployment — just with in-process dispatch.
        topics = {
            name: LocalTransport(name)
            for name in (
                "odom",
                "lidar",
                "color_image",
                "camera_info",
                "pointcloud",
                "tf",
                "cmd_vel",
                "global_map",
                "merged_map",
                "global_costmap",
            )
        }
        go2.odom.transport = topics["odom"]
        go2.lidar.transport = topics["lidar"]
        go2.color_image.transport = topics["color_image"]
        go2.camera_info.transport = topics["camera_info"]
        go2.pointcloud.transport = topics["pointcloud"]
        go2.tf.transport = topics["tf"]
        go2.cmd_vel.transport = topics["cmd_vel"]
        voxel.lidar.transport = topics["lidar"]
        voxel.global_map.transport = topics["global_map"]
        cost.global_map.transport = topics["global_map"]
        cost.merged_map.transport = topics["merged_map"]  # no producer: stays silent
        cost.global_costmap.transport = topics["global_costmap"]

        counts = {"odom": 0, "lidar": 0, "color_image": 0, "global_costmap": 0}
        lock = threading.Lock()

        def record(name: str) -> None:
            with lock:
                counts[name] += 1

        for name in counts:
            topics[name].subscribe(lambda _msg, _name=name: record(_name))

        def start_and_drain() -> None:
            # Consumers first so no frame is emitted before its subscriber
            # exists; the replay starts inside go2.start().
            voxel.start()
            cost.start()
            go2.start()
            deadline = time.monotonic() + DRAIN_TIMEOUT
            while time.monotonic() < deadline:
                with lock:
                    done = (
                        counts["odom"] >= expected["odom"]
                        and counts["lidar"] >= expected["lidar"]
                        and counts["color_image"] >= expected["color_image"]
                        and counts["global_costmap"] > 0
                    )
                if done:
                    return
                time.sleep(0.05)
            pytest.fail(f"window did not drain: counts={counts}, expected={expected}")

        def teardown() -> None:
            for module in reversed(modules):
                module.stop()

        benchmark.pedantic(start_and_drain, teardown=teardown, rounds=1, warmup_rounds=0)
    finally:
        global_config.update(**saved)
