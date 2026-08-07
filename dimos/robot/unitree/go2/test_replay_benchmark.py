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

"""CodSpeed walltime benchmark of the unitree-go2 blueprint under replay.

The equivalent of `dimos --replay --replay-db go2_hongkong_office run
unitree-go2`, bounded: build the coordinator, drain the first ~60s of the
recording as fast as the pipeline allows (large --replay-speed collapses the
wall-clock pacing), stop. The measured region is build+drain; replay starts
inside ModuleCoordinator.build() (GO2Connection.start() subscribes the replay
streams), so the two can't be separated without changing the blueprint.

Drain completion is detected by quiescence — no arrival on any watched source
topic for QUIET_S — because exact delivery counting is unreliable by design
(zenoh gives Image/PointCloud2 topics latest-wins QoS; LCM UDP drops fragments
under flood). Windowed per-stream counts read from the database beforehand act
as validity floors so a silently dead stream fails the run instead of producing
a fast-but-meaningless sample.

self_hosted (LFS data, ~10 worker processes): the self-hosted CI job runs it
as a plain test — the `benchmark` fixture just calls the function once — which
keeps the path in coverage; .github/workflows/codspeed.yml runs the same test
under `pytest --codspeed` for the measured walltime sample. Linux-only for
now (skipif_macos_bug): local macOS runs die on the coordinator->worker zenoh
RPC timeout, and the LCM fallback needs lo0 route + maxdgram tuning. Linux
smoke against the small bundled recording:

    DIMOS_BENCH_REPLAY_DB=go2_short DIMOS_BENCH_DURATION=10 \
        uv run pytest dimos/robot/unitree/go2/test_replay_benchmark.py \
        -m self_hosted --no-cov -v
"""

from __future__ import annotations

import os
import threading
import time
from typing import TYPE_CHECKING

import pytest

from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from pytest_codspeed import BenchmarkFixture

REPLAY_DB = os.environ.get("DIMOS_BENCH_REPLAY_DB", "go2_hongkong_office")
DURATION = float(os.environ.get("DIMOS_BENCH_DURATION", "60"))
SPEED = 1000.0  # every emission delay clamps to 0: a pure CPU-bound drain
QUIET_S = 3.0  # all watched streams silent this long => drained
DRAIN_TIMEOUT = 420.0  # build+drain deadline, inside the test timeout
# GO2Connection source streams. camera_info is deliberately absent: it is
# published by a 1 Hz forever-loop thread and never quiesces.
WATCHED = (("odom", PoseStamped), ("lidar", PointCloud2), ("color_image", Image))
# Validity gates, not the timing edge: odom is small and near-lossless; the
# heavy latest-wins streams legitimately drop under the flood.
FLOOR_FRACTION = {"odom": 0.9, "lidar": 0.5, "color_image": 0.5}


def _expected_counts(db_path: str) -> dict[str, int]:
    """Windowed per-stream counts straight from the DB.

    Mirrors ReplayConnection's stream-name fallback (mid360-era recordings use
    go2_lidar/go2_odom, older ones lidar/odom).
    """
    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    try:
        replay = store.replay(duration=DURATION)
        available = replay.list_streams()

        def first_present(*names: str) -> str:
            for name in names:
                if name in available:
                    return name
            raise KeyError(f"none of {names!r} in {db_path!r}; available: {available}")

        return {
            "odom": replay.stream(first_present("go2_odom", "odom")).count(),
            "lidar": replay.stream(first_present("go2_lidar", "lidar")).count(),
            "color_image": replay.stream("color_image").count(),
        }
    finally:
        store.stop()


@pytest.mark.self_hosted
# macOS: coordinator->worker zenoh RPC times out (set_transport), and the
# in-test LCM subscriptions would need lo0 route + maxdgram host tuning.
@pytest.mark.skipif_macos_bug
@pytest.mark.timeout(900)
def test_go2_replay_drain_walltime(benchmark: BenchmarkFixture) -> None:
    """Build the unitree-go2 blueprint and drain a replay window at max speed."""
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.memory2.replay import resolve_db_path
    from dimos.robot.get_all_blueprints import get_blueprint_by_name

    # Configure before resolving the blueprint: unitree_go2_basic composes its
    # vis bundle from global_config.viewer at import time. build() also writes
    # the blueprint's own overrides into global_config, so restore everything.
    saved = global_config.model_dump()
    global_config.update(
        replay=True,
        replay_db=REPLAY_DB,
        replay_speed=SPEED,
        replay_duration=DURATION,
        viewer="none",
    )
    try:
        db_path = str(resolve_db_path(REPLAY_DB))  # LFS pull/extract on miss
        expected = _expected_counts(db_path)
        assert all(count > 0 for count in expected.values()), f"empty window: {expected}"
        floors = {name: int(count * FLOOR_FRACTION[name]) for name, count in expected.items()}

        blueprint = get_blueprint_by_name("unitree-go2")

        counts = dict.fromkeys(FLOOR_FRACTION, 0)
        last_arrival = time.monotonic()
        lock = threading.Lock()

        def record(name: str) -> None:
            nonlocal last_arrival
            with lock:
                counts[name] += 1
                last_arrival = time.monotonic()

        # Same topics and backend the blueprint materializes for these
        # name-unique streams. Subscribe before build: replay data flows as
        # soon as GO2Connection starts, mid-build.
        transports = [make_transport(name, typ) for name, typ in WATCHED]
        for (name, _), transport in zip(WATCHED, transports, strict=True):
            transport.subscribe(lambda _msg, _name=name: record(_name))

        state: dict[str, ModuleCoordinator] = {}

        def build_and_drain() -> None:
            nonlocal last_arrival
            state["coordinator"] = ModuleCoordinator.build(blueprint)
            with lock:
                last_arrival = time.monotonic()
            deadline = time.monotonic() + DRAIN_TIMEOUT
            while time.monotonic() < deadline:
                with lock:
                    quiet = time.monotonic() - last_arrival
                    done = all(counts[name] >= floors[name] for name in counts)
                if done and quiet >= QUIET_S:
                    return
                time.sleep(0.2)
            pytest.fail(f"window did not drain: counts={counts}, expected~{expected}")

        def teardown() -> None:
            for transport in transports:
                transport.stop()
            coordinator = state.pop("coordinator", None)
            if coordinator is not None:
                # coordinator.stop() can hang on worker teardown; a hard exit
                # (tool_replay_bench style) would lose the benchmark upload.
                stopper = threading.Thread(target=coordinator.stop, daemon=True)
                stopper.start()
                stopper.join(timeout=90)
                if stopper.is_alive():
                    pytest.fail("coordinator.stop() hung")

        benchmark.pedantic(build_and_drain, teardown=teardown, rounds=1, warmup_rounds=0)
    finally:
        global_config.update(**saved)
