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

"""Realtime-load benchmark of the unitree-go2 blueprint under replay."""

import json
import os
from pathlib import Path
import threading
import time

import pytest

from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

REPLAY_DB = os.environ.get("DIMOS_BENCH_REPLAY_DB", "go2_hongkong_office")
DURATION = float(os.environ.get("DIMOS_BENCH_DURATION", 60))
QUIET_S = 3.0  # all watched streams silent this long => window complete
RUN_TIMEOUT = DURATION + 360.0  # build+run deadline, inside the test timeout
# GO2Connection source streams. camera_info is deliberately absent: it is
# published by a 1 Hz forever-loop thread, so it never quiesces.
WATCHED = (("odom", PoseStamped), ("lidar", PointCloud2), ("color_image", Image))
# Validity floors double as the realtime keep-up check. odom is small and
# near-lossless; lidar and color_image are large frames whose delivery relies
# on the 64MB rmem tuning, so leave headroom for designed shedding.
FLOOR_FRACTION = {"odom": 0.9, "lidar": 0.9, "color_image": 0.5}
# When set, write build wall/CPU and run-window CPU seconds to this path.
CPU_METRICS_PATH = os.environ.get("DIMOS_BENCH_CPU_METRICS")


def _expected_counts(db_path: str, duration: float = DURATION) -> dict[str, int]:
    """Windowed per-stream counts straight from the DB.

    Mirrors ReplayConnection's stream-name fallback (mid360-era recordings use
    go2_lidar/go2_odom, older ones lidar/odom).
    """
    from dimos.memory.store.sqlite import SqliteStore

    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    try:
        replay = store.replay(duration=duration)
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


def _cpu_mark() -> tuple[float, float, float]:
    """(wall, user, system): monotonic seconds and this cgroup's CPU seconds.

    Cgroup accounting counts every process in the job's cgroup, live or
    exited — per-process rusage can't: the forkserver workers doing most of
    the work are never reaped by the test process, so RUSAGE_CHILDREN misses
    them.
    """
    lines = Path("/proc/self/cgroup").read_text().splitlines()
    v2 = next((line for line in lines if line.startswith("0::")), None)
    if v2 is None:
        raise RuntimeError(f"cgroup v2 required for CPU accounting, got: {lines}")
    rel = v2.removeprefix("0::").strip().lstrip("/")
    stat = Path("/sys/fs/cgroup", rel, "cpu.stat").read_text()
    fields = dict(line.split() for line in stat.splitlines())
    return time.monotonic(), int(fields["user_usec"]) / 1e6, int(fields["system_usec"]) / 1e6


@pytest.mark.self_hosted
# macOS: coordinator->worker zenoh RPC times out (set_transport), and the
# in-test LCM subscriptions would need lo0 route + maxdgram host tuning.
@pytest.mark.skipif_macos_bug
@pytest.mark.timeout(900)
def test_go2_replay_realtime_load() -> None:
    """Build the unitree-go2 blueprint and run a replay window at realtime pace."""
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.memory.replay import resolve_db_path
    from dimos.robot.get_all_blueprints import get_blueprint_by_name

    # Configure before resolving the blueprint: unitree_go2_basic composes its
    # vis bundle from global_config.viewer at import time. build() also writes
    # the blueprint's own overrides into global_config, so restore everything.
    saved = global_config.model_dump()
    global_config.update(
        replay=True,
        replay_db=REPLAY_DB,
        replay_duration=DURATION,
        viewer="none",
    )
    try:
        db_path = str(resolve_db_path(REPLAY_DB))  # LFS pull/extract on miss
        expected = _expected_counts(db_path)
        assert all(count > 0 for count in expected.values()), f"empty window: {expected}"
        floors = {name: int(expected[name] * fraction) for name, fraction in FLOOR_FRACTION.items()}

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
        cpu_marks: dict[str, tuple[float, float, float]] = {}

        def build_and_run() -> None:
            nonlocal last_arrival
            if CPU_METRICS_PATH:
                cpu_marks["start"] = _cpu_mark()
            state["coordinator"] = ModuleCoordinator.build(blueprint)
            if CPU_METRICS_PATH:
                cpu_marks["built"] = _cpu_mark()
            with lock:
                last_arrival = time.monotonic()
            deadline = time.monotonic() + RUN_TIMEOUT
            while time.monotonic() < deadline:
                with lock:
                    quiet = time.monotonic() - last_arrival
                    done = all(counts[name] >= floors[name] for name in counts)
                if done and quiet >= QUIET_S:
                    if CPU_METRICS_PATH:
                        cpu_marks["end"] = _cpu_mark()
                    return
                time.sleep(0.2)
            pytest.fail(f"window did not complete: counts={counts}, expected~{expected}")

        def teardown() -> None:
            for transport in transports:
                transport.stop()
            coordinator = state.pop("coordinator", None)
            if coordinator is not None:
                # coordinator.stop() can hang on worker teardown; a hard exit
                # (tool_replay_bench style) would lose the metrics upload.
                stopper = threading.Thread(target=coordinator.stop, daemon=True)
                stopper.start()
                stopper.join(timeout=90)
                if stopper.is_alive():
                    pytest.fail("coordinator.stop() hung")

        try:
            build_and_run()
        finally:
            teardown()

        if CPU_METRICS_PATH:
            start, built, end = cpu_marks["start"], cpu_marks["built"], cpu_marks["end"]
            series = {
                # Startup cost: worker spawn + imports + transport setup.
                "build wall": built[0] - start[0],
                "build cpu": (built[1] + built[2]) - (start[1] + start[2]),
                # Steady-state cost of the realtime window — the headline.
                "run cpu": (end[1] + end[2]) - (built[1] + built[2]),
                "run cpu (user)": end[1] - built[1],
                "run cpu (system)": end[2] - built[2],
            }
            Path(CPU_METRICS_PATH).write_text(
                json.dumps(
                    [
                        {"name": name, "unit": "s", "value": round(value, 3)}
                        for name, value in series.items()
                    ],
                    indent=2,
                )
            )
    finally:
        global_config.update(**saved)
