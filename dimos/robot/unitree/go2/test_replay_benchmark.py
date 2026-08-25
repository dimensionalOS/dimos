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
import signal
import subprocess
import sys
import threading
import time

import pytest

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

REPLAY_DB = os.environ.get("DIMOS_BENCH_REPLAY_DB", "go2_short")
RUN_TIMEOUT = 420.0  # spawn -> exit deadline, inside the test timeout
# GO2Connection source streams. camera_info is deliberately absent: it is
# published by a 1 Hz forever-loop thread, so it never quiesces.
WATCHED = (("odom", PoseStamped), ("lidar", PointCloud2), ("color_image", Image))
# Validity floors double as the realtime keep-up check. odom is small and
# near-lossless; lidar and color_image are large frames whose delivery relies
# on the 64MB rmem tuning, so leave headroom for designed shedding.
FLOOR_FRACTION = {"odom": 0.9, "lidar": 0.9, "color_image": 0.5}
# When set, write the tracked series (wall/CPU/memory/threads/disk/network) to this path.
METRICS_PATH = os.environ.get("DIMOS_BENCH_METRICS")


def _expected_counts(db_path: str) -> dict[str, int]:
    """Per-stream message totals straight from the DB.

    Mirrors ReplayConnection's stream-name fallback (mid360-era recordings use
    go2_lidar/go2_odom, older ones lidar/odom).
    """
    from dimos.memory.store.sqlite import SqliteStore

    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    try:
        available = store.list_streams()

        def first_present(*names: str) -> str:
            for name in names:
                if name in available:
                    return name
            raise KeyError(f"none of {names!r} in {db_path!r}; available: {available}")

        return {
            "odom": store.stream(first_present("go2_odom", "odom")).count(),
            "lidar": store.stream(first_present("go2_lidar", "lidar")).count(),
            "color_image": store.stream("color_image").count(),
        }
    finally:
        store.stop()


def _cgroup_path() -> Path:
    lines = Path("/proc/self/cgroup").read_text().splitlines()
    v2 = next((line for line in lines if line.startswith("0::")), None)
    if v2 is None:
        raise RuntimeError(f"cgroup v2 required for benchmark accounting, got: {lines}")
    return Path("/sys/fs/cgroup", v2.removeprefix("0::").strip().lstrip("/"))


def _cgroup_stat(name: str) -> dict[str, str]:
    lines = (_cgroup_path() / name).read_text().splitlines()
    return dict(line.split() for line in lines)


def _cpu_mark() -> tuple[float, float, float]:
    """(wall, user, system): monotonic seconds and this cgroup's CPU seconds.

    Cgroup accounting counts every process in the job's cgroup, live or
    exited — per-process rusage can't: the forkserver workers doing most of
    the work are never reaped by the test process, so RUSAGE_CHILDREN misses
    them.
    """
    fields = _cgroup_stat("cpu.stat")
    return time.monotonic(), int(fields["user_usec"]) / 1e6, int(fields["system_usec"]) / 1e6


def _cgroup_anon_bytes() -> int:
    """Anonymous memory currently charged to this cgroup, whole process tree.

    Page cache is deliberately excluded (memory.current would include it): it
    scales with file reads and global memory pressure, not with the pipeline.
    memory.peak is no use either — it is cumulative since cgroup creation, so
    on a CI runner it would report the job's setup steps, not the benchmark.
    """
    return int(_cgroup_stat("memory.stat")["anon"])


def _cgroup_tasks() -> int:
    """Threads currently in this cgroup, whole process tree.

    The pids controller charges every task, so a single-threaded process
    counts as 1.
    """
    return int((_cgroup_path() / "pids.current").read_text())


def _cgroup_io_bytes() -> tuple[int, int]:
    """(read, written) block-device bytes charged to this cgroup so far.

    Device-level, not syscall-level: reads served from the page cache are
    free, so with the DB pre-extracted (and therefore cache-warm) reads
    mostly reflect cold imports, and writes reflect actual writeback.
    """
    read = written = 0
    for line in (_cgroup_path() / "io.stat").read_text().splitlines():
        fields = dict(part.split("=") for part in line.split()[1:])
        read += int(fields.get("rbytes", 0))
        written += int(fields.get("wbytes", 0))
    return read, written


def _net_bytes() -> tuple[int, int, int]:
    """(multicast, external rx, external tx) byte counters.

    cgroup v2 has no network accounting, so these are netns-wide — fine on a
    runner where the job is the only real user. LCM's ttl=0 UDP multicast is
    invisible to every interface counter (the kernel loops clones to local
    listeners inside the IP stack — not via lo — and nothing reaches a NIC),
    so the transport volume between the workers comes from IpExt
    InMcastOctets, which counts each looped datagram once. External
    interfaces should stay ~flat across the run: growth means something
    inside the measured region talks to the network.
    """
    lines = [
        line
        for line in Path("/proc/net/netstat").read_text().splitlines()
        if line.startswith("IpExt:")
    ]
    ipext = dict(zip(lines[0].split()[1:], lines[1].split()[1:], strict=True))
    ext_rx = ext_tx = 0
    for line in Path("/proc/net/dev").read_text().splitlines()[2:]:
        name, _, rest = line.partition(":")
        fields = rest.split()
        if name.strip() != "lo":
            ext_rx += int(fields[0])
            ext_tx += int(fields[8])
    return int(ipext["InMcastOctets"]), ext_rx, ext_tx


@pytest.mark.self_hosted_large  # Needs 8+ GB memory
# macOS: coordinator->worker zenoh RPC times out (set_transport), and the
# in-test LCM subscriptions would need lo0 route + maxdgram host tuning.
@pytest.mark.skipif_macos_bug
@pytest.mark.timeout(900)
def test_go2_replay_realtime_load() -> None:
    """Run `dimos --replay --replay-exit run unitree-go2` to completion."""
    from dimos.memory.replay import resolve_db_path

    db_path = str(resolve_db_path(REPLAY_DB))  # LFS pull/extract on miss
    expected = _expected_counts(db_path)
    assert all(count > 0 for count in expected.values()), f"empty recording: {expected}"
    floors = {name: int(expected[name] * fraction) for name, fraction in FLOOR_FRACTION.items()}

    counts = dict.fromkeys(FLOOR_FRACTION, 0)
    lock = threading.Lock()
    cpu_marks: dict[str, tuple[float, float, float]] = {}
    io_marks: dict[str, tuple[int, int]] = {}
    net_marks: dict[str, tuple[int, int, int]] = {}

    def mark(name: str) -> None:
        if METRICS_PATH:
            cpu_marks[name] = _cpu_mark()
            io_marks[name] = _cgroup_io_bytes()
            net_marks[name] = _net_bytes()

    def record(name: str) -> None:
        with lock:
            if not any(counts.values()):
                mark("first frame")
            counts[name] += 1

    # Same topics and backend the blueprint materializes for these
    # name-unique streams. Subscribe before spawning: replay data flows as
    # soon as GO2Connection starts, mid-build.
    transports = [make_transport(name, typ) for name, typ in WATCHED]
    for (name, _), transport in zip(WATCHED, transports, strict=True):
        transport.subscribe(lambda _msg, _name=name: record(_name))

    peak_anon = 0
    peak_tasks = 0
    stop_sampling = threading.Event()

    def sample_peaks() -> None:
        nonlocal peak_anon, peak_tasks
        while not stop_sampling.wait(0.1):
            peak_anon = max(peak_anon, _cgroup_anon_bytes())
            peak_tasks = max(peak_tasks, _cgroup_tasks())

    sampler = threading.Thread(target=sample_peaks, daemon=True)

    # The venv's console script: the CLI entrypoint, not an in-process build,
    # so the benchmark measures what `dimos run` users get. --replay-exit
    # makes the process exit once the recording finishes.
    cmd = [
        str(Path(sys.executable).with_name("dimos")),
        "--replay",
        "--replay-db",
        REPLAY_DB,
        "--replay-exit",
        "--viewer",
        "none",
        "run",
        "unitree-go2",
    ]
    mark("start")
    if METRICS_PATH:
        sampler.start()
    proc = subprocess.Popen(cmd)
    try:
        try:
            returncode = proc.wait(timeout=RUN_TIMEOUT)
        except subprocess.TimeoutExpired:
            pytest.fail(
                f"dimos did not exit within {RUN_TIMEOUT:.0f}s: counts={counts}, "
                f"expected~{expected}"
            )
        mark("end")
    finally:
        stop_sampling.set()
        if sampler.is_alive():
            sampler.join(timeout=5)
        if proc.poll() is None:
            # SIGINT first: the CLI's ctrl-c path stops the modules cleanly.
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=30)
        for transport in transports:
            transport.stop()

    assert returncode == 0, f"dimos exited with {returncode}: counts={counts}"
    low = {name: (counts[name], floors[name]) for name in floors if counts[name] < floors[name]}
    assert not low, f"floors not met (got, floor): {low}, expected~{expected}"

    if METRICS_PATH:
        start, first, end = cpu_marks["start"], cpu_marks["first frame"], cpu_marks["end"]
        entries = (
            # Startup: process spawn until the first frame reaches the bus.
            ("first frame wall", first[0] - start[0], "s"),
            ("first frame cpu", (first[1] + first[2]) - (start[1] + start[2]), "s"),
            # Steady-state cost of the realtime run — the headline.
            ("run cpu", (end[1] + end[2]) - (first[1] + first[2]), "s"),
            ("run cpu (user)", end[1] - first[1], "s"),
            ("run cpu (system)", end[2] - first[2], "s"),
            # Maxima sampled at 10Hz across the run, whole process tree.
            ("peak memory", peak_anon / 2**20, "MB"),
            ("peak threads", float(peak_tasks), "threads"),
            # Block-device totals; page-cache hits are free.
            ("disk read", (io_marks["end"][0] - io_marks["start"][0]) / 2**20, "MB"),
            ("disk write", (io_marks["end"][1] - io_marks["start"][1]) / 2**20, "MB"),
            # Multicast = LCM transport volume; external ~0 unless something
            # in the run talks to the network.
            (
                "network (multicast)",
                (net_marks["end"][0] - net_marks["start"][0]) / 2**20,
                "MB",
            ),
            (
                "network (external rx)",
                (net_marks["end"][1] - net_marks["start"][1]) / 2**20,
                "MB",
            ),
            (
                "network (external tx)",
                (net_marks["end"][2] - net_marks["start"][2]) / 2**20,
                "MB",
            ),
        )
        Path(METRICS_PATH).write_text(
            json.dumps(
                [
                    {"name": name, "unit": unit, "value": round(value, 3)}
                    for name, value, unit in entries
                ],
                indent=2,
            )
        )
