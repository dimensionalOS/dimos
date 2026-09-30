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
# With DIMOS_BENCH_METRICS: perf events (e.g. "instructions:u,cycles:u") counted over
# the CLI's whole process tree and added to the series. A core's clock and idle
# state don't move them, unlike CPU time. Needs a PMU and perf_event_paranoid <= 2.
PERF_EVENTS = os.environ.get("DIMOS_BENCH_PERF_EVENTS") if METRICS_PATH else None


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


def _cgroup_io_bytes() -> int:
    """Block-device bytes written by this cgroup so far.

    Device-level, not syscall-level, so it reflects actual writeback. Reads
    are deliberately not tracked: served from the page cache they are free,
    so the number only says how cold the runner's cache happened to be.
    """
    written = 0
    for line in (_cgroup_path() / "io.stat").read_text().splitlines():
        fields = dict(part.split("=") for part in line.split()[1:])
        written += int(fields.get("wbytes", 0))
    return written


def _cpu_model() -> str:
    """The CPU model name, so a run's numbers can be attributed to its hardware."""
    lscpu = subprocess.run(
        ["lscpu"], capture_output=True, text=True, check=True, env={**os.environ, "LC_ALL": "C"}
    )
    for line in lscpu.stdout.splitlines():
        key, _, value = line.partition(":")
        if key.strip() == "Model name":
            return value.strip()
    return "unknown"


def _net_bytes() -> int:
    """Bytes the transport moved between the workers so far.

    cgroup v2 has no network accounting, so this is netns-wide — fine on a
    runner where the job is the only real user. Where the volume shows up
    depends on the backend: zenoh's loopback TCP is counted on lo (once, as
    rx), while LCM's ttl=0 UDP multicast is invisible to every interface
    counter (the kernel loops clones to local listeners inside the IP stack —
    not via lo — and nothing reaches a NIC) and only appears as IpExt
    InMcastOctets, which counts each looped datagram once. Their sum covers
    either backend. External interfaces are deliberately not tracked: their
    traffic is the runner agent's own chatter, a fraction of a megabyte that
    swings by half between identical runs.
    """
    lines = [
        line
        for line in Path("/proc/net/netstat").read_text().splitlines()
        if line.startswith("IpExt:")
    ]
    ipext = dict(zip(lines[0].split()[1:], lines[1].split()[1:], strict=True))
    loopback = 0
    for line in Path("/proc/net/dev").read_text().splitlines()[2:]:
        name, _, rest = line.partition(":")
        if name.strip() == "lo":
            loopback += int(rest.split()[0])
    return loopback + int(ipext["InMcastOctets"])


def _perf_counts(path: Path, events: list[str], split_at: float) -> dict[str, tuple[float, float]]:
    """Per event, the count up to `split_at` seconds into the run, and the rest.

    perf's -I mode writes one CSV row per event per interval (time, count,
    unit, event, ...), so the split is as fine as the interval. An interval in
    which nothing ran reads "<not counted>", which is zero; anything else that
    isn't a number means the runner couldn't count the event, and must not
    pass as zero either.
    """
    before: dict[str, float] = {}
    after: dict[str, float] = {}
    for line in path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        time_s, count, _unit, event, *_ = line.split(",")
        if count == "<not counted>":
            continue
        try:
            value = float(count)
        except ValueError:
            raise RuntimeError(f"perf could not count {event}: {count!r}") from None
        bucket = before if float(time_s) <= split_at else after
        bucket[event] = bucket.get(event, 0.0) + value
    missing = [event for event in events if event not in after]
    if missing:
        raise RuntimeError(f"perf reported nothing for {missing} in {path}")
    return {event: (before.get(event, 0.0), after[event]) for event in events}


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
    io_marks: dict[str, int] = {}
    net_marks: dict[str, int] = {}

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
    if PERF_EVENTS:
        # Children inherit the counters, so this covers the workers too. perf's
        # interval mode doesn't pass on the workload's exit status; a shell
        # around the CLI writes it to a file instead.
        perf_csv = Path(METRICS_PATH).with_suffix(".perf.csv")
        rc_path = Path(METRICS_PATH).with_suffix(".rc")
        cmd = [
            *("perf", "stat", "-e", PERF_EVENTS, "-x", ",", "-I", "100", "-o", str(perf_csv)),
            *("--", "sh", "-c", '"$@"; echo $? >"$0"', str(rc_path)),
            *cmd,
        ]
    mark("start")
    if METRICS_PATH:
        sampler.start()
    # Its own process group, so a timeout can signal the CLI through perf.
    proc = subprocess.Popen(cmd, start_new_session=True)
    try:
        try:
            returncode = proc.wait(timeout=RUN_TIMEOUT)
        except subprocess.TimeoutExpired:
            pytest.fail(
                f"dimos did not exit within {RUN_TIMEOUT:.0f}s: counts={counts}, "
                f"expected~{expected}"
            )
        mark("end")
        if PERF_EVENTS and rc_path.exists():
            returncode = int(rc_path.read_text())
    finally:
        stop_sampling.set()
        if sampler.is_alive():
            sampler.join(timeout=5)
        if proc.poll() is None:
            # SIGINT first: the CLI's ctrl-c path stops the modules cleanly.
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=30)
        for transport in transports:
            transport.stop()

    assert returncode == 0, f"dimos exited with {returncode}: counts={counts}"
    low = {name: (counts[name], floors[name]) for name in floors if counts[name] < floors[name]}
    assert not low, f"floors not met (got, floor): {low}, expected~{expected}"

    if METRICS_PATH:
        start, first, end = cpu_marks["start"], cpu_marks["first frame"], cpu_marks["end"]
        entries = [
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
            # Block-device writeback across the run.
            ("disk write", (io_marks["end"] - io_marks["start"]) / 2**20, "MB"),
            # Bytes between the workers: loopback for zenoh, looped multicast
            # for LCM.
            ("network (transport)", (net_marks["end"] - net_marks["start"]) / 2**20, "MB"),
        ]
        if PERF_EVENTS:
            # Split at the first frame like the CPU time. "instructions:u" -> "instructions".
            counted = _perf_counts(perf_csv, PERF_EVENTS.split(","), first[0] - start[0])
            for event, (startup, run) in counted.items():
                name = event.partition(":")[0]
                entries += [
                    (f"first frame {name}", startup / 1e9, "G"),
                    (f"run {name}", run / 1e9, "G"),
                ]
        extra = f"cpu: {_cpu_model()}"
        Path(METRICS_PATH).write_text(
            json.dumps(
                [
                    {"name": name, "unit": unit, "value": round(value, 3), "extra": extra}
                    for name, value, unit in entries
                ],
                indent=2,
            )
        )
