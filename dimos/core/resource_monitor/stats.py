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

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TypedDict

import psutil

from dimos.utils.decorators.decorators import ttl_cache

# Cache Process objects so cpu_percent(interval=None) has a previous sample.
_proc_cache: dict[int, psutil.Process] = {}


@dataclass(frozen=True)
class ProcessStats:
    """Resource stats for a single OS process."""

    pid: int
    alive: bool
    cpu_percent: float = 0.0
    cpu_time_user: float = 0.0
    cpu_time_system: float = 0.0
    cpu_time_iowait: float = 0.0
    pss: int = 0
    num_threads: int = 0
    num_children: int = 0
    num_fds: int = 0
    io_read_bytes: int = 0
    io_write_bytes: int = 0


def _get_process(pid: int) -> psutil.Process:
    """Return a cached Process object, creating a new one if missing or dead."""
    proc = _proc_cache.get(pid)
    if proc is None or not proc.is_running():
        proc = psutil.Process(pid)
        _proc_cache[pid] = proc
    return proc


class CpuStats(TypedDict):
    cpu_percent: float
    cpu_time_user: float
    cpu_time_system: float
    cpu_time_iowait: float


def _collect_cpu(proc: psutil.Process) -> CpuStats:
    """Collect CPU metrics. Call inside oneshot()."""
    cpu_pct = proc.cpu_percent(interval=None)
    ct = proc.cpu_times()
    return CpuStats(
        cpu_percent=cpu_pct,
        cpu_time_user=ct.user,
        cpu_time_system=ct.system,
        cpu_time_iowait=getattr(ct, "iowait", 0.0),
    )


@ttl_cache(4.0)
def _collect_pss(pid: int) -> int:
    """Collect PSS memory in bytes. TTL-cached to avoid expensive smaps reads."""
    try:
        proc = _get_process(pid)
        mem_full = proc.memory_full_info()
        return getattr(mem_full, "pss", 0)
    except (psutil.AccessDenied, psutil.NoSuchProcess, AttributeError):
        return 0


class IoStats(TypedDict):
    io_read_bytes: int
    io_write_bytes: int


def _collect_io(proc: psutil.Process) -> IoStats:
    """Collect IO counters in bytes. Call inside oneshot()."""
    try:
        io = proc.io_counters()  # type: ignore[attr-defined]  # not available on macOS
        return IoStats(io_read_bytes=io.read_bytes, io_write_bytes=io.write_bytes)
    except (psutil.AccessDenied, AttributeError):
        return IoStats(io_read_bytes=0, io_write_bytes=0)


class ProcStats(TypedDict):
    num_threads: int
    num_children: int
    num_fds: int


def _collect_proc(proc: psutil.Process) -> ProcStats:
    """Collect thread/children/fd counts. Call inside oneshot()."""
    try:
        fds = proc.num_fds()
    except (psutil.AccessDenied, AttributeError):
        fds = 0
    return ProcStats(
        num_threads=proc.num_threads(),
        num_children=len(proc.children(recursive=True)),
        num_fds=fds,
    )


@dataclass(frozen=True)
class ChildProcessStats:
    """CPU stats for a single child process."""

    pid: int
    name: str
    cpu_percent: float


def collect_children_stats(pid: int) -> list[ChildProcessStats]:
    """Return per-child CPU stats for all direct children of pid."""
    result: list[ChildProcessStats] = []
    try:
        proc = _get_process(pid)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return result
    for child in proc.children(recursive=False):
        try:
            child_proc = _get_process(child.pid)
            name = child_proc.name()
            cpu = child_proc.cpu_percent(interval=None)
            result.append(ChildProcessStats(pid=child.pid, name=name, cpu_percent=cpu))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return result


def collect_process_stats(pid: int) -> ProcessStats:
    """Collect resource stats for a single process by PID."""
    try:
        proc = _get_process(pid)
        with proc.oneshot():
            cpu = _collect_cpu(proc)
            io = _collect_io(proc)
            proc_stats = _collect_proc(proc)
        pss = _collect_pss(pid)
        return ProcessStats(pid=pid, alive=True, pss=pss, **cpu, **io, **proc_stats)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        _proc_cache.pop(pid, None)
        _collect_pss.cache.pop((pid,), None)
        return ProcessStats(pid=pid, alive=False)


@dataclass(frozen=True)
class WorkerStats(ProcessStats):
    """Process stats extended with worker-specific metadata."""

    worker_id: int = -1
    modules: list[str] = field(default_factory=list)
    children: list[ChildProcessStats] = field(default_factory=list)
