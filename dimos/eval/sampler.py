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

from collections import defaultdict
import re
import shutil
import subprocess
import threading

from dimos.core.resource_monitor.stats import ProcessStats, WorkerStats
from dimos.eval.metrics import GpuFootprint, WorkerFootprint

ResourceSample = tuple[ProcessStats, list[WorkerStats]]


class InMemoryResourceLogger:
    """A ResourceLogger that accumulates samples instead of publishing them.
    """

    def __init__(self) -> None:
        self._samples: list[ResourceSample] = []
        self._lock = threading.Lock()

    def log_stats(self, coordinator: ProcessStats, workers: list[WorkerStats]) -> None:
        with self._lock:
            self._samples.append((coordinator, list(workers)))

    def stop(self) -> None:
        pass

    def reset(self) -> None:
        """Drop everything collected so far (used to discard warmup samples)."""
        with self._lock:
            self._samples.clear()

    @property
    def samples(self) -> list[ResourceSample]:
        with self._lock:
            return list(self._samples)


class GpuSampler:
    """Poll system-wide GPU utilization in a background thread."""

    def __init__(self, interval: float = 1.0) -> None:
        self._interval = interval
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._samples: dict[int, list[tuple[float, int, int]]] = defaultdict(list)
        self._names: dict[int, str] = {}
        self._mode = _gpu_query_mode()

    @property
    def available(self) -> bool:
        return self._mode is not None

    def start(self) -> None:
        if self._mode is None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def reset(self) -> None:
        with self._lock:
            self._samples.clear()

    def _loop(self) -> None:
        while not self._stop.wait(self._interval):
            try:
                self._sample_once()
            except (OSError, subprocess.SubprocessError, ValueError):
                pass

    def _sample_once(self) -> None:
        if self._mode == "nvidia-smi":
            reading = _sample_nvidia_smi()
        else:
            reading = _sample_tegrastats()
        with self._lock:
            for index, name, util, mem_used, mem_total in reading:
                self._samples[index].append((util, mem_used, mem_total))
                self._names[index] = name

    def footprints(self) -> list[GpuFootprint]:
        with self._lock:
            result: list[GpuFootprint] = []
            for index in sorted(self._samples):
                rows = self._samples[index]
                if not rows:
                    continue
                utils = [r[0] for r in rows]
                mems = [r[1] for r in rows]
                result.append(
                    GpuFootprint(
                        index=index,
                        name=self._names.get(index, f"gpu{index}"),
                        util_percent_mean=sum(utils) / len(utils),
                        util_percent_peak=max(utils),
                        mem_used_mean_bytes=int(sum(mems) / len(mems)),
                        mem_used_peak_bytes=max(mems),
                        mem_total_bytes=rows[-1][2],
                        samples=len(rows),
                    )
                )
            return result


def _gpu_query_mode() -> str | None:
    if shutil.which("nvidia-smi") is not None:
        return "nvidia-smi"
    if shutil.which("tegrastats") is not None:
        return "tegrastats"
    return None


def _sample_nvidia_smi() -> list[tuple[int, str, float, int, int]]:
    out = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=index,name,utilization.gpu,memory.used,memory.total",
            "--format=csv,noheader,nounits",
        ],
        text=True,
        timeout=10,
    )
    readings: list[tuple[int, str, float, int, int]] = []
    for line in out.strip().splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 5:
            continue
        index, name, util, used_mib, total_mib = parts
        readings.append(
            (
                int(index),
                name,
                float(util),
                int(float(used_mib)) * 1024 * 1024,
                int(float(total_mib)) * 1024 * 1024,
            )
        )
    return readings


def _sample_tegrastats() -> list[tuple[int, str, float, int, int]]:
    """Single-shot tegrastats read for Jetson boards without nvidia-smi."""
    proc = subprocess.Popen(
        ["tegrastats", "--interval", "500"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    try:
        assert proc.stdout is not None
        line = proc.stdout.readline()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    util = 0.0
    m = re.search(r"GR3D_FREQ\s+(\d+)%", line)
    if m:
        util = float(m.group(1))
    mem_used = mem_total = 0
    m = re.search(r"RAM\s+(\d+)/(\d+)MB", line)
    if m:
        mem_used = int(m.group(1)) * 1024 * 1024
        mem_total = int(m.group(2)) * 1024 * 1024
    return [(0, "Jetson iGPU", util, mem_used, mem_total)]


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def aggregate_worker(worker_id: int, modules: list[str], dedicated: bool,
                     stats: list[ProcessStats]) -> WorkerFootprint:
    """Collapse a process's samples into a single WorkerFootprint."""
    alive = [s for s in stats if s.alive] or stats
    cpu = [s.cpu_percent for s in alive]
    pss = [s.pss for s in alive]
    last = alive[-1]
    return WorkerFootprint(
        worker_id=worker_id,
        modules=modules,
        dedicated=dedicated,
        cpu_percent_mean=_mean(cpu),
        cpu_percent_peak=max(cpu) if cpu else 0.0,
        pss_mean_bytes=int(_mean([float(p) for p in pss])),
        pss_peak_bytes=max(pss) if pss else 0,
        num_threads=last.num_threads,
        num_children=last.num_children,
        num_fds=last.num_fds,
        io_read_bytes=last.io_read_bytes,
        io_write_bytes=last.io_write_bytes,
        samples=len(stats),
    )


def aggregate_totals(samples: list[ResourceSample]) -> tuple[float, int, int]:
    
    if not samples:
        return 0.0, 0, 0
    cpu_series: list[float] = []
    pss_series: list[int] = []
    for coord, workers in samples:
        cpu = coord.cpu_percent + sum(w.cpu_percent for w in workers if w.alive)
        pss = coord.pss + sum(w.pss for w in workers if w.alive)
        cpu_series.append(cpu)
        pss_series.append(pss)
    pss_mean = int(sum(pss_series) / len(pss_series))
    return _mean(cpu_series), pss_mean, max(pss_series)


def aggregate_resources(
    samples: list[ResourceSample],
) -> tuple[WorkerFootprint | None, list[WorkerFootprint]]:
    """Turn raw resource samples into (coordinator, per-worker) footprints."""
    if not samples:
        return None, []

    coord_stats = [coord for coord, _ in samples]
    coordinator = aggregate_worker(-1, [], False, coord_stats)

    by_worker: dict[int, list[WorkerStats]] = defaultdict(list)
    modules: dict[int, list[str]] = {}
    dedicated: dict[int, bool] = {}
    for _, workers in samples:
        for w in workers:
            by_worker[w.worker_id].append(w)
            modules[w.worker_id] = w.modules
            dedicated[w.worker_id] = w.dedicated

    footprints = [
        aggregate_worker(wid, modules[wid], dedicated[wid], list(stats))
        for wid, stats in sorted(by_worker.items())
    ]
    return coordinator, footprints
