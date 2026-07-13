from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import asdict
import math
import statistics
import threading
import time

import numpy as np
import psutil

from dimos.core.resource_monitor.monitor import StatsMonitor, WorkerInfo
from dimos.mapping.pointclouds.occupancy import HeightCostConfig, height_cost_occupancy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class _BenchmarkProcess:
    @property
    def workers(self) -> tuple[WorkerInfo, ...]:
        return ()


class _ResourceProfiler:
    def __init__(self, interval: float = 0.05) -> None:
        self.interval = interval
        self.process = psutil.Process()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.cpu_samples: list[float] = []
        self.rss_samples: list[int] = []
        self.pss_samples: list[int] = []
        self.wall_seconds = 0.0
        self.cpu_seconds = 0.0

    def _sample(self) -> None:
        self.cpu_samples.append(self.process.cpu_percent(interval=None))
        self.rss_samples.append(self.process.memory_info().rss)
        pss = getattr(self.process.memory_full_info(), "pss", None)
        if pss is not None:
            self.pss_samples.append(pss)

    def _sample_loop(self) -> None:
        while not self.stop_event.wait(self.interval):
            self._sample()

    def __enter__(self) -> _ResourceProfiler:
        self.process.cpu_percent(interval=None)
        self._sample()
        self.cpu_start = self.process.cpu_times()
        self.wall_start = time.perf_counter()
        self.thread = threading.Thread(target=self._sample_loop, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
        self._sample()
        self.wall_seconds = time.perf_counter() - self.wall_start
        cpu_end = self.process.cpu_times()
        self.cpu_seconds = (cpu_end.user - self.cpu_start.user) + (
            cpu_end.system - self.cpu_start.system
        )

    def print_summary(self) -> None:
        mib = 1024 * 1024
        average_cores = self.cpu_seconds / self.wall_seconds
        print(f"CPU average: {average_cores:.2f} cores ({average_cores * 100:.1f}%)")
        print(f"CPU peak sampled: {max(self.cpu_samples):.1f}%")
        print(
            f"RSS: {self.rss_samples[0] / mib:.1f} MiB baseline, "
            f"{max(self.rss_samples) / mib:.1f} MiB peak, "
            f"{(max(self.rss_samples) - self.rss_samples[0]) / mib:+.1f} MiB delta"
        )
        if self.pss_samples:
            print(
                f"PSS: {self.pss_samples[0] / mib:.1f} MiB baseline, "
                f"{max(self.pss_samples) / mib:.1f} MiB peak, "
                f"{(max(self.pss_samples) - self.pss_samples[0]) / mib:+.1f} MiB delta"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark the Python costmapper baseline.")
    parser.add_argument("--points", type=int, default=100_000)
    parser.add_argument("--iterations", type=int)
    parser.add_argument("--warmups", type=int, default=2)
    parser.add_argument(
        "--dtop",
        action="store_true",
        help="Publish this process's resource stats for the dtop TUI.",
    )
    parser.add_argument(
        "--profile",
        action="store_true",
        help="Report CPU and memory load sampled during timed iterations.",
    )
    args = parser.parse_args()
    profile_resources = args.profile or args.dtop
    iterations = args.iterations or (1_000 if profile_resources else 10)

    rng = np.random.default_rng(0)
    xy = rng.uniform(-10.0, 10.0, size=(args.points, 2)).astype(np.float32)
    z = (0.05 * np.sin(xy[:, 0]) + 0.05 * np.cos(xy[:, 1])).astype(np.float32)
    cloud = PointCloud2.from_numpy(np.column_stack((xy, z)), frame_id="world", timestamp=1.0)
    config = asdict(HeightCostConfig())

    monitor = StatsMonitor(_BenchmarkProcess(), interval=0.25) if args.dtop else nullcontext()
    with monitor:
        for _ in range(args.warmups):
            height_cost_occupancy(cloud, **config)

        elapsed_ms: list[float] = []
        grid = None
        profiler = _ResourceProfiler() if profile_resources else nullcontext()
        with profiler:
            for _ in range(iterations):
                start = time.perf_counter()
                grid = height_cost_occupancy(cloud, **config)
                elapsed_ms.append((time.perf_counter() - start) * 1_000.0)

    assert grid is not None
    ordered = sorted(elapsed_ms)
    p95 = ordered[math.ceil(0.95 * len(ordered)) - 1]
    median = statistics.median(elapsed_ms)
    print(f"points: {args.points:,}")
    print(f"grid: {grid.width}x{grid.height}")
    print(f"iterations: {iterations} (+{args.warmups} warmups)")
    print(f"median: {median:.2f} ms")
    print(f"mean: {statistics.mean(elapsed_ms):.2f} ms")
    print(f"p95: {p95:.2f} ms")
    print(f"range: {min(elapsed_ms):.2f}-{max(elapsed_ms):.2f} ms")
    print(f"throughput: {args.points / (median / 1_000.0):,.0f} points/s")
    if isinstance(profiler, _ResourceProfiler):
        profiler.print_summary()


if __name__ == "__main__":
    main()
