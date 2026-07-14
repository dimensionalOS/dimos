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

"""Profile a full DimOS replay while swapping only the CostMapper implementation."""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import asdict, dataclass
import json
import logging
import os
from pathlib import Path
import platform
import statistics
import sys
import threading
import time
from typing import TYPE_CHECKING, Any

import psutil

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.python_worker import MethodCallProxy
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.mapping.costmapper import CostMapper as PythonCostMapper
from dimos.mapping.native_costmapper.module import CostMapper as RustCostMapper
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    from matplotlib.figure import Figure

LOGGER = logging.getLogger(__name__)
_COLORS = {"python": "#dc2626", "rust": "#16a34a"}
_DTOP_INTERVAL_SECONDS = 1.0


def _start_module_over_worker_pipe(module: Any) -> None:
    """Start a local module without putting its lifecycle response on the data bus."""
    actor = module.actor_instance
    MethodCallProxy(actor).start().result()


class BenchmarkModuleCoordinator(ModuleCoordinator):
    """Keep benchmark startup reliable while runtime streams use the selected transport."""

    def start_all_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        if not modules:
            raise ValueError("No modules deployed. Call deploy() before start_all_modules().")

        # Replays can flood LCM with multi-megabyte camera frames before all
        # lifecycle responses return. The workers are local, so use their
        # reliable control pipes for startup; measured streams remain on LCM.
        safe_thread_map(modules, _start_module_over_worker_pipe)
        self._send_on_system_modules()


@dataclass(frozen=True)
class ResourceSample:
    elapsed_seconds: float
    cpu_percent: float
    rss_mib: float
    process_count: int

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> ResourceSample:
        return cls(
            elapsed_seconds=float(values["elapsed_seconds"]),
            cpu_percent=float(values["cpu_percent"]),
            rss_mib=float(values["rss_mib"]),
            process_count=int(values["process_count"]),
        )


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, round((len(ordered) - 1) * fraction)))
    return float(ordered[index])


@dataclass(frozen=True)
class SystemBenchmarkResult:
    label: str
    implementation: str
    blueprint: str
    replay_db: str
    transport: str
    requested_duration_seconds: float
    observed_duration_seconds: float
    sample_interval_seconds: float
    interrupted: bool
    samples: list[ResourceSample]

    @property
    def median_cpu_percent(self) -> float:
        return float(statistics.median(sample.cpu_percent for sample in self.samples))

    @property
    def p95_cpu_percent(self) -> float:
        return _percentile([sample.cpu_percent for sample in self.samples], 0.95)

    @property
    def peak_cpu_percent(self) -> float:
        return max((sample.cpu_percent for sample in self.samples), default=0.0)

    @property
    def median_rss_mib(self) -> float:
        return float(statistics.median(sample.rss_mib for sample in self.samples))

    @property
    def peak_rss_mib(self) -> float:
        return max((sample.rss_mib for sample in self.samples), default=0.0)

    def summary_dict(self) -> dict[str, float | int]:
        return {
            "sample_count": len(self.samples),
            "median_cpu_percent": self.median_cpu_percent,
            "p95_cpu_percent": self.p95_cpu_percent,
            "peak_cpu_percent": self.peak_cpu_percent,
            "median_rss_mib": self.median_rss_mib,
            "peak_rss_mib": self.peak_rss_mib,
        }

    def workload_dict(self) -> dict[str, str | float]:
        return {
            "blueprint": self.blueprint,
            "replay_db": self.replay_db,
            "transport": self.transport,
            "requested_duration_seconds": self.requested_duration_seconds,
            "sample_interval_seconds": self.sample_interval_seconds,
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "implementation": self.implementation,
            "workload": self.workload_dict(),
            "observed_duration_seconds": self.observed_duration_seconds,
            "interrupted": self.interrupted,
            "summary": self.summary_dict(),
            "samples": [asdict(sample) for sample in self.samples],
        }

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> SystemBenchmarkResult:
        workload = values["workload"]
        return cls(
            label=str(values["label"]),
            implementation=str(values["implementation"]),
            blueprint=str(workload["blueprint"]),
            replay_db=str(workload["replay_db"]),
            transport=str(workload["transport"]),
            requested_duration_seconds=float(workload["requested_duration_seconds"]),
            observed_duration_seconds=float(values["observed_duration_seconds"]),
            sample_interval_seconds=float(workload["sample_interval_seconds"]),
            interrupted=bool(values["interrupted"]),
            samples=[ResourceSample.from_dict(sample) for sample in values["samples"]],
        )


class DtopAggregateSampler:
    """Collapse dtop's process readings into one full-system CPU/RSS series."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.samples: list[ResourceSample] = []
        self._root = psutil.Process()
        self._transport: PubSubTransport[dict[str, Any]] | None = None
        self._lock = threading.Lock()
        self._started_at = 0.0

    def _tree_rss(self) -> tuple[float, int]:
        try:
            processes = [self._root, *self._root.children(recursive=True)]
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            processes = [self._root]
        rss_bytes = 0
        process_count = 0
        for process in processes:
            try:
                rss_bytes += process.memory_info().rss
                process_count += 1
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return rss_bytes / 1024**2, process_count

    def _record_sample(self, message: dict[str, Any]) -> None:
        coordinator = message.get("coordinator") or {}
        workers = message.get("workers") or []
        cpu_percent = float(coordinator.get("cpu_percent", 0.0)) + sum(
            float(worker.get("cpu_percent", 0.0))
            for worker in workers
            if worker.get("alive", False)
        )
        rss_mib, process_count = self._tree_rss()
        sample = ResourceSample(
            elapsed_seconds=time.monotonic() - self._started_at,
            cpu_percent=cpu_percent,
            rss_mib=rss_mib,
            process_count=process_count,
        )
        with self._lock:
            self.samples.append(sample)
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(asdict(sample), sort_keys=True) + "\n")

    def __enter__(self) -> DtopAggregateSampler:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("", encoding="utf-8")
        self._started_at = time.monotonic()
        self._transport = make_transport("/resource_stats")
        self._transport.subscribe(self._record_sample)
        self._transport.start()
        return self

    def __exit__(self, *_args: object) -> None:
        if self._transport is not None:
            self._transport.stop()
            self._transport = None
        if not self.samples:
            raise RuntimeError(
                "dtop did not emit any resource samples; use a duration longer than one second"
            )


def _select_blueprint(name: str, implementation: str) -> Blueprint:
    blueprint = get_blueprint_by_name(name)
    if implementation == "python":
        return blueprint
    if implementation != "rust":
        raise ValueError(f"unknown CostMapper implementation: {implementation}")

    active_modules = {atom.module for atom in blueprint.active_blueprints}
    if PythonCostMapper not in active_modules:
        raise ValueError(
            f"blueprint {name!r} does not contain {PythonCostMapper.__name__}; "
            "the benchmark cannot guarantee a one-module swap"
        )
    return autoconnect(
        blueprint.disabled_modules(PythonCostMapper),
        RustCostMapper.blueprint(),
    )


def _active_module_names(blueprint: Blueprint) -> list[str]:
    return [
        f"{atom.module.__module__}.{atom.module.__qualname__}"
        for atom in blueprint.active_blueprints
    ]


def _stop_coordinator(coordinator: ModuleCoordinator, timeout_seconds: float) -> bool:
    errors: list[BaseException] = []

    def stop() -> None:
        try:
            coordinator.stop()
        except BaseException as error:
            errors.append(error)

    thread = threading.Thread(target=stop, name="dimos-benchmark-stop", daemon=True)
    thread.start()
    thread.join(timeout=timeout_seconds)
    if errors:
        raise RuntimeError("failed to stop DimOS") from errors[0]
    return not thread.is_alive()


def _write_json(path: Path, values: dict[str, Any]) -> None:
    path.write_text(json.dumps(values, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _save_figure(figure: Figure, path: Path) -> None:
    # Keep matplotlib out of the forkserver workers being measured.
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    FigureCanvasAgg(figure)
    figure.savefig(path, dpi=160, bbox_inches="tight")
    figure.clear()


def _render_run_charts(result: SystemBenchmarkResult, out_dir: Path) -> None:
    # Import only after DimOS has stopped so charting cannot affect the profile.
    from matplotlib.figure import Figure

    elapsed = [sample.elapsed_seconds for sample in result.samples]
    cpu = [sample.cpu_percent for sample in result.samples]
    rss = [sample.rss_mib for sample in result.samples]
    color = _COLORS[result.implementation]

    timeline = Figure(figsize=(10, 7), constrained_layout=True)
    axes = timeline.subplots(2, 1, sharex=True)
    axes[0].plot(elapsed, cpu, color=color, linewidth=1.6)
    axes[0].axhline(result.median_cpu_percent, color=color, linestyle="--", alpha=0.65)
    axes[0].set_ylabel("CPU (%)")
    axes[0].set_title(f"{result.label}: full DimOS process tree")
    axes[0].grid(alpha=0.2)
    axes[1].plot(elapsed, rss, color=color, linewidth=1.6)
    axes[1].axhline(result.median_rss_mib, color=color, linestyle="--", alpha=0.65)
    axes[1].set_xlabel("Elapsed time (s)")
    axes[1].set_ylabel("RSS (MiB)")
    axes[1].grid(alpha=0.2)
    _save_figure(timeline, out_dir / "resources_over_time.png")

    summary = Figure(figsize=(8, 4), constrained_layout=True)
    cpu_axis, rss_axis = summary.subplots(1, 2)
    cpu_axis.bar(
        ["Median", "p95", "Peak"],
        [result.median_cpu_percent, result.p95_cpu_percent, result.peak_cpu_percent],
        color=color,
    )
    cpu_axis.set_title("CPU")
    cpu_axis.set_ylabel("Process-tree CPU (%)")
    cpu_axis.grid(axis="y", alpha=0.2)
    rss_axis.bar(
        ["Median", "Peak"],
        [result.median_rss_mib, result.peak_rss_mib],
        color=color,
    )
    rss_axis.set_title("Memory")
    rss_axis.set_ylabel("Process-tree RSS (MiB)")
    rss_axis.grid(axis="y", alpha=0.2)
    _save_figure(summary, out_dir / "resource_summary.png")


def _run_system_benchmark(args: argparse.Namespace) -> SystemBenchmarkResult:
    out_dir: Path = args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    blueprint = _select_blueprint(args.blueprint, args.implementation)
    label = args.label or args.implementation.title()
    metadata: dict[str, Any] = {
        "label": label,
        "implementation": args.implementation,
        "blueprint": args.blueprint,
        "replay_db": args.replay_db,
        "transport": args.transport,
        "duration_seconds": args.duration,
        "sample_interval_seconds": _DTOP_INTERVAL_SECONDS,
        "profiler": "dtop aggregate (CPU) + full process tree (RSS)",
        "active_modules": _active_module_names(blueprint),
        "host": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "logical_cpu_count": psutil.cpu_count(logical=True),
            "physical_cpu_count": psutil.cpu_count(logical=False),
            "total_memory_mib": psutil.virtual_memory().total / 1024**2,
        },
    }
    _write_json(out_dir / "meta.json", metadata)

    coordinator: ModuleCoordinator | None = None
    stopped_cleanly = True
    interrupted = False
    result: SystemBenchmarkResult | None = None
    started_at = time.monotonic()
    sampler = DtopAggregateSampler(out_dir / "host.jsonl")
    try:
        coordinator = BenchmarkModuleCoordinator.build(
            blueprint,
            {
                "g": {
                    "replay": True,
                    "replay_db": args.replay_db,
                    "viewer": "none",
                    "transport": args.transport,
                    "dtop": True,
                }
            },
        )
        started_at = time.monotonic()
        with sampler:
            try:
                threading.Event().wait(args.duration)
            except KeyboardInterrupt:
                interrupted = True

        result = SystemBenchmarkResult(
            label=label,
            implementation=args.implementation,
            blueprint=args.blueprint,
            replay_db=args.replay_db,
            transport=args.transport,
            requested_duration_seconds=args.duration,
            observed_duration_seconds=time.monotonic() - started_at,
            sample_interval_seconds=_DTOP_INTERVAL_SECONDS,
            interrupted=interrupted,
            samples=list(sampler.samples),
        )
        _write_json(out_dir / "result.json", result.to_dict())
    finally:
        if coordinator is not None:
            stopped_cleanly = _stop_coordinator(coordinator, args.stop_timeout)
        if not stopped_cleanly:
            LOGGER.error("DimOS did not stop within %.1f seconds", args.stop_timeout)
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)

    if result is None:
        raise RuntimeError("benchmark finished without a result")
    _render_run_charts(result, out_dir)
    (out_dir / "done").write_text("ok\n", encoding="utf-8")
    return result


def _load_result(path: Path) -> SystemBenchmarkResult:
    return SystemBenchmarkResult.from_dict(json.loads(path.read_text(encoding="utf-8")))


def _validate_comparison(results: list[SystemBenchmarkResult]) -> None:
    if not results:
        raise ValueError("comparison requires at least one result file")
    implementations = {result.implementation for result in results}
    if implementations != {"python", "rust"}:
        raise ValueError("comparison requires both Python and Rust result files")

    reference = results[0].workload_dict()
    for result in results[1:]:
        if result.workload_dict() != reference:
            raise ValueError(
                "all comparison runs must use the same blueprint, replay, transport, "
                "duration, and sampling interval"
            )


def _pooled_summary(results: list[SystemBenchmarkResult]) -> dict[str, float | int]:
    samples = [sample for result in results for sample in result.samples]
    cpu = [sample.cpu_percent for sample in samples]
    rss = [sample.rss_mib for sample in samples]
    return {
        "run_count": len(results),
        "sample_count": len(samples),
        "median_cpu_percent": float(statistics.median(cpu)),
        "p95_cpu_percent": _percentile(cpu, 0.95),
        "peak_cpu_percent": max(cpu, default=0.0),
        "median_rss_mib": float(statistics.median(rss)),
        "peak_rss_mib": max(rss, default=0.0),
    }


def _render_comparison(
    results: list[SystemBenchmarkResult],
    summaries: dict[str, dict[str, float | int]],
    out_dir: Path,
) -> None:
    from matplotlib.figure import Figure

    implementations = ["python", "rust"]
    labels = [implementation.title() for implementation in implementations]
    colors = [_COLORS[implementation] for implementation in implementations]

    overview = Figure(figsize=(10, 4.5), constrained_layout=True)
    cpu_axis, rss_axis = overview.subplots(1, 2)
    cpu_axis.bar(
        labels,
        [float(summaries[item]["median_cpu_percent"]) for item in implementations],
        color=colors,
    )
    cpu_axis.set_title("Median CPU")
    cpu_axis.set_ylabel("Process-tree CPU (%)")
    cpu_axis.grid(axis="y", alpha=0.2)
    rss_axis.bar(
        labels,
        [float(summaries[item]["median_rss_mib"]) for item in implementations],
        color=colors,
    )
    rss_axis.set_title("Median memory")
    rss_axis.set_ylabel("Process-tree RSS (MiB)")
    rss_axis.grid(axis="y", alpha=0.2)
    _save_figure(overview, out_dir / "comparison_summary.png")

    timeline = Figure(figsize=(10, 7), constrained_layout=True)
    cpu_axis, rss_axis = timeline.subplots(2, 1, sharex=True)
    seen: set[str] = set()
    for result in results:
        label = result.implementation.title() if result.implementation not in seen else None
        seen.add(result.implementation)
        elapsed = [sample.elapsed_seconds for sample in result.samples]
        color = _COLORS[result.implementation]
        cpu_axis.plot(
            elapsed,
            [sample.cpu_percent for sample in result.samples],
            color=color,
            alpha=0.65,
            linewidth=1.2,
            label=label,
        )
        rss_axis.plot(
            elapsed,
            [sample.rss_mib for sample in result.samples],
            color=color,
            alpha=0.65,
            linewidth=1.2,
            label=label,
        )
    cpu_axis.set_ylabel("CPU (%)")
    cpu_axis.set_title("Full DimOS process tree")
    cpu_axis.grid(alpha=0.2)
    cpu_axis.legend()
    rss_axis.set_xlabel("Elapsed time (s)")
    rss_axis.set_ylabel("RSS (MiB)")
    rss_axis.grid(alpha=0.2)
    rss_axis.legend()
    _save_figure(timeline, out_dir / "comparison_over_time.png")


def _compare_results(paths: list[Path], out_dir: Path) -> dict[str, Any]:
    results = [_load_result(path) for path in paths]
    _validate_comparison(results)
    grouped: defaultdict[str, list[SystemBenchmarkResult]] = defaultdict(list)
    for result in results:
        grouped[result.implementation].append(result)
    summaries = {
        implementation: _pooled_summary(grouped[implementation])
        for implementation in ("python", "rust")
    }
    comparison: dict[str, Any] = {
        "workload": results[0].workload_dict(),
        "groups": summaries,
        "inputs": [str(path) for path in paths],
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "comparison.json", comparison)
    _render_comparison(results, summaries, out_dir)
    return comparison


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Profile the complete DimOS Go2 replay and compare Python versus Rust "
            "CostMapper with every other module held constant."
        )
    )
    parser.add_argument("--implementation", choices=("python", "rust"), default="python")
    parser.add_argument("--blueprint", default="unitree-go2")
    parser.add_argument("--replay-db", default="go2_bigoffice")
    parser.add_argument("--transport", choices=("lcm", "zenoh"), default="lcm")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--stop-timeout", type=float, default=30.0)
    parser.add_argument("--label")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--compare",
        type=Path,
        nargs="+",
        metavar="RESULT_JSON",
        help="build comparison charts from completed result.json files instead of running DimOS",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0:
        raise ValueError("--duration must be greater than zero")
    if args.stop_timeout <= 0:
        raise ValueError("--stop-timeout must be greater than zero")


def main() -> None:
    args = _build_parser().parse_args()
    _validate_args(args)
    if args.compare:
        comparison = _compare_results(args.compare, args.out)
        print(json.dumps(comparison["groups"], indent=2, sort_keys=True))
        print(f"Comparison written to {args.out}")
        return

    result = _run_system_benchmark(args)
    print(json.dumps(result.summary_dict(), indent=2, sort_keys=True))
    print(f"Benchmark written to {args.out}")


if __name__ == "__main__":
    main()
