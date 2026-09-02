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

"""Compare Python and native Memory2 replay timing.

The public command runs every trial in a fresh process, then writes raw samples,
per-run metrics, aggregate JSON, and a Markdown report. The hidden ``--trial``
mode exists only so the orchestrator can isolate process-global DimOS state.
"""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import threading
import time
from typing import Any, Literal, cast

import psutil
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import ZenohTransport
from dimos.experimental.memory.rust_replayer import RustSqliteReplayStoreConfig
from dimos.memory.replay import resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP, Topic as ZenohTopic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import _unitree_go2_stack
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_rust_replay import (
    Go2ReplaySupport,
    Go2RustReplayer,
)
from dimos.utils.logging_config import setup_logger

Engine = Literal["python", "rust"]
Layer = Literal["isolated", "full"]

STREAMS = ("lidar", "odom", "color_image")
MODULE_NAME = "dimos.experimental.memory.tool_replay_benchmark"
DEFAULT_PROFILES = ("isolated:1", "isolated:4", "full:1")
P99_LIMIT_MS = 50.0
DRIFT_DELTA_LIMIT_MS = 10.0
SUBSCRIBER_SETTLE_SECONDS = 1.0
PUBLISHER_WARMUP_SECONDS = 1.0

logger = setup_logger()


@dataclass(frozen=True)
class Profile:
    layer: Layer
    speed: float

    @property
    def name(self) -> str:
        return f"{self.layer}:{self.speed:g}x"

    @classmethod
    def parse(cls, value: str) -> Profile:
        try:
            raw_layer, raw_speed = value.split(":", 1)
            layer = cast("Layer", raw_layer)
            speed = float(raw_speed.removesuffix("x"))
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                f"invalid profile {value!r}; expected isolated:SPEED or full:SPEED"
            ) from error
        if layer not in ("isolated", "full"):
            raise argparse.ArgumentTypeError(f"invalid replay layer: {layer!r}")
        if not math.isfinite(speed) or speed <= 0:
            raise argparse.ArgumentTypeError("profile speed must be finite and positive")
        return cls(layer=layer, speed=speed)


class PythonReplaySourceConfig(ModuleConfig):
    path: str
    speed: float = Field(gt=0)
    seek: float = Field(ge=0)
    duration: float = Field(gt=0)


class PythonReplaySource(Module):
    """Minimal graph source using the production Python ``Store.replay`` API."""

    config: PythonReplaySourceConfig
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        time.sleep(SUBSCRIBER_SETTLE_SECONDS)
        store = self.register_disposable(SqliteStore(path=self.config.path, must_exist=True))
        store.start()
        replay = store.replay(
            speed=self.config.speed,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        outputs: dict[str, Out[Any]] = {
            "lidar": self.lidar,
            "odom": self.odom,
            "color_image": self.color_image,
        }
        # Match GO2Connection: subscribe to the expensive lidar decoder first.
        # Subscribing a cheap stream first pins the shared anchor while lidar's
        # first decode is still running, which makes Python correctly treat its
        # own lidar frames as late and skip them.
        for name in STREAMS:

            def on_error(error: Exception, stream: str = name) -> None:
                logger.exception("Python replay stream failed", stream=stream, error=error)

            subscription = (
                replay.stream(name)
                .observable()
                .subscribe(
                    outputs[name].publish,
                    on_error,
                )
            )
            self.register_disposable(subscription)


class BenchmarkRustReplaySource(Go2RustReplayer):
    """Rust source with the same transport-discovery delay as Python."""

    @rpc
    def start(self) -> None:
        time.sleep(SUBSCRIBER_SETTLE_SECONDS)
        super().start()


class ReplayTimingSinkConfig(ModuleConfig):
    # Each pair is (payload timestamp, observation timestamp). The former is
    # visible after transport; the latter is the replay scheduler's clock.
    schedule: dict[str, list[tuple[float, float]]]
    warmup_schedule: dict[str, list[tuple[float, float]]]


class ReplayTimingSink(Module):
    """Capture source arrival times before downstream processing."""

    config: ReplayTimingSinkConfig
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    color_image: In[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._samples: list[dict[str, Any]] = []
        self._last_source_ts: dict[str, float] = {}
        self._out_of_order = dict.fromkeys(STREAMS, 0)
        self._duplicates = dict.fromkeys(STREAMS, 0)
        self._unexpected = dict.fromkeys(STREAMS, 0)
        self._warmup_received = dict.fromkeys(STREAMS, 0)
        self._expected_payloads: dict[str, set[str]] = {}
        self._remaining: dict[str, dict[str, deque[float]]] = {}
        self._warmup_remaining: dict[str, dict[str, deque[float]]] = {}
        for stream, pairs in self.config.schedule.items():
            by_payload: dict[str, deque[float]] = defaultdict(deque)
            for payload_ts, source_ts in pairs:
                by_payload[payload_ts.hex()].append(source_ts)
            self._remaining[stream] = dict(by_payload)
            self._expected_payloads[stream] = set(by_payload)
        for stream, pairs in self.config.warmup_schedule.items():
            by_payload = defaultdict(deque)
            for payload_ts, source_ts in pairs:
                by_payload[payload_ts.hex()].append(source_ts)
            self._warmup_remaining[stream] = dict(by_payload)

    @rpc
    def build(self) -> None:
        # Coordinator.build_all_modules finishes before any module starts, so
        # these subscriptions exist before either replay source can emit.
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))

    def _capture(self, stream: str, payload_ts: float) -> None:
        arrival_ns = time.perf_counter_ns()
        with self._lock:
            warmup_queue = self._warmup_remaining.get(stream, {}).get(payload_ts.hex())
            if warmup_queue:
                warmup_queue.popleft()
                self._warmup_received[stream] += 1
                return
            queue = self._remaining.get(stream, {}).get(payload_ts.hex())
            matched = bool(queue)
            if queue:
                source_ts = queue.popleft()
            else:
                source_ts = payload_ts
                if payload_ts.hex() in self._expected_payloads.get(stream, set()):
                    self._duplicates[stream] += 1
                else:
                    self._unexpected[stream] += 1
            previous = self._last_source_ts.get(stream)
            if matched and previous is not None and source_ts < previous:
                self._out_of_order[stream] += 1
            if matched:
                self._last_source_ts[stream] = source_ts
            self._samples.append(
                {
                    "stream": stream,
                    "source_ts": source_ts,
                    "payload_ts": payload_ts,
                    "arrival_ns": arrival_ns,
                    "matched": matched,
                }
            )

    def _on_lidar(self, message: PointCloud2) -> None:
        self._capture("lidar", float(message.ts))

    def _on_odom(self, message: PoseStamped) -> None:
        self._capture("odom", float(message.ts))

    def _on_color_image(self, message: Image) -> None:
        self._capture("color_image", float(message.ts))

    @rpc
    def progress(self) -> dict[str, int]:
        with self._lock:
            counts = dict.fromkeys(STREAMS, 0)
            for sample in self._samples:
                if sample["matched"]:
                    counts[cast("str", sample["stream"])] += 1
            return counts

    @rpc
    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            missing = {
                stream: sum(len(queue) for queue in self._remaining[stream].values())
                for stream in STREAMS
            }
            return {
                "samples": list(self._samples),
                "missing": missing,
                "duplicates": dict(self._duplicates),
                "unexpected": dict(self._unexpected),
                "out_of_order": dict(self._out_of_order),
                "warmup_received": dict(self._warmup_received),
            }


class ResourceSampler:
    """Sample aggregate CPU and RSS for the trial process tree."""

    def __init__(self) -> None:
        self.samples: list[dict[str, float]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._root = psutil.Process()
        self._known: dict[int, psutil.Process] = {self._root.pid: self._root}

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3)

    def _processes(self) -> list[psutil.Process]:
        try:
            processes = [self._root, *self._root.children(recursive=True)]
        except psutil.NoSuchProcess:
            processes = []
        for process in processes:
            self._known.setdefault(process.pid, process)
        return list(self._known.values())

    def _run(self) -> None:
        primed: set[int] = set()
        while not self._stop.is_set():
            cpu_pct = 0.0
            rss_bytes = 0
            for process in self._processes():
                try:
                    if process.pid not in primed:
                        process.cpu_percent(None)
                        primed.add(process.pid)
                    else:
                        cpu_pct += process.cpu_percent(None)
                    rss_bytes += process.memory_info().rss
                except (psutil.AccessDenied, psutil.NoSuchProcess):
                    continue
            self.samples.append(
                {
                    "elapsed_s": time.perf_counter(),
                    "cpu_pct": cpu_pct,
                    "rss_mb": rss_bytes / 1_000_000,
                }
            )
            self._stop.wait(1.0)


def _trial_schedules(
    path: Path, seek: float, duration: float, speed: float
) -> tuple[
    float,
    float,
    float,
    dict[str, list[tuple[float, float]]],
    dict[str, list[tuple[float, float]]],
]:
    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    try:
        first = min(store.stream(name).first().ts for name in STREAMS)
        target_start = first + seek
        target_stop = target_start + duration
        warmup_recording_seconds = min(seek, speed * PUBLISHER_WARMUP_SECONDS)
        replay_seek = seek - warmup_recording_seconds
        replay_duration = duration + warmup_recording_seconds
        replay_start = first + replay_seek
        target_schedule: dict[str, list[tuple[float, float]]] = {}
        warmup_schedule: dict[str, list[tuple[float, float]]] = {}
        for name in STREAMS:
            observations: Any = (
                store.stream(name).time_range(replay_start, target_stop).order_by("ts")
            )
            pairs = [
                (float(observation.data.ts), float(observation.ts)) for observation in observations
            ]
            warmup_schedule[name] = [pair for pair in pairs if pair[1] < target_start]
            target_schedule[name] = [pair for pair in pairs if pair[1] >= target_start]
        return (
            replay_seek,
            replay_duration,
            warmup_recording_seconds / speed,
            target_schedule,
            warmup_schedule,
        )
    finally:
        store.stop()


def _trial_blueprint(
    *,
    engine: Engine,
    layer: Layer,
    path: Path,
    speed: float,
    seek: float,
    duration: float,
    transport: str,
    topic_namespace: str,
    schedule: dict[str, list[tuple[float, float]]],
    warmup_schedule: dict[str, list[tuple[float, float]]],
) -> Blueprint:
    sink = ReplayTimingSink.blueprint(schedule=schedule, warmup_schedule=warmup_schedule)
    if engine == "python":
        source = PythonReplaySource.blueprint(
            path=str(path), speed=speed, seek=seek, duration=duration
        )
    else:
        source = BenchmarkRustReplaySource.blueprint(
            build_command=None,
            store=RustSqliteReplayStoreConfig(path=str(path)),
            speed=speed,
            seek=seek,
            duration=duration,
        )

    graph = autoconnect(sink, source)
    if layer == "full":
        graph = _unitree_go2_stack(autoconnect(graph, Go2ReplaySupport.blueprint()))
    if transport == "zenoh":
        types = {
            "lidar": PointCloud2,
            "odom": PoseStamped,
            "color_image": Image,
        }
        graph = graph.transports(
            {
                (name, message_type): ZenohTransport(
                    ZenohTopic(
                        f"{topic_namespace}/{name}",
                        message_type,
                        qos=QOS_NEVER_DROP,
                    )
                )
                for name, message_type in types.items()
            }
        )
    return graph


def _run_trial(args: argparse.Namespace) -> int:
    path = resolve_db_path(args.dataset).resolve()
    replay_seek, replay_duration, warmup_seconds, schedule, warmup_schedule = _trial_schedules(
        path, args.seek, args.duration, args.speed
    )
    expected = {stream: len(pairs) for stream, pairs in schedule.items()}
    expected_total = sum(expected.values())
    if expected_total == 0:
        raise ValueError("the selected replay window has no observations")

    blueprint = _trial_blueprint(
        engine=args.engine,
        layer=args.layer,
        path=path,
        speed=args.speed,
        seek=replay_seek,
        duration=replay_duration,
        transport=args.transport,
        topic_namespace=f"dimos/replay-benchmark/{os.getpid()}",
        schedule=schedule,
        warmup_schedule=warmup_schedule,
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        global_overrides={
            "viewer": "none",
            "replay": True,
            "replay_db": str(path),
            "transport": args.transport,
            "n_workers": len(blueprint.active_blueprints),
        },
    )

    sampler = ResourceSampler()
    sampler.start()
    build_started_ns = time.perf_counter_ns()
    coordinator: ModuleCoordinator | None = None
    try:
        coordinator = ModuleCoordinator.build(blueprint, parsed)
        sink = coordinator.get_instance(ReplayTimingSink)
        deadline = time.monotonic() + max(replay_duration / args.speed + 5.0, 8.0)
        while time.monotonic() < deadline:
            progress = sink.progress()
            if sum(progress.values()) >= expected_total:
                time.sleep(0.25)
                break
            time.sleep(0.05)
        snapshot = cast("dict[str, Any]", sink.snapshot())
    finally:
        if coordinator is not None:
            coordinator.stop()
        sampler.stop()

    matched_samples = [sample for sample in snapshot["samples"] if sample["matched"]]
    first_arrival_ns = min(
        (cast("int", sample["arrival_ns"]) for sample in matched_samples),
        default=None,
    )
    trial = {
        "profile": f"{args.layer}:{args.speed:g}x",
        "engine": args.engine,
        "repeat": args.repeat,
        "speed": args.speed,
        "warmup_seconds": warmup_seconds,
        "expected": expected,
        "build_started_ns": build_started_ns,
        "startup_ms": (
            None
            if first_arrival_ns is None
            else max(
                0.0,
                (first_arrival_ns - build_started_ns) / 1_000_000
                - (SUBSCRIBER_SETTLE_SECONDS + warmup_seconds) * 1_000,
            )
        ),
        "resource_samples": sampler.samples,
        **snapshot,
    }
    Path(args.trial_out).write_text(json.dumps(trial, indent=2) + "\n")
    return 0


def percentile(values: Sequence[float], percent: float) -> float | None:
    """Return a linearly interpolated percentile, or ``None`` for no values."""
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * percent / 100
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def _timing_metrics(errors_ms: Sequence[float]) -> dict[str, float | None]:
    jitter = [abs(value) for value in errors_ms]
    return {
        "p50_ms": percentile(jitter, 50),
        "p95_ms": percentile(jitter, 95),
        "p99_ms": percentile(jitter, 99),
        "max_ms": max(jitter, default=None),
    }


def _drift(errors_ms: Sequence[float]) -> float | None:
    if not errors_ms:
        return None
    width = max(1, math.ceil(len(errors_ms) * 0.10))
    return statistics.fmean(errors_ms[-width:]) - statistics.fmean(errors_ms[:width])


def analyze_run(trial: dict[str, Any]) -> dict[str, Any]:
    all_samples = sorted(trial["samples"], key=lambda sample: sample["arrival_ns"])
    samples = [sample for sample in all_samples if sample["matched"]]
    if samples:
        arrival_origin = min(sample["arrival_ns"] for sample in samples)
        source_origin = min(sample["source_ts"] for sample in samples)
        speed = trial["speed"]
        phases_ms = [
            (sample["arrival_ns"] - arrival_origin) / 1_000_000
            - (sample["source_ts"] - source_origin) * 1_000 / speed
            for sample in samples
        ]
        phase_center_ms = statistics.median(phases_ms)
        for sample, phase_ms in zip(samples, phases_ms, strict=True):
            schedule_error_ms = phase_ms - phase_center_ms
            sample["schedule_error_ms"] = schedule_error_ms
            sample["jitter_ms"] = abs(schedule_error_ms)

    by_stream_samples: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for sample in samples:
        by_stream_samples[sample["stream"]].append(sample)
    by_stream = {
        stream: [
            sample["schedule_error_ms"]
            for sample in sorted(by_stream_samples[stream], key=lambda item: item["source_ts"])
        ]
        for stream in STREAMS
    }
    stream_metrics = {
        stream: {
            **_timing_metrics(by_stream[stream]),
            "drift_ms": _drift(by_stream[stream]),
        }
        for stream in STREAMS
    }
    all_errors = [sample["schedule_error_ms"] for sample in samples]
    p99_values = [
        metric["p99_ms"] for metric in stream_metrics.values() if metric["p99_ms"] is not None
    ]
    drift_values = [
        abs(metric["drift_ms"])
        for metric in stream_metrics.values()
        if metric["drift_ms"] is not None
    ]
    resources = trial["resource_samples"]
    return {
        **{
            key: value for key, value in trial.items() if key not in ("samples", "resource_samples")
        },
        "received": {
            stream: sum(1 for sample in samples if sample["stream"] == stream) for stream in STREAMS
        },
        "timing": _timing_metrics(all_errors),
        "stream_timing": stream_metrics,
        "primary_p99_ms": max(p99_values, default=None),
        "primary_abs_drift_ms": max(drift_values, default=None),
        "cpu_mean_pct": (
            statistics.fmean(sample["cpu_pct"] for sample in resources) if resources else None
        ),
        "rss_peak_mb": max((sample["rss_mb"] for sample in resources), default=None),
        "samples": all_samples,
    }


def _median(values: Iterable[float | None]) -> float | None:
    present = [value for value in values if value is not None]
    return statistics.median(present) if present else None


def aggregate_runs(runs: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for run in runs:
        grouped[(run["profile"], run["engine"])].append(run)

    aggregates: list[dict[str, Any]] = []
    for (profile, engine), group in sorted(grouped.items()):
        pooled_errors = [
            sample["schedule_error_ms"]
            for run in group
            for sample in run["samples"]
            if sample["matched"]
        ]
        stream_drift_medians = {
            stream: _median(run["stream_timing"][stream]["drift_ms"] for run in group)
            for stream in STREAMS
        }
        aggregates.append(
            {
                "profile": profile,
                "engine": engine,
                "runs": len(group),
                "expected": sum(sum(run["expected"].values()) for run in group),
                "received": sum(sum(run["received"].values()) for run in group),
                "missing": sum(sum(run["missing"].values()) for run in group),
                "duplicates": sum(sum(run["duplicates"].values()) for run in group),
                "unexpected": sum(sum(run["unexpected"].values()) for run in group),
                "out_of_order": sum(sum(run["out_of_order"].values()) for run in group),
                "timing_pooled": _timing_metrics(pooled_errors),
                "median_run_p99_ms": _median(run["primary_p99_ms"] for run in group),
                "median_startup_ms": _median(run["startup_ms"] for run in group),
                "median_cpu_pct": _median(run["cpu_mean_pct"] for run in group),
                "median_rss_peak_mb": _median(run["rss_peak_mb"] for run in group),
                "median_drift_ms": stream_drift_medians,
            }
        )
    return aggregates


def evaluate_gates(aggregates: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    by_key = {(item["profile"], item["engine"]): item for item in aggregates}
    profiles = sorted({item["profile"] for item in aggregates})
    results: list[dict[str, Any]] = []
    for profile in profiles:
        python = by_key[(profile, "python")]
        rust = by_key[(profile, "rust")]
        python_p99 = python["median_run_p99_ms"]
        rust_p99 = rust["median_run_p99_ms"]
        lossless = all(
            item[key] == 0
            for item in (python, rust)
            for key in ("missing", "duplicates", "unexpected", "out_of_order")
        )
        drift_delta = max(
            abs(
                cast("float", rust["median_drift_ms"][stream])
                - cast("float", python["median_drift_ms"][stream])
            )
            for stream in STREAMS
            if rust["median_drift_ms"][stream] is not None
            and python["median_drift_ms"][stream] is not None
        )
        checks = {
            "lossless_and_ordered": lossless,
            "python_p99_at_most_50ms": (python_p99 is not None and python_p99 <= P99_LIMIT_MS),
            "rust_p99_at_most_50ms": rust_p99 is not None and rust_p99 <= P99_LIMIT_MS,
            "drift_delta_at_most_10ms": drift_delta <= DRIFT_DELTA_LIMIT_MS,
        }
        results.append(
            {
                "profile": profile,
                "passed": all(checks.values()),
                "checks": checks,
                "python_p99_ms": python_p99,
                "rust_p99_ms": rust_p99,
                "p99_improvement_pct": (
                    None if not python_p99 else (python_p99 - rust_p99) / python_p99 * 100
                ),
                "max_drift_delta_ms": drift_delta,
            }
        )
    return results


def _fmt(value: Any, digits: int = 2) -> str:
    return "—" if value is None else f"{value:.{digits}f}"


def render_report(
    aggregates: Sequence[dict[str, Any]], gates: Sequence[dict[str, Any]], metadata: dict[str, Any]
) -> str:
    lines = [
        "# Memory2 replay benchmark",
        "",
        (
            f"Dataset: `{metadata['dataset']}` · window: {metadata['seek']:g}s + "
            f"{metadata['duration']:g}s · repeats: {metadata['repeats']} · "
            f"transport: `{metadata['transport']}`"
        ),
        "",
        "Timing uses replay observation timestamps after an unmeasured publisher warm-up. "
        "Each run is centered on its median delivery phase; percentiles measure absolute "
        "schedule jitter, and the primary p99 is the slowest stream's p99 aggregated as the "
        "median run.",
        "",
        "| Profile | Engine | Received | Missing | Duplicate | Unexpected | OOO | p50 jitter (ms) | p95 jitter (ms) | p99 jitter (ms) | Max jitter (ms) | Startup (ms) | CPU (%) | RSS peak (MB) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in aggregates:
        pooled = item["timing_pooled"]
        lines.append(
            "| {profile} | {engine} | {received}/{expected} | {missing} | {duplicates} | "
            "{unexpected} | {out_of_order} | {p50} | {p95} | {p99} | {maximum} | {startup} | {cpu} | {rss} |".format(
                **item,
                p50=_fmt(pooled["p50_ms"]),
                p95=_fmt(pooled["p95_ms"]),
                p99=_fmt(item["median_run_p99_ms"]),
                maximum=_fmt(pooled["max_ms"]),
                startup=_fmt(item["median_startup_ms"]),
                cpu=_fmt(item["median_cpu_pct"]),
                rss=_fmt(item["median_rss_peak_mb"]),
            )
        )
    lines.extend(
        [
            "",
            "## Gates",
            "",
            "| Profile | Result | Python p99 (ms) | Rust p99 (ms) | Improvement | Max drift delta (ms) |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for gate in gates:
        improvement = gate["p99_improvement_pct"]
        lines.append(
            f"| {gate['profile']} | {'PASS' if gate['passed'] else 'FAIL'} | "
            f"{_fmt(gate['python_p99_ms'])} | {_fmt(gate['rust_p99_ms'])} | "
            f"{_fmt(improvement)}% | {_fmt(gate['max_drift_delta_ms'])} |"
        )
    lines.extend(
        [
            "",
            "A profile passes when both engines are lossless and ordered, each engine's median "
            "p99 jitter is at most 50 ms, and the per-stream drift delta is at most 10 ms. "
            "Relative improvement is descriptive and is not an acceptance gate.",
            "",
        ]
    )
    return "\n".join(lines)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_metadata() -> dict[str, Any]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=DIMOS_PROJECT_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    dirty = bool(
        subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=DIMOS_PROJECT_ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    )
    return {"commit": commit, "dirty": dirty}


def _orchestrate(args: argparse.Namespace) -> int:
    profiles = [Profile.parse(value) for value in args.profile]
    path = resolve_db_path(args.dataset).resolve()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    trial_dir = out / ".trials"
    trial_dir.mkdir(exist_ok=True)

    subprocess.run(
        ["cargo", "build", "--release", "-p", "dimos-memory-replayer"],
        cwd=DIMOS_PROJECT_ROOT,
        check=True,
    )

    run_id = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    trials: list[dict[str, Any]] = []
    for profile in profiles:
        for repeat in range(args.repeats):
            engines: tuple[Engine, Engine] = (
                ("python", "rust") if repeat % 2 == 0 else ("rust", "python")
            )
            for engine in engines:
                trial_path = trial_dir / f"{run_id}-{profile.name}-{engine}-{repeat}.json"
                command = [
                    sys.executable,
                    "-m",
                    MODULE_NAME,
                    "--trial",
                    "--dataset",
                    args.dataset,
                    "--seek",
                    str(args.seek),
                    "--duration",
                    str(args.duration),
                    "--transport",
                    args.transport,
                    "--engine",
                    engine,
                    "--layer",
                    profile.layer,
                    "--speed",
                    str(profile.speed),
                    "--repeat",
                    str(repeat),
                    "--trial-out",
                    str(trial_path),
                ]
                print(f"[{profile.name} repeat {repeat + 1}/{args.repeats}] {engine}", flush=True)
                completed = subprocess.run(
                    command,
                    cwd=DIMOS_PROJECT_ROOT,
                    capture_output=True,
                    text=True,
                )
                if completed.returncode != 0:
                    (trial_path.with_suffix(".log")).write_text(completed.stdout + completed.stderr)
                    print(completed.stdout, end="", file=sys.stderr)
                    print(completed.stderr, end="", file=sys.stderr)
                    return completed.returncode
                trials.append(json.loads(trial_path.read_text()))

    runs = [analyze_run(trial) for trial in trials]
    aggregates = aggregate_runs(runs)
    gates = evaluate_gates(aggregates)
    metadata = {
        "created_at": datetime.now(UTC).isoformat(),
        "dataset": args.dataset,
        "dataset_path": str(path),
        "dataset_size_bytes": path.stat().st_size,
        "dataset_sha256": _sha256(path),
        "seek": args.seek,
        "duration": args.duration,
        "repeats": args.repeats,
        "profiles": [profile.name for profile in profiles],
        "transport": args.transport,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "processor": platform.processor(),
        **_git_metadata(),
    }
    serializable_runs = [
        {key: value for key, value in run.items() if key != "samples"} for run in runs
    ]
    (out / "runs.json").write_text(json.dumps(serializable_runs, indent=2) + "\n")
    summary = {"metadata": metadata, "aggregates": aggregates, "gates": gates}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    with (out / "samples.jsonl").open("w") as destination:
        for run in runs:
            context = {
                "profile": run["profile"],
                "engine": run["engine"],
                "repeat": run["repeat"],
            }
            for sample in run["samples"]:
                destination.write(json.dumps({**context, **sample}) + "\n")
    report = render_report(aggregates, gates, metadata)
    (out / "report.md").write_text(report)
    print("\n" + report)
    print(f"Results: {out.resolve()}")
    return 1 if args.check and not all(gate["passed"] for gate in gates) else 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="go2_short")
    parser.add_argument("--seek", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--transport", default="zenoh")
    parser.add_argument("--out", default="results/replay-benchmark")
    parser.add_argument("--profile", action="append", default=None)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--trial", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--engine", choices=("python", "rust"), help=argparse.SUPPRESS)
    parser.add_argument("--layer", choices=("isolated", "full"), help=argparse.SUPPRESS)
    parser.add_argument("--speed", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--repeat", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--trial-out", help=argparse.SUPPRESS)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.seek < 0 or args.duration <= 0 or args.repeats <= 0:
        raise SystemExit("seek must be non-negative; duration and repeats must be positive")
    if args.trial:
        required = (args.engine, args.layer, args.speed, args.repeat, args.trial_out)
        if any(value is None for value in required):
            raise SystemExit("hidden trial mode requires engine, layer, speed, repeat, and output")
        return _run_trial(args)
    args.profile = args.profile or list(DEFAULT_PROFILES)
    return _orchestrate(args)


if __name__ == "__main__":
    raise SystemExit(main())
