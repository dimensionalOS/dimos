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

"""Container endpoint process for emulated transport benchmark trials."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import threading
import time
from typing import Any

import psutil

from dimos.protocol.pubsub.benchmark.model import ProcessUsage, TrialSpec, trial_spec_from_dict
from dimos.protocol.pubsub.benchmark.runner import ImageBenchmarkBus

_POLL_INTERVAL_S = 0.01


def _usage(role: str, started_cpu: float) -> dict[str, Any]:
    process = psutil.Process()
    cpu = process.cpu_times()
    switches = process.num_ctx_switches()
    peak_rss = process.memory_info().rss
    for line in Path(f"/proc/{process.pid}/status").read_text().splitlines():
        if line.startswith("VmHWM:"):
            peak_rss = int(line.split()[1]) * 1024
            break
    return asdict(
        ProcessUsage(
            role=role,
            cpu_seconds=cpu.user + cpu.system - started_cpu,
            peak_rss_bytes=peak_rss,
            voluntary_context_switches=switches.voluntary,
            involuntary_context_switches=switches.involuntary,
        )
    )


def _write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, sort_keys=True) + "\n")


def _load_spec(path: Path) -> TrialSpec:
    return trial_spec_from_dict(json.loads(path.read_text()))


def run_subscriber(spec: TrialSpec, receiver: int, coordination: Path, output: Path) -> None:
    process = psutil.Process()
    started_cpu = 0.0
    usage: dict[str, Any] | None = None
    received: list[tuple[str, int, int, int]] = []
    lock = threading.Lock()
    error = None
    bus: ImageBenchmarkBus | None = None
    try:
        deadline = time.monotonic() + 120.0
        while not (coordination / "publisher-ready").exists():
            if time.monotonic() >= deadline:
                raise TimeoutError("publisher readiness timed out")
            time.sleep(_POLL_INTERVAL_S)
        bus = ImageBenchmarkBus(spec, "subscriber")

        def record(topic: str, sequence: int, publish_start_ns: int) -> None:
            if sequence < 0:
                return
            with lock:
                received.append((topic, sequence, publish_start_ns, time.perf_counter_ns()))

        bus.start()
        for topic in spec.workload.topics:
            bus.subscribe(topic.name, record)
        (coordination / f"ready-{receiver}").touch()
        deadline = time.monotonic() + spec.warmup_s + spec.duration_s + 120.0
        while not (coordination / "measure-started").exists():
            if (coordination / "done").exists():
                raise RuntimeError("publisher stopped before measurement started")
            if time.monotonic() >= deadline:
                raise TimeoutError("measurement start timed out")
            time.sleep(_POLL_INTERVAL_S)
        cpu = process.cpu_times()
        started_cpu = cpu.user + cpu.system
        while not (coordination / "done").exists():
            if time.monotonic() >= deadline:
                raise TimeoutError("publisher completion timed out")
            time.sleep(_POLL_INTERVAL_S)
        usage = _usage("subscriber", started_cpu)
        time.sleep(spec.drain_s)
    except Exception as caught:
        error = str(caught)
    finally:
        if bus is not None:
            bus.stop()
    with lock:
        result = received.copy()
    _write(
        output,
        {
            "receiver": receiver,
            "received": result,
            "usage": usage or _usage("subscriber", started_cpu),
            "error": error,
        },
    )


def run_publisher(spec: TrialSpec, coordination: Path, output: Path) -> None:
    process = psutil.Process()
    started_cpu = 0.0
    usage: dict[str, Any] | None = None
    sent: list[tuple[str, int, int, int, int, int]] = []
    error = None
    bus: ImageBenchmarkBus | None = None
    try:
        bus = ImageBenchmarkBus(spec, "publisher")
        bus.start()
        (coordination / "publisher-ready").touch()
        deadline = time.monotonic() + 120.0
        while len(list(coordination.glob("ready-*"))) < spec.subscribers:
            if time.monotonic() >= deadline:
                raise TimeoutError("subscriber readiness timed out")
            time.sleep(_POLL_INTERVAL_S)
        sequence_by_topic = {topic.name: 0 for topic in spec.workload.topics}

        def run_window(duration_s: float, measured: bool) -> float:
            start_ns = time.perf_counter_ns()
            end_ns = start_ns + int(duration_s * 1e9)
            next_ns = {topic.name: start_ns for topic in spec.workload.topics}
            offered_bytes = 0
            while time.perf_counter_ns() < end_ns:
                if measured and spec.workload.saturation:
                    if (
                        spec.workload.saturation_max_messages is not None
                        and len(sent) >= spec.workload.saturation_max_messages
                    ) or (
                        spec.workload.saturation_max_bytes is not None
                        and offered_bytes >= spec.workload.saturation_max_bytes
                    ):
                        break
                workload = min(spec.workload.topics, key=lambda topic: next_ns[topic.name])
                due_ns = next_ns[workload.name]
                now_ns = time.perf_counter_ns()
                if not spec.workload.saturation and now_ns < due_ns:
                    time.sleep((due_ns - now_ns) / 1e9)
                    if time.perf_counter_ns() >= end_ns:
                        break
                current_sequence = sequence_by_topic[workload.name]
                sequence = current_sequence if measured else -1 - current_sequence
                publish_start_ns = time.perf_counter_ns()
                bus.publish(workload.name, sequence, publish_start_ns)
                publish_end_ns = time.perf_counter_ns()
                if measured:
                    sent.append(
                        (
                            workload.name,
                            sequence,
                            workload.payload_bytes,
                            due_ns,
                            publish_start_ns,
                            publish_end_ns,
                        )
                    )
                    offered_bytes += workload.payload_bytes
                sequence_by_topic[workload.name] = current_sequence + 1
                if workload.rate_hz:
                    next_ns[workload.name] += int(1e9 / workload.rate_hz)
            return (time.perf_counter_ns() - start_ns) / 1e9

        run_window(spec.warmup_s, measured=False)
        sequence_by_topic = {topic.name: 0 for topic in spec.workload.topics}
        cpu = process.cpu_times()
        started_cpu = cpu.user + cpu.system
        (coordination / "measure-started").touch()
        measurement_s = run_window(spec.duration_s, measured=True)
        usage = _usage("publisher", started_cpu)
    except Exception as caught:
        error = str(caught)
    finally:
        if bus is not None:
            bus.stop()
        _write(
            output,
            {
                "sent": sent,
                "measurement_s": measurement_s if error is None else None,
                "usage": usage or _usage("publisher", started_cpu),
                "error": error,
            },
        )
        (coordination / "done").touch()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("role", choices=("publisher", "subscriber"))
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--coordination", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receiver", type=int, default=0)
    args = parser.parse_args()
    args.coordination.mkdir(parents=True, exist_ok=True)
    spec = _load_spec(args.spec)
    if args.role == "publisher":
        run_publisher(spec, args.coordination, args.output)
    else:
        run_subscriber(spec, args.receiver, args.coordination, args.output)


if __name__ == "__main__":
    main()
