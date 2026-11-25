# Copyright 2025-2026 Dimensional Inc.
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

"""Local observer microbenchmark. No robot, network traffic or synthetic latency claims.

Run in separate processes with DIMOS_DIAGNOSTICS=off, metrics and trace. This
measures a small in-process path; it cannot qualify a complete camera/control stack.
"""

import argparse
import json
import math
import os
from pathlib import Path
import time

from dimos.core.stream import Out
from dimos.diagnostics.runtime import get_runtime, span


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--hz", type=float, default=200)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 1 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("samples and rate must be positive")
    stream: Out[int] = Out(int, "benchmark")

    def receive(value: int) -> None:
        with span("benchmark.compute"):
            sum(range(value))

    unsubscribe = stream.subscribe(receive)
    stream.publish(64)  # Exclude one-time observer initialization from hot-path latency.
    runtime = get_runtime()
    timings = []
    started_cpu, started = time.process_time(), time.monotonic()
    deadline = started
    try:
        for _ in range(args.samples):
            begin = time.perf_counter_ns()
            stream.publish(64)
            timings.append(time.perf_counter_ns() - begin)
            deadline += 1 / args.hz
            time.sleep(max(0, deadline - time.monotonic()))
    finally:
        unsubscribe()
        if runtime:
            runtime.close()
    cpu = time.process_time() - started_cpu
    elapsed = time.monotonic() - started
    timings.sort()
    result = {
        "scope": "one in-process publication and callback; includes observer worker CPU and final flush",
        "mode": os.environ.get("DIMOS_DIAGNOSTICS", "off"),
        "samples": args.samples,
        "target_hz": args.hz,
        "publish_p50_us": timings[math.ceil(len(timings) * 0.5) - 1] / 1000,
        "publish_p99_us": timings[math.ceil(len(timings) * 0.99) - 1] / 1000,
        "process_cpu_s": cpu,
        "elapsed_s": elapsed,
        "observer_drops": runtime.collector.dropped if runtime else 0,
        "observer_io_errors": runtime.collector.errors if runtime else 0,
    }
    with args.output.open("x") as file:
        json.dump(result, file, indent=2)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
