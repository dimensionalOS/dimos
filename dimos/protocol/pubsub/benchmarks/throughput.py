#!/usr/bin/env python3

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

"""Compare pubsub throughput, CPU usage, receive drain time, and loss.

Run with python -m dimos.protocol.pubsub.benchmarks.throughput --help.
"""

import argparse
import math
import os
import threading
import time
from typing import Any

import psutil

from dimos.protocol.pubsub.benchmarks.cases import Case, UnsupportedMessageSizeError, cases
from dimos.protocol.pubsub.benchmarks.results import BenchmarkResult, BenchmarkResults

MSG_SIZES = [
    64,
    256,
    1024,
    4096,
    16384,
    65536,
    262144,
    524288,
    1048576,
    2 * 1048576,
    5 * 1048576,
    10 * 1048576,
]


def measure(
    case: Case[Any, Any],
    msg_size: int,
    duration: float = 1.0,
    max_messages: int = 5000,
    receive_timeout: float = 1.0,
) -> BenchmarkResult:
    """Measure one transport and payload size over a bounded publish window."""
    topic, msg = case.msg_gen(msg_size)
    with case.pubsub_context() as pubsub:
        received_count = 0
        target_count = [0]  # Use list to allow modification after publish loop
        lock = threading.Lock()
        all_received = threading.Event()

        def callback(message: Any, _topic: Any) -> None:
            nonlocal received_count
            with lock:
                received_count += 1
                if target_count[0] > 0 and received_count >= target_count[0]:
                    all_received.set()

        # Subscribe
        unsubscribe = pubsub.subscribe(topic, callback)

        try:
            # Warmup: give DDS/ROS time to establish connection
            time.sleep(0.1)

            # Set target so callback can signal when all received
            target_count[0] = max_messages

            # CPU sampled over the publish window only, so lossy transports' receive
            # idle doesn't skew it. psutil counts in-process threads but not kernel
            # softirq or external daemons, so it does not measure total system CPU.
            msgs_sent = 0
            process = psutil.Process()
            cpu_before = process.cpu_times()
            start = time.perf_counter()
            end_time = start + duration

            while time.perf_counter() < end_time and msgs_sent < max_messages:
                pubsub.publish(topic, msg)
                msgs_sent += 1
                # Check if all already received (fast transports)
                if all_received.is_set():
                    break

            publish_end = time.perf_counter()
            cpu_after = process.cpu_times()
            cpu_seconds = (cpu_after.user - cpu_before.user) + (
                cpu_after.system - cpu_before.system
            )
            target_count[0] = msgs_sent  # Update to actual sent count

            # Check if already done, otherwise wait up to receive_timeout
            with lock:
                if received_count >= msgs_sent:
                    all_received.set()

            if not all_received.is_set():
                all_received.wait(timeout=receive_timeout)
            latency_end = time.perf_counter()

            with lock:
                final_received = received_count

            # Latency: how long we waited after publishing for messages to arrive
            # 0 = all arrived during publishing, 1000ms = hit timeout (loss occurred)
            latency = latency_end - publish_end

            # Record result (duration is publish time only for throughput calculation)
            return BenchmarkResult(
                transport=case.label,
                duration=publish_end - start,
                msgs_sent=msgs_sent,
                msgs_received=final_received,
                msg_size_bytes=msg_size,
                receive_time=latency,
                cpu_seconds=cpu_seconds,
            )
        finally:
            unsubscribe()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--transport",
        action="append",
        choices=[case.name for case in cases],
        help="backend to measure, repeat for several (default: local backends)",
    )
    parser.add_argument(
        "--sizes", type=int, nargs="+", default=MSG_SIZES, help="payload sizes in bytes"
    )
    parser.add_argument("--duration", type=float, default=1.0, help="publish seconds per case")
    parser.add_argument("--max-messages", type=int, default=5000)
    parser.add_argument(
        "--receive-timeout",
        type=float,
        default=float(os.environ.get("DIMOS_BENCH_RECEIVE_TIMEOUT_S", "1.0")),
    )
    parser.add_argument("--list", action="store_true", help="list available transports and exit")
    args = parser.parse_args()
    if args.list:
        print("\n".join(case.name for case in cases))
        return
    if any(size <= 0 for size in args.sizes) or args.max_messages <= 0:
        parser.error("sizes and max-messages must be positive")
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be positive and finite")
    if not math.isfinite(args.receive_timeout) or args.receive_timeout < 0:
        parser.error("receive-timeout must be nonnegative and finite")
    selected = (
        [case for case in cases if case.name in args.transport]
        if args.transport
        else [case for case in cases if case.name != "webrtc_cloudflare"]
    )
    results = BenchmarkResults()
    try:
        for case in selected:
            for size in args.sizes:
                print(f"Measuring {case.name}: {size} bytes", flush=True)
                try:
                    result = measure(
                        case, size, args.duration, args.max_messages, args.receive_timeout
                    )
                except UnsupportedMessageSizeError as exc:
                    print(f"Skipped: {exc}")
                    continue
                results.add(result)
    finally:
        results.print_summary()
        results.print_heatmap()
        results.print_bandwidth_heatmap()
        results.print_cpu_heatmap()
        results.print_latency_heatmap()
        results.print_loss_heatmap()


if __name__ == "__main__":
    main()
