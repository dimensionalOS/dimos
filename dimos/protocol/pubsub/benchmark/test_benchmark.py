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

from collections.abc import Generator
import threading
import time
from typing import Any

import pytest

from dimos.protocol.pubsub.benchmark.testdata import testdata
from dimos.protocol.pubsub.benchmark.type import (
    BenchmarkResult,
    BenchmarkResults,
    Case,
    MsgGen,
    PubSubContext,
)

# Message sizes for throughput benchmarking (powers of 2 from 64B to 10MB)
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
    1048576 * 2,
    1048576 * 5,
    1048576 * 10,
]

# Benchmark duration in seconds
BENCH_DURATION = 1.0

# Max messages to send per test (prevents overwhelming slower transports)
MAX_MESSAGES = 5000

# Max time to wait for in-flight messages after publishing stops
RECEIVE_TIMEOUT = 1.0


def size_id(size: int) -> str:
    """Convert byte size to human-readable string for test IDs."""
    if size >= 1048576:
        return f"{size // 1048576}MB"
    if size >= 1024:
        return f"{size // 1024}KB"
    return f"{size}B"


def pubsub_id(testcase: Case[Any, Any]) -> str:
    """Extract pubsub implementation name from context manager function name."""
    name: str = testcase.pubsub_context.__name__
    # Convert e.g. "lcm_pubsub_channel" -> "LCM", "memory_pubsub_channel" -> "Memory"
    prefix = name.replace("_pubsub_channel", "").replace("_", " ")
    return prefix.upper() if len(prefix) <= 3 else prefix.title().replace(" ", "")


@pytest.fixture(scope="module")
def benchmark_results() -> Generator[BenchmarkResults, None, None]:
    """Module-scoped fixture to collect benchmark results."""
    results = BenchmarkResults()
    yield results
    results.print_summary()
    results.print_heatmap()
    results.print_bandwidth_heatmap()
    results.print_latency_heatmap()


@pytest.mark.tool
@pytest.mark.parametrize("msg_size", MSG_SIZES, ids=[size_id(s) for s in MSG_SIZES])
@pytest.mark.parametrize("pubsub_context, msggen", testdata, ids=[pubsub_id(t) for t in testdata])
def test_throughput(
    pubsub_context: PubSubContext[Any, Any],
    msggen: MsgGen[Any, Any],
    msg_size: int,
    benchmark_results: BenchmarkResults,
) -> None:
    """Measure throughput for publishing and receiving messages over a fixed duration."""
    with pubsub_context() as pubsub:
        topic, msg = msggen(msg_size)
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
        pubsub.subscribe(topic, callback)

        # Warmup: give DDS/ROS time to establish connection
        time.sleep(0.1)

        # Set target so callback can signal when all received
        target_count[0] = MAX_MESSAGES

        # Publish messages until time limit, max messages, or all received
        msgs_sent = 0
        start = time.perf_counter()
        end_time = start + BENCH_DURATION

        while time.perf_counter() < end_time and msgs_sent < MAX_MESSAGES:
            pubsub.publish(topic, msg)
            msgs_sent += 1
            # Check if all already received (fast transports)
            if all_received.is_set():
                break

        publish_end = time.perf_counter()
        target_count[0] = msgs_sent  # Update to actual sent count

        # Check if already done, otherwise wait up to RECEIVE_TIMEOUT
        with lock:
            if received_count >= msgs_sent:
                all_received.set()

        if not all_received.is_set():
            all_received.wait(timeout=RECEIVE_TIMEOUT)
        latency_end = time.perf_counter()

        with lock:
            final_received = received_count

        # Latency: how long we waited after publishing for messages to arrive
        # 0 = all arrived during publishing, 1000ms = hit timeout (loss occurred)
        latency = latency_end - publish_end

        # Record result (duration is publish time only for throughput calculation)
        # Extract transport name from context manager function name
        ctx_name = pubsub_context.__name__
        prefix = ctx_name.replace("_pubsub_channel", "").replace("_", " ")
        transport_name = prefix.upper() if len(prefix) <= 3 else prefix.title().replace(" ", "")
        result = BenchmarkResult(
            transport=transport_name,
            duration=publish_end - start,
            msgs_sent=msgs_sent,
            msgs_received=final_received,
            msg_size_bytes=msg_size,
            receive_time=latency,
        )
        benchmark_results.add(result)

        # Warn if significant message loss (but don't fail - benchmark records the data)
        loss_pct = (1 - final_received / msgs_sent) * 100 if msgs_sent > 0 else 0
        if loss_pct > 10:
            import warnings

            warnings.warn(
                f"{transport_name} {msg_size}B: {loss_pct:.1f}% message loss "
                f"({final_received}/{msgs_sent})",
                stacklevel=2,
            )
