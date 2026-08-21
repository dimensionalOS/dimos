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

"""One publish/receive throughput measurement, shared by every caller.

Both the pytest transport benchmark (``tool_benchmark.test_throughput``) and the
network-degradation eval (``dimos.eval.network``) call :func:`measure_throughput`
so there is exactly one measurement implementation — a change to how loss,
latency, or CPU is measured lands in both at once.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from dimos.protocol.pubsub.benchmark.type import BenchmarkResult


def transport_name(context_name: str) -> str:
    """Display name from a pubsub-context function name.

    "lcm_pubsub_channel" -> "LCM"; "zenoh_peers_pubsub_channel" -> "ZenohPeers".
    """
    prefix = context_name.replace("_pubsub_channel", "").replace("_", " ")
    return prefix.upper() if len(prefix) <= 3 else prefix.title().replace(" ", "")


def measure_throughput(
    pubsub: Any,
    topic: Any,
    msg: Any,
    name: str,
    msg_size: int,
    *,
    duration: float = 1.0,
    max_messages: int = 5000,
    receive_timeout: float = 1.0,
) -> BenchmarkResult:
    """Publish ``msg`` as fast as possible for ``duration`` and record the result.

    A subscriber counts arrivals; we publish until the window closes or
    ``max_messages`` is hit, then wait up to ``receive_timeout`` for in-flight
    messages to drain. Latency = the drain wait (0 == all arrived while
    publishing); loss is derived from sent vs received. CPU is sampled over the
    publish window only, so a lossy transport's idle receive wait doesn't skew it.

    ``pubsub`` must already be started; the caller owns its lifecycle (and, for
    the benchmark, the message generation / size-skip that happens before
    connecting).
    """
    import psutil

    received_count = 0
    target_count = [0]  # list so the publish loop can update it post-hoc
    lock = threading.Lock()
    all_received = threading.Event()

    def callback(_message: Any, _topic: Any) -> None:
        nonlocal received_count
        with lock:
            received_count += 1
            if target_count[0] > 0 and received_count >= target_count[0]:
                all_received.set()

    pubsub.subscribe(topic, callback)

    # Warmup: give DDS/ROS/zenoh time to establish their connection.
    time.sleep(0.1)
    target_count[0] = max_messages

    msgs_sent = 0
    process = psutil.Process()
    cpu_before = process.cpu_times()
    start = time.perf_counter()
    end_time = start + duration

    while time.perf_counter() < end_time and msgs_sent < max_messages:
        pubsub.publish(topic, msg)
        msgs_sent += 1
        if all_received.is_set():  # fast transports finish early
            break

    publish_end = time.perf_counter()
    cpu_after = process.cpu_times()
    cpu_seconds = (cpu_after.user - cpu_before.user) + (cpu_after.system - cpu_before.system)
    target_count[0] = msgs_sent

    with lock:
        if received_count >= msgs_sent:
            all_received.set()
    if not all_received.is_set():
        all_received.wait(timeout=receive_timeout)
    latency_end = time.perf_counter()

    with lock:
        final_received = received_count

    return BenchmarkResult(
        transport=name,
        duration=publish_end - start,
        msgs_sent=msgs_sent,
        msgs_received=final_received,
        msg_size_bytes=msg_size,
        receive_time=latency_end - publish_end,
        cpu_seconds=cpu_seconds,
    )
