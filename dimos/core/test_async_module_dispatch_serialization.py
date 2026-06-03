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

import asyncio
import itertools
from queue import Empty, Queue
import time
from typing import Any

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import pLCMTransport


class BurstModule(Module):
    """Slow handler that records a (value, start, end) tuple per invocation."""

    a: In[int]
    record: Out[dict]

    async def handle_a(self, value: int) -> None:
        start = time.monotonic()
        await asyncio.sleep(0.05)
        end = time.monotonic()
        self.record.publish({"value": value, "start": start, "end": end})


@pytest.fixture
def start_burst_module():
    blueprint = BurstModule.blueprint()
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def burst_a_transport():
    tr = pLCMTransport("/a")
    tr.start()
    yield tr
    tr.stop()


@pytest.fixture
def burst_record_transport():
    tr = pLCMTransport("/record")
    tr.start()
    yield tr
    tr.stop()


def _drain(queue: Queue, settle_timeout: float = 0.5) -> list[Any]:
    items: list[Any] = []
    while True:
        try:
            items.append(queue.get(timeout=settle_timeout))
        except Empty:
            return items


def test_bursts_are_coalesced_and_handler_is_serialized(
    start_burst_module, burst_a_transport, burst_record_transport
):
    """Publishing 100 messages in a tight loop while the handler sleeps 50ms
    must (a) coalesce (the handler is invoked far fewer than 100 times),
    (b) eventually deliver the most recently published value, and
    (c) never run two handler invocations concurrently."""
    queue: Queue = Queue()
    burst_record_transport.subscribe(queue.put)

    n = 100
    for i in range(n):
        burst_a_transport.publish(i)

    records = _drain(queue, settle_timeout=2.0)

    # Coalescing actually happened.
    assert 0 < len(records) < n, f"expected coalescing, got {len(records)} records"

    # The most recently published value eventually reaches the handler.
    assert records[-1]["value"] == n - 1, (
        f"last record {records[-1]['value']} should equal final published value {n - 1}"
    )

    # No two recorded [start, end] intervals overlap (handler is serial).
    intervals = sorted((r["start"], r["end"]) for r in records)
    for (_, prev_end), (next_start, _) in itertools.pairwise(intervals):
        assert next_start >= prev_end, (
            f"overlapping handler intervals: prev_end={prev_end}, next_start={next_start}"
        )


class InterleaveModule(Module):
    """Handler that yields between writing and reading a per-instance marker."""

    a: In[int]
    record: Out[dict]

    _marker: int = -1

    async def handle_a(self, value: int) -> None:
        self._marker = value
        # Yield to the loop. Without serialization, another invocation could
        # run here and overwrite _marker before this coroutine resumes.
        await asyncio.sleep(0)
        self.record.publish({"value": value, "marker": self._marker})


@pytest.fixture
def start_interleave_module():
    blueprint = InterleaveModule.blueprint()
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def interleave_a_transport():
    tr = pLCMTransport("/a")
    tr.start()
    yield tr
    tr.stop()


@pytest.fixture
def interleave_record_transport():
    tr = pLCMTransport("/record")
    tr.start()
    yield tr
    tr.stop()


def test_handler_does_not_interleave_across_awaits(
    start_interleave_module, interleave_a_transport, interleave_record_transport
):
    """No invocation of `handle_a` may observe a marker written by a different
    invocation (the dispatcher must serialize handler execution across `await`
    points)."""
    queue: Queue = Queue()
    interleave_record_transport.subscribe(queue.put)

    for i in range(50):
        interleave_a_transport.publish(i)

    records = _drain(queue, settle_timeout=1.0)
    assert records, "expected at least one record"

    for r in records:
        assert r["value"] == r["marker"], (
            f"marker {r['marker']} differs from value {r['value']} — "
            "another handler invocation overwrote our state mid-handler"
        )


class CleanupModule(Module):
    """Handler that sleeps for a long time so we can stop the coordinator
    while the handler is mid-await."""

    a: In[int]

    async def handle_a(self, value: int) -> None:
        await asyncio.sleep(5.0)


@pytest.fixture
def cleanup_a_transport():
    tr = pLCMTransport("/a")
    tr.start()
    yield tr
    tr.stop()


def test_stop_cancels_in_flight_handler(cleanup_a_transport):
    """Stopping the coordinator while a handler is awaiting must complete
    quickly. The dispatcher cancels the handler instead of waiting for it."""
    blueprint = CleanupModule.blueprint()
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        cleanup_a_transport.publish(1)
        time.sleep(0.1)  # let the handler enter its sleep
        start = time.monotonic()
        coordinator.stop()
        elapsed = time.monotonic() - start
    except BaseException:
        coordinator.stop()
        raise

    # Without cancellation, stop() would either hang or be bounded only by the
    # 5s asyncio.sleep. The dispatcher cancels the task synchronously.
    assert elapsed < 3.0, f"stop() took {elapsed:.2f}s (handler not cancelled)"
