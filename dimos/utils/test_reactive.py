# Copyright 2025 Dimensional Inc.
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

from collections.abc import Callable
import time
from typing import Any, TypeVar

import numpy as np
import pytest
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.utils.reactive import (
    backpressure,
    callback_to_observable,
    getter_ondemand,
    getter_streaming,
)


def measure_time(func: Callable[[], Any], iterations: int = 1) -> float:
    start_time = time.time()
    result = func()
    end_time = time.time()
    total_time = end_time - start_time
    return result, total_time


def assert_time(
    func: Callable[[], Any], assertion: Callable[[int], bool], assert_fail_msg=None
) -> None:
    [result, total_time] = measure_time(func)
    assert assertion(total_time), assert_fail_msg + f", took {round(total_time, 2)}s"
    return result


def min_time(
    func: Callable[[], Any], min_t: int, assert_fail_msg: str = "Function returned too fast"
):
    return assert_time(
        func, (lambda t: t >= min_t * 0.98), assert_fail_msg + f", min: {min_t} seconds"
    )


def max_time(func: Callable[[], Any], max_t: int, assert_fail_msg: str = "Function took too long"):
    return assert_time(func, (lambda t: t < max_t), assert_fail_msg + f", max: {max_t} seconds")


T = TypeVar("T")


def dispose_spy(source: rx.Observable[T]) -> rx.Observable[T]:
    state = {"active": 0}

    def factory(observer, scheduler=None):
        state["active"] += 1
        upstream = source.subscribe(observer, scheduler=scheduler)

        def _dispose() -> None:
            upstream.dispose()
            state["active"] -= 1

        return Disposable(_dispose)

    proxy = rx.create(factory)
    proxy.subs_number = lambda: state["active"]
    proxy.is_disposed = lambda: state["active"] == 0
    return proxy


def test_backpressure_handling() -> None:
    # Create a dedicated scheduler for this test to avoid thread leaks
    test_scheduler = ThreadPoolScheduler(max_workers=8)
    try:
        received_fast = []
        received_slow = []
        # Create an observable that emits numpy arrays instead of integers
        source = dispose_spy(
            rx.interval(0.1).pipe(ops.map(lambda i: np.array([i, i + 1, i + 2])), ops.take(50))
        )

        # Wrap with backpressure handling
        safe_source = backpressure(source, scheduler=test_scheduler)

        # Fast sub
        subscription1 = safe_source.subscribe(lambda x: received_fast.append(x))

        # Slow sub (shouldn't block above)
        subscription2 = safe_source.subscribe(lambda x: (time.sleep(0.25), received_slow.append(x)))

        time.sleep(2.5)

        subscription1.dispose()
        assert not source.is_disposed(), "Observable should not be disposed yet"
        subscription2.dispose()
        # Wait longer to ensure background threads finish processing
        # (the slow subscriber sleeps for 0.25s, so we need to wait at least that long)
        time.sleep(0.5)
        assert source.is_disposed(), "Observable should be disposed"

        # Check results
        print("Fast observer received:", len(received_fast), [arr[0] for arr in received_fast])
        print("Slow observer received:", len(received_slow), [arr[0] for arr in received_slow])

        # Fast observer should get all or nearly all items
        assert len(received_fast) > 15, (
            f"Expected fast observer to receive most items, got {len(received_fast)}"
        )

        # Slow observer should get fewer items due to backpressure handling
        assert len(received_slow) < len(received_fast), (
            "Slow observer should receive fewer items than fast observer"
        )
        # Specifically, processing at 0.25s means ~4 items per second, so expect 8-10 items
        assert 7 <= len(received_slow) <= 11, f"Expected 7-11 items, got {len(received_slow)}"

        # The slow observer should skip items (not process them in sequence)
        # We test this by checking that the difference between consecutive arrays is sometimes > 1
        has_skips = False
        for i in range(1, len(received_slow)):
            if received_slow[i][0] - received_slow[i - 1][0] > 1:
                has_skips = True
                break
        assert has_skips, "Slow observer should skip items due to backpressure"
    finally:
        # Always shutdown the scheduler to clean up threads
        test_scheduler.executor.shutdown(wait=True)


def test_getter_streaming_blocking() -> None:
    source = dispose_spy(
        rx.interval(0.2).pipe(ops.map(lambda i: np.array([i, i + 1, i + 2])), ops.take(50))
    )
    assert source.is_disposed()

    getter = min_time(
        lambda: getter_streaming(source),
        0.2,
        "Latest getter needs to block until first msg is ready",
    )
    assert np.array_equal(getter(), np.array([0, 1, 2])), (
        f"Expected to get the first array [0,1,2], got {getter()}"
    )

    time.sleep(0.5)
    assert getter()[0] >= 2, f"Expected array with first value >= 2, got {getter()}"
    time.sleep(0.5)
    assert getter()[0] >= 4, f"Expected array with first value >= 4, got {getter()}"

    getter.dispose()
    time.sleep(0.3)  # Wait for background interval timer threads to finish
    assert source.is_disposed(), "Observable should be disposed"


def test_getter_streaming_blocking_timeout() -> None:
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))
    with pytest.raises(Exception):
        getter = getter_streaming(source, timeout=0.1)
        getter.dispose()
    time.sleep(0.3)  # Wait for background interval timer threads to finish
    assert source.is_disposed()


def test_getter_streaming_nonblocking() -> None:
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))

    getter = max_time(
        lambda: getter_streaming(source, nonblocking=True),
        0.1,
        "nonblocking getter init shouldn't block",
    )
    min_time(getter, 0.1, "Expected for first value call to block if cache is empty")
    assert getter() == 0

    time.sleep(0.5)
    assert getter() >= 2, f"Expected value >= 2, got {getter()}"

    # sub is active
    assert not source.is_disposed()

    time.sleep(0.5)
    assert getter() >= 4, f"Expected value >= 4, got {getter()}"

    getter.dispose()
    time.sleep(0.3)  # Wait for background interval timer threads to finish
    assert source.is_disposed(), "Observable should be disposed"


def test_getter_streaming_nonblocking_timeout() -> None:
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))
    getter = getter_streaming(source, timeout=0.1, nonblocking=True)
    with pytest.raises(Exception):
        getter()

    assert not source.is_disposed(), "is not disposed, this is a job of the caller"

    # Clean up the subscription to avoid thread leak
    getter.dispose()
    time.sleep(0.3)  # Wait for background threads to finish
    assert source.is_disposed(), "Observable should be disposed after cleanup"


def test_getter_ondemand() -> None:
    # Create a controlled scheduler to avoid thread leaks from rx.interval
    test_scheduler = ThreadPoolScheduler(max_workers=4)
    try:
        source = dispose_spy(rx.interval(0.1, scheduler=test_scheduler).pipe(ops.take(50)))
        getter = getter_ondemand(source)
        assert source.is_disposed(), "Observable should be disposed"
        result = min_time(getter, 0.05)
        assert result == 0, f"Expected to get the first value of 0, got {result}"
        # Wait for background threads to clean up
        time.sleep(0.3)
        assert source.is_disposed(), "Observable should be disposed"
        result2 = getter()
        assert result2 == 0, f"Expected to get the first value of 0, got {result2}"
        assert source.is_disposed(), "Observable should be disposed"
        # Wait for threads to finish
        time.sleep(0.3)
    finally:
        # Explicitly shutdown the scheduler to clean up threads
        test_scheduler.executor.shutdown(wait=True)


def test_getter_ondemand_timeout() -> None:
    source = dispose_spy(rx.interval(0.2).pipe(ops.take(50)))
    getter = getter_ondemand(source, timeout=0.1)
    with pytest.raises(Exception):
        getter()
    assert source.is_disposed(), "Observable should be disposed"
    # Wait for background interval timer threads to finish
    time.sleep(0.3)


def test_callback_to_observable() -> None:
    # Test converting a callback-based API to an Observable
    received = []
    callback = None

    # Mock start function that captures the callback
    def start_fn(cb) -> str:
        nonlocal callback
        callback = cb
        return "start_result"

    # Mock stop function
    stop_called = False

    def stop_fn(cb) -> None:
        nonlocal stop_called
        stop_called = True

    # Create observable from callback
    observable = callback_to_observable(start_fn, stop_fn)

    # Subscribe to the observable
    subscription = observable.subscribe(lambda x: received.append(x))

    # Verify start was called and we have access to the callback
    assert callback is not None

    # Simulate callback being triggered with different messages
    callback("message1")
    callback(42)
    callback({"key": "value"})

    # Check that all messages were received
    assert received == ["message1", 42, {"key": "value"}]

    # Dispose subscription and check that stop was called
    subscription.dispose()
    assert stop_called, "Stop function should be called on dispose"
