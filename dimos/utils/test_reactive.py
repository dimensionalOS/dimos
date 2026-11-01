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

from collections.abc import Callable
import gc
from queue import Queue
import sys
import threading
import time
from typing import Any, TypeVar
import weakref

import numpy as np
import pytest
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable, Disposable
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import Subject

from dimos.utils.reactive import (
    _LatestBackpressureObserver,
    backpressure,
    callback_to_observable,
    getter_ondemand,
    getter_streaming,
    iter_observable,
)

_IS_MACOS = sys.platform == "darwin"


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


class BackpressureResources:
    """Own subscriptions, callback gates, and threads even when a test fails."""

    def __init__(self):
        self.scheduler = ThreadPoolScheduler(max_workers=4)
        self.disposables = CompositeDisposable()
        self.threads = []
        self.waiting = Queue()
        self.submitted = Queue()
        self.releases = []

    def own(self, disposable):
        self.disposables.add(disposable)
        return disposable

    def release_event(self):
        event = threading.Event()
        self.releases.append(event)
        return event

    def start(self, target):
        thread = threading.Thread(target=target)
        self.threads.append(thread)
        thread.start()
        return thread


def join_worker(worker):
    # Shorter than the former five-second idle timeout: expiry cannot pass this check.
    worker.join(timeout=2.0)
    assert not worker.is_alive(), "backpressure worker did not stop promptly"


@pytest.fixture
def backpressure_resources(monkeypatch):
    resources = BackpressureResources()
    run = _LatestBackpressureObserver._run
    submit = _LatestBackpressureObserver._submit

    def tracked_run(observer):
        worker = threading.current_thread()
        resources.threads.append(worker)
        wait = observer._cond.wait

        def tracked_wait(timeout=None):
            # Signal under the condition lock, so the next submission cannot
            # overtake entry into wait(). Do not retain observers or payloads.
            resources.waiting.put((worker, timeout))
            return wait(timeout)

        with monkeypatch.context() as patch:
            patch.setattr(observer._cond, "wait", tracked_wait)
            run(observer)

    def tracked_submit(observer, kind, payload):
        submit(observer, kind, payload)
        resources.submitted.put(kind)

    monkeypatch.setattr(_LatestBackpressureObserver, "_run", tracked_run)
    monkeypatch.setattr(_LatestBackpressureObserver, "_submit", tracked_submit)
    try:
        yield resources
    finally:
        for release in resources.releases:
            release.set()
        resources.disposables.dispose()
        resources.scheduler.executor.shutdown(wait=True)
        for thread in resources.threads:
            thread.join(timeout=2.0)
        assert all(not thread.is_alive() for thread in resources.threads)


class ThreadReportingSink:
    def __init__(self, delivered):
        self.delivered = delivered

    def on_next(self, value):
        self.delivered.put(threading.current_thread())


def test_backpressure_handling(backpressure_resources) -> None:
    """Subscribers own independent workers and can unsubscribe independently."""
    resources = backpressure_resources
    source = resources.own(Subject())
    shared = backpressure(source, scheduler=resources.scheduler)
    first = Queue()
    second = Queue()
    release = resources.release_event()

    def slow(value):
        second.put((value, threading.current_thread()))
        assert release.wait(5.0)

    subscription = resources.own(
        shared.subscribe(lambda value: first.put((value, threading.current_thread())))
    )
    slow_subscription = resources.own(shared.subscribe(slow))
    assert resources.threads == []  # Delivery workers start lazily.

    source.on_next(0)
    first_value, first_worker = first.get(timeout=2.0)
    second_value, second_worker = second.get(timeout=2.0)
    assert (first_value, second_value) == (0, 0)
    assert first_worker is not second_worker

    source.on_next(1)
    assert first.get(timeout=2.0) == (1, first_worker)
    subscription.dispose()
    join_worker(first_worker)
    release.set()
    assert second.get(timeout=2.0) == (1, second_worker)

    source.on_next(2)
    assert second.get(timeout=2.0) == (2, second_worker)
    assert first.empty()

    slow_subscription.dispose()
    join_worker(second_worker)
    assert source.observers == []  # The last unsubscribe disconnects the source.


def test_backpressure_single_worker_thread(backpressure_resources) -> None:
    """A held callback and the latest of a burst use the same resident worker."""
    resources = backpressure_resources
    received = Queue()
    release = resources.release_event()

    def on_next(value):
        received.put((value, threading.current_thread()))
        if value == 0:
            assert release.wait(5.0)

    observer = resources.own(_LatestBackpressureObserver(rx.Observer(on_next)))
    observer.on_next(0)
    first_value, worker = received.get(timeout=2.0)
    for value in range(1, 200):
        observer.on_next(value)
    release.set()

    assert first_value == 0
    assert received.get(timeout=2.0) == (199, worker)
    assert resources.waiting.get(timeout=2.0) == (worker, None)
    assert received.empty()


def test_backpressure_releases_delivered_payload_while_idle(backpressure_resources) -> None:
    resources = backpressure_resources
    received = Queue()
    # Exercise worker retention separately from backpressure()'s replay cache.
    observer = resources.own(
        _LatestBackpressureObserver(rx.Observer(ThreadReportingSink(received).on_next))
    )
    payload = np.zeros(1024 * 1024, dtype=np.uint8)
    payload_ref = weakref.ref(payload)
    observer.on_next(payload)
    worker = received.get(timeout=2.0)
    assert resources.waiting.get(timeout=2.0) == (worker, None)

    del payload
    gc.collect()

    assert payload_ref() is None
    assert worker.is_alive()


def test_backpressure_disposal_releases_workers_owners_and_payloads(backpressure_resources) -> None:
    resources = backpressure_resources
    delivered = Queue()
    owners = []
    payloads = []
    workers = []
    for _ in range(3):
        source = resources.own(Subject())
        sink = ThreadReportingSink(delivered)
        payload = np.zeros(1024 * 1024, dtype=np.uint8)
        owners.append(weakref.ref(sink))
        payloads.append(weakref.ref(payload))
        subscription = resources.own(
            backpressure(source, scheduler=resources.scheduler).subscribe(sink.on_next)
        )
        source.on_next(payload)
        worker = delivered.get(timeout=2.0)
        workers.append(worker)
        assert resources.waiting.get(timeout=2.0) == (worker, None)
        subscription.dispose()
        source.dispose()
        del source, sink, payload, subscription

    # Drop fixture ownership too; Rx disposables and replay caches can retain callbacks/data.
    resources.disposables.clear()
    resources.scheduler.executor.shutdown(wait=True)
    for worker in workers:
        join_worker(worker)
    gc.collect()

    assert all(owner() is None for owner in owners)
    assert all(payload() is None for payload in payloads)


def test_backpressure_disposal_during_callback_discards_pending(backpressure_resources) -> None:
    resources = backpressure_resources
    source = resources.own(Subject())
    received = []
    started = Queue()
    release = resources.release_event()
    disposed = threading.Event()

    def on_next(value):
        received.append(value)
        started.put(threading.current_thread())
        assert release.wait(5.0)

    subscription = resources.own(
        backpressure(source, scheduler=resources.scheduler).subscribe(on_next)
    )
    source.on_next(0)
    worker = started.get(timeout=2.0)
    assert resources.submitted.get(timeout=2.0) == "next"
    pending = np.zeros(1024 * 1024, dtype=np.uint8)
    pending_ref = weakref.ref(pending)
    source.on_next(pending)
    assert resources.submitted.get(timeout=2.0) == "next"

    def dispose():
        subscription.dispose()
        disposed.set()

    disposer = resources.start(dispose)
    assert disposed.wait(2.0), "disposal must not wait for the running callback"
    join_worker(disposer)
    resources.disposables.clear()
    del source, subscription, pending
    gc.collect()
    assert pending_ref() is None
    assert worker.is_alive()

    release.set()
    join_worker(worker)
    assert received == [0]


def test_backpressure_one_shot_disposes_inside_worker(backpressure_resources) -> None:
    resources = backpressure_resources
    source = resources.own(Subject())
    received = Queue()
    completed = threading.Event()
    resources.own(
        backpressure(source, scheduler=resources.scheduler)
        .pipe(ops.first())
        .subscribe(
            lambda value: received.put((value, threading.current_thread())),
            on_completed=completed.set,
        )
    )

    source.on_next(1)
    value, worker = received.get(timeout=2.0)
    assert completed.wait(2.0)
    join_worker(worker)
    source.on_next(2)

    assert value == 1
    assert received.empty()
    assert resources.threads == [worker]


def test_backpressure_racing_submission_cannot_resurrect_worker(
    backpressure_resources, monkeypatch
) -> None:
    resources = backpressure_resources
    source = resources.own(Subject())
    received = Queue()
    subscription = resources.own(
        backpressure(source, scheduler=resources.scheduler).subscribe(
            ThreadReportingSink(received).on_next
        )
    )
    source.on_next(0)
    worker = received.get(timeout=2.0)
    assert resources.waiting.get(timeout=2.0) == (worker, None)
    assert resources.submitted.get(timeout=2.0) == "next"
    entered = threading.Event()
    release = resources.release_event()
    submit = _LatestBackpressureObserver._submit

    def paused_submit(observer, kind, payload):
        entered.set()
        assert release.wait(5.0)
        submit(observer, kind, payload)

    monkeypatch.setattr(_LatestBackpressureObserver, "_submit", paused_submit)
    pending = np.zeros(1024 * 1024, dtype=np.uint8)
    pending_ref = weakref.ref(pending)
    source.on_next(pending)
    assert entered.wait(2.0)

    subscription.dispose()
    subscription.dispose()
    join_worker(worker)  # The idle worker is gone before the paused submission resumes.
    release.set()
    assert resources.submitted.get(timeout=2.0) == "next"
    resources.disposables.clear()
    resources.scheduler.executor.shutdown(wait=True)
    del source, subscription, pending
    gc.collect()

    assert resources.threads == [worker]
    assert received.empty()
    assert pending_ref() is None


def test_backpressure_error_delivered_once(backpressure_resources) -> None:
    """A busy observer keeps only the latest error, delivered after the callback ends the worker."""
    resources = backpressure_resources
    started = Queue()
    received = []
    errors = []
    release = resources.release_event()

    def on_next(value):
        received.append(value)
        started.put(threading.current_thread())
        assert release.wait(5.0)

    observer = resources.own(_LatestBackpressureObserver(rx.Observer(on_next, errors.append)))
    observer.on_next(1)
    worker = started.get(timeout=2.0)
    observer.on_next(2)
    observer.on_error(RuntimeError("a"))
    observer.on_error(RuntimeError("b"))
    assert errors == []
    release.set()
    join_worker(worker)

    assert received == [1]
    assert [str(error) for error in errors] == ["b"]


@pytest.mark.parametrize("terminal", ["on_completed", "on_error"])
def test_backpressure_terminal_releases_idle_worker(backpressure_resources, terminal) -> None:
    resources = backpressure_resources
    source = resources.own(Subject())
    received = Queue()
    notifications = Queue()
    resources.own(
        backpressure(source, scheduler=resources.scheduler).subscribe(
            ThreadReportingSink(received).on_next,
            on_error=lambda error: notifications.put(str(error)),
            on_completed=lambda: notifications.put("completed"),
        )
    )
    source.on_next(1)
    worker = received.get(timeout=2.0)
    assert resources.waiting.get(timeout=2.0) == (worker, None)

    if terminal == "on_error":
        source.on_error(RuntimeError("failed"))
    else:
        source.on_completed()
    notification = notifications.get(timeout=2.0)
    join_worker(worker)

    assert notification == {"on_error": "failed", "on_completed": "completed"}[terminal]
    assert notifications.empty()


def test_backpressure_completion_stays_immediate_during_callback(backpressure_resources) -> None:
    resources = backpressure_resources
    source = resources.own(Subject())
    release = resources.release_event()
    started = Queue()
    completed = threading.Event()
    received = []

    def on_next(value):
        received.append(value)
        started.put(threading.current_thread())
        assert release.wait(5.0)

    resources.own(
        backpressure(source, scheduler=resources.scheduler).subscribe(
            on_next, on_completed=completed.set
        )
    )
    source.on_next(0)
    worker = started.get(timeout=2.0)
    assert resources.submitted.get(timeout=2.0) == "next"
    source.on_next(1)
    assert resources.submitted.get(timeout=2.0) == "next"
    source.on_completed()

    assert completed.wait(2.0)
    assert worker.is_alive()
    release.set()
    join_worker(worker)
    assert received == [0]


def test_backpressure_callback_exception_allows_later_recovery(
    backpressure_resources, monkeypatch
) -> None:
    resources = backpressure_resources
    source = resources.own(Subject())
    received = Queue()
    failures = Queue()
    release = resources.release_event()
    monkeypatch.setattr(threading, "excepthook", lambda args: failures.put(str(args.exc_value)))

    def on_next(value):
        received.put((value, threading.current_thread()))
        if value == 0:
            assert release.wait(5.0)
            raise RuntimeError("callback failed")

    resources.own(backpressure(source, scheduler=resources.scheduler).subscribe(on_next))
    source.on_next(0)
    first_value, first_worker = received.get(timeout=2.0)
    assert resources.submitted.get(timeout=2.0) == "next"
    source.on_next(1)
    assert resources.submitted.get(timeout=2.0) == "next"
    release.set()
    join_worker(first_worker)
    assert failures.get(timeout=2.0) == "callback failed"
    assert received.empty()

    source.on_next(2)
    recovered_value, recovered_worker = received.get(timeout=2.0)
    assert resources.waiting.get(timeout=2.0) == (recovered_worker, None)

    assert (first_value, recovered_value) == (0, 2)
    assert recovered_worker is not first_worker


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

    time.sleep(1.5 if _IS_MACOS else 0.5)
    assert getter()[0] >= 2, f"Expected array with first value >= 2, got {getter()}"
    time.sleep(1.5 if _IS_MACOS else 0.5)
    assert getter()[0] >= 4, f"Expected array with first value >= 4, got {getter()}"

    getter.dispose()
    time.sleep(0.3)  # Wait for background interval timer threads to finish
    assert source.is_disposed(), "Observable should be disposed"


@pytest.mark.skipif_macos_bug
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

    time.sleep(1.5 if _IS_MACOS else 0.5)
    assert getter() >= 2, f"Expected value >= 2, got {getter()}"

    # sub is active
    assert not source.is_disposed()

    time.sleep(1.5 if _IS_MACOS else 0.5)
    assert getter() >= 4, f"Expected value >= 4, got {getter()}"

    getter.dispose()
    time.sleep(0.3)  # Wait for background interval timer threads to finish
    assert source.is_disposed(), "Observable should be disposed"


def test_getter_streaming_nonblocking_timeout() -> None:
    # macOS: nonblocking subscription starts buffering immediately; first item from
    # rx.interval(0.2) can arrive in <0.1s due to scheduling, racing the timeout.
    _interval = 0.5 if _IS_MACOS else 0.2
    source = dispose_spy(rx.interval(_interval).pipe(ops.take(50)))
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


@pytest.mark.skipif_macos_bug
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


def test_iter_observable() -> None:
    source = dispose_spy(rx.of(1, 2, 3, 4, 5))

    result = list(iter_observable(source))

    assert result == [1, 2, 3, 4, 5]
    assert source.is_disposed(), "Observable should be disposed after iteration"
