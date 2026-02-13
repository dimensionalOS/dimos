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

from collections.abc import Callable, Generator
from queue import Queue
import threading
from typing import Any, Generic, TypeVar

import reactivex as rx
from reactivex import operators as ops
from reactivex.abc import DisposableBase
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.rxpy_backpressure import BackPressure
from dimos.utils.threadpool import get_scheduler

T = TypeVar("T")


# Observable ─► ReplaySubject─► observe_on(pool) ─► backpressure.latest ─► sub1 (fast)
#                           ├──► observe_on(pool) ─► backpressure.latest ─► sub2 (slow)
#                           └──► observe_on(pool) ─► backpressure.latest ─► sub3 (slower)
def backpressure(
    observable: Observable[T],
    scheduler: ThreadPoolScheduler | None = None,
    drop_unprocessed: bool = True,
) -> Observable[T]:
    if scheduler is None:
        scheduler = get_scheduler()

    # hot, latest-cached core (similar to replay subject)
    core = observable.pipe(
        ops.replay(buffer_size=1),
        ops.ref_count(),  # Shared but still synchronous!
    )

    # per-subscriber factory
    def per_sub():  # type: ignore[no-untyped-def]
        # Move processing to thread pool
        base = core.pipe(ops.observe_on(scheduler))

        # optional back-pressure handling
        if not drop_unprocessed:
            return base

        def _subscribe(observer, sch=None):  # type: ignore[no-untyped-def]
            return base.subscribe(BackPressure.LATEST(observer), scheduler=sch)

        return rx.create(_subscribe)

    # each `.subscribe()` call gets its own async backpressure chain
    return rx.defer(lambda *_: per_sub())  # type: ignore[no-untyped-call]


class LatestReader(DisposableBase, Generic[T]):
    """A callable object that returns the latest value from an observable."""

    def __init__(self, initial_value: T, subscription, connection=None) -> None:  # type: ignore[no-untyped-def]
        self._value = initial_value
        self._subscription = subscription
        self._connection = connection

    def __call__(self) -> T:
        """Return the latest value from the observable."""
        return self._value

    def dispose(self) -> None:
        """Dispose of the subscription to the observable."""
        self._subscription.dispose()
        if self._connection:
            self._connection.dispose()


def getter_ondemand(observable: Observable[T], timeout: float | None = 30.0) -> T:
    def getter():  # type: ignore[no-untyped-def]
        result = []
        error = []
        event = threading.Event()

        def on_next(value) -> None:  # type: ignore[no-untyped-def]
            result.append(value)
            event.set()

        def on_error(e) -> None:  # type: ignore[no-untyped-def]
            error.append(e)
            event.set()

        def on_completed() -> None:
            event.set()

        # Subscribe and wait for first value
        subscription = observable.pipe(ops.first()).subscribe(
            on_next=on_next, on_error=on_error, on_completed=on_completed
        )

        try:
            if timeout is not None:
                if not event.wait(timeout):
                    raise TimeoutError(f"No value received after {timeout} seconds")
            else:
                event.wait()

            if error:
                raise error[0]

            if not result:
                raise Exception("Observable completed without emitting a value")

            return result[0]
        finally:
            subscription.dispose()

    return getter  # type: ignore[return-value]


def getter_cold(source: Observable[T], timeout: float | None = 30.0) -> T:
    return getter_ondemand(source, timeout)


T = TypeVar("T")  # type: ignore[misc]


def getter_streaming(
    source: Observable[T],
    timeout: float | None = 30.0,
    *,
    nonblocking: bool = False,
) -> LatestReader[T]:
    shared = source.pipe(
        ops.replay(buffer_size=1),
        ops.ref_count(),  # auto-connect & auto-disconnect
    )

    _val_lock = threading.Lock()
    _val: T | None = None
    _ready = threading.Event()

    def _update(v: T) -> None:
        nonlocal _val
        with _val_lock:
            _val = v
        _ready.set()

    sub = shared.subscribe(_update)

    # If we’re in blocking mode, wait right now
    if not nonblocking:
        if timeout is not None and not _ready.wait(timeout):
            sub.dispose()
            raise TimeoutError(f"No value received after {timeout} s")
        else:
            _ready.wait()  # wait indefinitely if timeout is None

    def reader() -> T:
        if not _ready.is_set():  # first call in non-blocking mode
            if timeout is not None and not _ready.wait(timeout):
                raise TimeoutError(f"No value received after {timeout} s")
            else:
                _ready.wait()
        with _val_lock:
            return _val  # type: ignore[return-value]

    def _dispose() -> None:
        sub.dispose()

    reader.dispose = _dispose  # type: ignore[attr-defined]
    return reader  # type: ignore[return-value]


def getter_hot(
    source: Observable[T], timeout: float | None = 30.0, *, nonblocking: bool = False
) -> LatestReader[T]:
    return getter_streaming(source, timeout, nonblocking=nonblocking)


T = TypeVar("T")  # type: ignore[misc]
CB = Callable[[T], Any]


def callback_to_observable(
    start: Callable[[CB[T]], Any],
    stop: Callable[[CB[T]], Any],
) -> Observable[T]:
    """Convert a register/unregister callback API to an Observable.

    Use this for APIs where you register a callback with one function and
    unregister with another, passing the same callback reference:

        sensor.register(callback)    # start
        sensor.unregister(callback)  # stop - needs the same callback

    Example:
        obs = callback_to_observable(
            start=sensor.register,
            stop=sensor.unregister,
        )
        sub = obs.subscribe(lambda x: print(x))
        # ...
        sub.dispose()  # calls sensor.unregister(callback)

    For APIs where subscribe() returns an unsubscribe callable, use
    unsub_to_observable() instead.
    """

    def _subscribe(observer, _scheduler=None):  # type: ignore[no-untyped-def]
        def _on_msg(value: T) -> None:
            observer.on_next(value)

        start(_on_msg)
        return Disposable(lambda: stop(_on_msg))

    return rx.create(_subscribe)


def to_observable(
    subscribe: Callable[[Callable[[T], Any]], Callable[[], None]],
) -> Observable[T]:
    """Convert a subscribe-returns-unsub API to an Observable.

    Use this for APIs where subscribe() returns an unsubscribe callable:

        unsub = pubsub.subscribe(callback)  # returns unsubscribe function
        unsub()  # to unsubscribe

    Example:
        obs = to_observable(pubsub.subscribe)
        sub = obs.subscribe(lambda x: print(x))
        # ...
        sub.dispose()  # calls the unsub function returned by pubsub.subscribe

    For APIs with separate register/unregister functions, use
    callback_to_observable() instead.
    """

    def _subscribe(observer, _scheduler=None):  # type: ignore[no-untyped-def]
        unsub = subscribe(observer.on_next)
        return Disposable(unsub)

    return rx.create(_subscribe)


def spy(name: str):  # type: ignore[no-untyped-def]
    def spyfun(x):  # type: ignore[no-untyped-def]
        print(f"SPY {name}:", x)
        return x

    return ops.map(spyfun)


def quality_barrier(
    quality_func: Callable[[T], float], target_frequency: float
) -> Callable[[Observable[T]], Observable[T]]:
    """
    RxPY pipe operator that selects the highest quality item within each time window.

    Args:
        quality_func: Function to compute quality score for each item
        target_frequency: Output frequency in Hz (e.g., 1.0 for 1 item per second)

    Returns:
        A pipe operator that can be used with .pipe()
    """
    window_duration = 1.0 / target_frequency  # Duration of each window in seconds

    def _quality_barrier(source: Observable[T]) -> Observable[T]:
        return source.pipe(
            # Create non-overlapping time-based windows
            ops.window_with_time(window_duration, window_duration),
            # For each window, find the highest quality item
            ops.flat_map(
                lambda window: window.pipe(  # type: ignore[attr-defined]
                    ops.to_list(),
                    ops.map(lambda items: max(items, key=quality_func) if items else None),  # type: ignore[call-overload]
                    ops.filter(lambda x: x is not None),  # type: ignore[arg-type]
                )
            ),
        )

    return _quality_barrier


def iter_observable(observable: Observable[T]) -> Generator[T, None, None]:
    """Convert an Observable to a blocking iterator.

    Yields items as they arrive from the observable. Properly disposes
    the subscription when the generator is closed.
    """
    q: Queue[T | None] = Queue()
    done = threading.Event()

    def on_next(value: T) -> None:
        q.put(value)

    def on_complete() -> None:
        done.set()
        q.put(None)

    def on_error(e: Exception) -> None:
        done.set()
        q.put(None)

    sub = observable.subscribe(on_next=on_next, on_completed=on_complete, on_error=on_error)

    try:
        while not done.is_set() or not q.empty():
            item = q.get()
            if item is None and done.is_set():
                break
            yield item  # type: ignore[misc]
    finally:
        sub.dispose()
