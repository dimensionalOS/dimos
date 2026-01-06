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
import threading
from typing import Any, Generic, TypeVar

import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.rxpy_backpressure import BackPressure  # type: ignore[import-untyped]
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


class LatestReader(Generic[T]):
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


T = TypeVar("T")  # type: ignore[misc]
CB = Callable[[T], Any]


def callback_to_observable(
    start: Callable[[CB[T]], Any],
    stop: Callable[[CB[T]], Any],
) -> Observable[T]:
    def _subscribe(observer, _scheduler=None):  # type: ignore[no-untyped-def]
        def _on_msg(value: T) -> None:
            observer.on_next(value)

        start(_on_msg)
        return Disposable(lambda: stop(_on_msg))

    return rx.create(_subscribe)


def spy(name: str):  # type: ignore[no-untyped-def]
    def spyfun(x):  # type: ignore[no-untyped-def]
        print(f"SPY {name}:", x)
        return x

    return ops.map(spyfun)


def quality_barrier(quality_func: Callable[[T], float], target_frequency: float):  # type: ignore[no-untyped-def]
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
