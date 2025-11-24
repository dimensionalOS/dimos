import threading
from typing import Optional, TypeVar, Generic

import reactivex as rx
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.observable import Observable
from rxpy_backpressure import BackPressure

from dimos.utils.threadpool import get_scheduler

T = TypeVar('T')

# Observable ─► ReplaySubject─► observe_on(pool) ─► backpressure.latest ─► sub1 (fast)
#                           ├──► observe_on(pool) ─► backpressure.latest ─► sub2 (slow)
#                           └──► observe_on(pool) ─► backpressure.latest ─► sub3 (slower)
def backpressure(
    observable: Observable[T],
    scheduler: Optional[ThreadPoolScheduler] = None,
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
    def per_sub():
        # Move processing to thread pool
        base = core.pipe(ops.observe_on(scheduler))

        # optional back-pressure handling
        if not drop_unprocessed:
            return base

        def _subscribe(observer, sch=None):
            return base.subscribe(BackPressure.LATEST(observer), scheduler=sch)

        return rx.create(_subscribe)

    # each `.subscribe()` call gets its own async backpressure chain
    return rx.defer(lambda *_: per_sub())


class LatestReader(Generic[T]):
    """A callable object that returns the latest value from an observable."""

    def __init__(self, initial_value: T, subscription, connection=None):
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

def getter_ondemand(
    observable: Observable[T],
    timeout: Optional[float] = 30.0
) -> T:
    def getter():
        try:
            # Wait for first value with optional timeout
            value = observable.pipe(
                ops.first(),
                *([ops.timeout(timeout)] if timeout is not None else [])
            ).run()
            return value
        except Exception as e:
            raise Exception(f"No value received after {timeout} seconds") from e
    return getter

T = TypeVar("T")


def getter_streaming(
    source: Observable[T],
    timeout: Optional[float] = 30.0,
    *,
    nonblocking: bool = False,
) -> LatestReader[T]:
    shared = source.pipe(
        ops.replay(buffer_size=1),
        ops.ref_count(),            # auto-connect & auto-disconnect
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
            _ready.wait()           # wait indefinitely if timeout is None

    def reader() -> T:
        if not _ready.is_set():                 # first call in non-blocking mode
            if timeout is not None and not _ready.wait(timeout):
                raise TimeoutError(f"No value received after {timeout} s")
            else:
                _ready.wait()
        with _val_lock:
            return _val    # type: ignore[return-value]

    def _dispose() -> None:
        sub.dispose()

    reader.dispose = _dispose          # type: ignore[attr-defined]
    return reader
