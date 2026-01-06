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
from functools import wraps
import threading
import time
from typing import Any, Protocol, TypeVar

from .accumulators import Accumulator, LatestAccumulator

_CacheResult_co = TypeVar("_CacheResult_co", covariant=True)
_CacheReturn = TypeVar("_CacheReturn")


class CachedMethod(Protocol[_CacheResult_co]):
    """Protocol for methods decorated with simple_mcache."""

    def __call__(self) -> _CacheResult_co: ...
    def invalidate_cache(self, instance: Any) -> None: ...


def limit(max_freq: float, accumulator: Accumulator | None = None):  # type: ignore[no-untyped-def, type-arg]
    """
    Decorator that limits function call frequency.

    If calls come faster than max_freq, they are skipped.
    If calls come slower than max_freq, they pass through immediately.

    Args:
        max_freq: Maximum frequency in Hz (calls per second)
        accumulator: Optional accumulator to collect skipped calls (defaults to LatestAccumulator)

    Returns:
        Decorated function that respects the frequency limit
    """
    if max_freq <= 0:
        raise ValueError("Frequency must be positive")

    min_interval = 1.0 / max_freq

    # Create default accumulator if none provided
    if accumulator is None:
        accumulator = LatestAccumulator()

    def decorator(func: Callable) -> Callable:  # type: ignore[type-arg]
        last_call_time = 0.0
        lock = threading.Lock()
        timer: threading.Timer | None = None

        def execute_accumulated() -> None:
            nonlocal last_call_time, timer
            with lock:
                if len(accumulator):
                    acc_args, acc_kwargs = accumulator.get()  # type: ignore[misc]
                    last_call_time = time.time()
                    timer = None
                    func(*acc_args, **acc_kwargs)

        @wraps(func)
        def wrapper(*args, **kwargs):  # type: ignore[no-untyped-def]
            nonlocal last_call_time, timer
            current_time = time.time()

            with lock:
                time_since_last = current_time - last_call_time

                if time_since_last >= min_interval:
                    # Cancel any pending timer
                    if timer is not None:
                        timer.cancel()
                        timer = None

                    # Enough time has passed, execute the function
                    last_call_time = current_time

                    # if we have accumulated data, we get a compound value
                    if len(accumulator):
                        accumulator.add(*args, **kwargs)
                        acc_args, acc_kwargs = accumulator.get()  # type: ignore[misc]  # accumulator resets here
                        return func(*acc_args, **acc_kwargs)

                    # No accumulated data, normal call
                    return func(*args, **kwargs)

                else:
                    # Too soon, skip this call
                    accumulator.add(*args, **kwargs)

                    # Schedule execution for when the interval expires
                    if timer is not None:
                        timer.cancel()

                    time_to_wait = min_interval - time_since_last
                    timer = threading.Timer(time_to_wait, execute_accumulated)
                    timer.start()

                    return None

        return wrapper

    return decorator


def simple_mcache(method: Callable) -> Callable:  # type: ignore[type-arg]
    """
    Decorator to cache the result of a method call on the instance.

    The cached value is stored as an attribute on the instance with the name
    `_cached_<method_name>`. Subsequent calls to the method will return the
    cached value instead of recomputing it.

    Thread-safe: Uses a lock per instance to ensure the cached value is
    computed only once even in multi-threaded environments.

    Args:
        method: The method to be decorated.

    Returns:
        The decorated method with caching behavior.
    """

    attr_name = f"_cached_{method.__name__}"
    lock_name = f"_lock_{method.__name__}"

    @wraps(method)
    def getter(self):  # type: ignore[no-untyped-def]
        # Get or create the lock for this instance
        if not hasattr(self, lock_name):
            setattr(self, lock_name, threading.Lock())

        lock = getattr(self, lock_name)

        if hasattr(self, attr_name):
            return getattr(self, attr_name)

        with lock:
            # Check again inside the lock
            if not hasattr(self, attr_name):
                setattr(self, attr_name, method(self))
            return getattr(self, attr_name)

    def invalidate_cache(instance: Any) -> None:
        """Clear the cached value for the given instance."""
        if not hasattr(instance, lock_name):
            return

        lock = getattr(instance, lock_name)
        with lock:
            if hasattr(instance, attr_name):
                delattr(instance, attr_name)

    getter.invalidate_cache = invalidate_cache  # type: ignore[attr-defined]

    return getter


def retry(max_retries: int = 3, on_exception: type[Exception] = Exception, delay: float = 0.0):  # type: ignore[no-untyped-def]
    """
    Decorator that retries a function call if it raises an exception.

    Args:
        max_retries: Maximum number of retry attempts (default: 3)
        on_exception: Exception type to catch and retry on (default: Exception)
        delay: Fixed delay in seconds between retries (default: 0.0)

    Returns:
        Decorated function that will retry on failure

    Example:
        @retry(max_retries=5, on_exception=ConnectionError, delay=0.5)
        def connect_to_server():
            # connection logic that might fail
            pass

        @retry()  # Use defaults: 3 retries on any Exception, no delay
        def risky_operation():
            # might fail occasionally
            pass
    """
    if max_retries < 0:
        raise ValueError("max_retries must be non-negative")
    if delay < 0:
        raise ValueError("delay must be non-negative")

    def decorator(func: Callable) -> Callable:  # type: ignore[type-arg]
        @wraps(func)
        def wrapper(*args, **kwargs):  # type: ignore[no-untyped-def]
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except on_exception as e:
                    last_exception = e
                    if attempt < max_retries:
                        # Still have retries left
                        if delay > 0:
                            time.sleep(delay)
                        continue
                    else:
                        # Out of retries, re-raise the last exception
                        raise

            # This should never be reached, but just in case
            if last_exception:
                raise last_exception

        return wrapper

    return decorator
