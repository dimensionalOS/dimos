"""Validated monotonic clock dependency for execution deadlines."""

from collections.abc import Callable
import math
import threading


class InvalidMonotonicClock(ValueError):  # noqa: N818
    """Raised when a clock violates the execution clock contract."""


class ValidatedMonotonicClock:
    """Serialize sampling and validate finite, nondecreasing seconds."""

    def __init__(self, source: Callable[[], float]) -> None:
        self._source = source
        self._lock = threading.Lock()
        with self._lock:
            initial = self._source()
            if not math.isfinite(initial):
                raise InvalidMonotonicClock("monotonic clock returned a non-finite value")
            self._last = initial

    def now(self) -> float:
        with self._lock:
            sample = self._source()
            if not math.isfinite(sample):
                raise InvalidMonotonicClock("monotonic clock returned a non-finite value")
            if sample < self._last:
                raise InvalidMonotonicClock("monotonic clock moved backwards")
            self._last = sample
            return sample
