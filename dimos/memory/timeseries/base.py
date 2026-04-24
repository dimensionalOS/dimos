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
"""Unified time series storage and replay."""

from __future__ import annotations

from abc import ABC, abstractmethod
import time
from typing import TYPE_CHECKING, Generic, TypeVar

import reactivex as rx
from reactivex import operators as ops

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.observable import Observable

    from dimos.types.timestamped import Timestamped

T = TypeVar("T", bound="Timestamped")


class TimeSeriesStore(Generic[T], ABC):
    """Unified storage + replay for sensor data.

    Implement abstract methods for your backend (in-memory, pickle, sqlite, etc.).
    All iteration, streaming, and seek logic comes free from the base class.

    T must be a Timestamped subclass — timestamps are taken from .ts attribute.
    """

    @abstractmethod
    def _save(self, timestamp: float, data: T) -> None:
        """Save data at timestamp."""
        ...

    @abstractmethod
    def _load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp. Returns None if not found."""
        ...

    @abstractmethod
    def _delete(self, timestamp: float) -> T | None:
        """Delete data at exact timestamp. Returns the deleted item or None."""
        ...

    @abstractmethod
    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        """Lazy iteration of (timestamp, data) in range."""
        ...

    @abstractmethod
    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        """Find closest timestamp. Backend can optimize (binary search, db index, etc.)."""
        ...

    @abstractmethod
    def _count(self) -> int:
        """Return number of stored items."""
        ...

    @abstractmethod
    def _last_timestamp(self) -> float | None:
        """Return the last (largest) timestamp, or None if empty."""
        ...

    @abstractmethod
    def _find_before(self, timestamp: float) -> tuple[float, T] | None:
        """Find the last (ts, data) strictly before the given timestamp."""
        ...

    @abstractmethod
    def _find_after(self, timestamp: float) -> tuple[float, T] | None:
        """Find the first (ts, data) strictly after the given timestamp."""
        ...

    def __len__(self) -> int:
        return self._count()

    def __iter__(self) -> Iterator[T]:
        """Iterate over data items in timestamp order."""
        for _, data in self._iter_items():
            yield data

    def last_timestamp(self) -> float | None:
        """Get the last timestamp in the store."""
        return self._last_timestamp()

    def last(self) -> T | None:
        """Get the last data item in the store."""
        ts = self._last_timestamp()
        if ts is None:
            return None
        return self._load(ts)

    @property
    def start_ts(self) -> float | None:
        """Get the start timestamp of the store."""
        return self.first_timestamp()

    @property
    def end_ts(self) -> float | None:
        """Get the end timestamp of the store."""
        return self._last_timestamp()

    def time_range(self) -> tuple[float, float] | None:
        """Get the time range (start, end) of the store."""
        s = self.first_timestamp()
        e = self._last_timestamp()
        if s is None or e is None:
            return None
        return (s, e)

    def duration(self) -> float:
        """Get the duration of the store in seconds."""
        r = self.time_range()
        return (r[1] - r[0]) if r else 0.0

    def find_before(self, timestamp: float) -> T | None:
        """Find the last item strictly before the given timestamp."""
        result = self._find_before(timestamp)
        return result[1] if result else None

    def find_after(self, timestamp: float) -> T | None:
        """Find the first item strictly after the given timestamp."""
        result = self._find_after(timestamp)
        return result[1] if result else None

    def slice_by_time(self, start: float, end: float) -> list[T]:
        """Return items in [start, end) range."""
        return [data for _, data in self._iter_items(start=start, end=end)]

    def save(self, *data: T) -> None:
        """Save one or more Timestamped items."""
        for item in data:
            self._save(item.ts, item)

    def pipe_save(self, source: Observable[T]) -> Observable[T]:
        """Operator for Observable.pipe() — saves items using .ts.

        Usage:
            observable.pipe(store.pipe_save).subscribe(...)
        """

        def _save_and_return(data: T) -> T:
            self._save(data.ts, data)
            return data

        return source.pipe(ops.map(_save_and_return))

    def consume_stream(self, observable: Observable[T]) -> rx.abc.DisposableBase:
        """Subscribe to an observable and save items using .ts.

        Usage:
            disposable = store.consume_stream(observable)
        """
        return observable.subscribe(on_next=lambda data: self._save(data.ts, data))

    def load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp."""
        return self._load(timestamp)

    def prune_old(self, cutoff: float) -> None:
        """Prune items older than cutoff timestamp."""
        to_delete = [ts for ts, _ in self._iter_items(end=cutoff)]
        for ts in to_delete:
            self._delete(ts)

    def find_closest(
        self,
        timestamp: float,
        tolerance: float | None = None,
    ) -> T | None:
        """Find data closest to the given absolute timestamp."""
        closest_ts = self._find_closest_timestamp(timestamp, tolerance)
        if closest_ts is None:
            return None
        return self._load(closest_ts)

    def find_closest_seek(
        self,
        relative_seconds: float,
        tolerance: float | None = None,
    ) -> T | None:
        """Find data closest to a time relative to the start."""
        first = self.first_timestamp()
        if first is None:
            return None
        return self.find_closest(first + relative_seconds, tolerance)

    def first_timestamp(self) -> float | None:
        """Get the first timestamp in the store."""
        for ts, _ in self._iter_items():
            return ts
        return None

    def first(self) -> T | None:
        """Get the first data item in the store."""
        for _, data in self._iter_items():
            return data
        return None

    def iterate_items(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[tuple[float, T]]:
        """Iterate over (timestamp, data) tuples with optional seek/duration."""
        first = self.first_timestamp()
        if first is None:
            return

        if from_timestamp is not None:
            start = from_timestamp
        elif seek is not None:
            start = first + seek
        else:
            start = None

        end = None
        if duration is not None:
            start_ts = start if start is not None else first
            end = start_ts + duration

        while True:
            yield from self._iter_items(start=start, end=end)
            if not loop:
                break

    def iterate(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[T]:
        """Iterate over data items with optional seek/duration."""
        for _, data in self.iterate_items(
            seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
        ):
            yield data

    def iterate_realtime(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[T]:
        """Iterate data, sleeping to match original timing."""
        prev_ts: float | None = None
        for ts, data in self.iterate_items(
            seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
        ):
            if prev_ts is not None:
                delay = (ts - prev_ts) / speed
                if delay > 0:
                    time.sleep(delay)
            prev_ts = ts
            yield data

    def stream(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Observable[T]:
        """Stream data as Observable with timing control.

        Uses scheduler-based timing with absolute time reference to prevent drift.
        """
        from dimos.utils.testing.replay import timed_playback

        return timed_playback(
            lambda: self.iterate_items(
                seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
            ),
            speed=speed,
            detect_loop=loop,
        )
