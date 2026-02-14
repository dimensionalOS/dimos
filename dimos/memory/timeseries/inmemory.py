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
"""In-memory backend for TimeSeriesStore."""

from collections.abc import Iterator

from sortedcontainers import SortedKeyList  # type: ignore[import-untyped]

from dimos.memory.timeseries.base import T, TimeSeriesStore


class InMemoryStore(TimeSeriesStore[T]):
    """In-memory storage using SortedKeyList. O(log n) insert, lookup, and range queries."""

    def __init__(self) -> None:
        self._entries: SortedKeyList = SortedKeyList(key=lambda e: e.ts)

    def _bisect_exact(self, timestamp: float) -> int | None:
        """Return index of entry with exact timestamp, or None."""
        pos = self._entries.bisect_key_left(timestamp)
        if pos < len(self._entries) and self._entries[pos].ts == timestamp:
            return pos  # type: ignore[no-any-return]
        return None

    def _save(self, timestamp: float, data: T) -> None:
        self._entries.add(data)

    def _load(self, timestamp: float) -> T | None:
        idx = self._bisect_exact(timestamp)
        if idx is not None:
            return self._entries[idx]  # type: ignore[no-any-return]
        return None

    def _delete(self, timestamp: float) -> T | None:
        idx = self._bisect_exact(timestamp)
        if idx is not None:
            data = self._entries[idx]
            del self._entries[idx]
            return data  # type: ignore[no-any-return]
        return None

    def __iter__(self) -> Iterator[T]:
        yield from self._entries

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        if start is not None and end is not None:
            it = self._entries.irange_key(start, end, (True, False))
        elif start is not None:
            it = self._entries.irange_key(min_key=start)
        elif end is not None:
            it = self._entries.irange_key(max_key=end, inclusive=(True, False))
        else:
            it = iter(self._entries)
        for e in it:
            yield (e.ts, e)

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        if not self._entries:
            return None

        pos = self._entries.bisect_key_left(timestamp)

        candidates: list[float] = []
        if pos > 0:
            candidates.append(self._entries[pos - 1].ts)
        if pos < len(self._entries):
            candidates.append(self._entries[pos].ts)

        if not candidates:
            return None

        # On ties, prefer the later timestamp (more recent data)
        closest = max(candidates, key=lambda ts: (-abs(ts - timestamp), ts))

        if tolerance is not None and abs(closest - timestamp) > tolerance:
            return None

        return closest

    def _count(self) -> int:
        return len(self._entries)

    def _last_timestamp(self) -> float | None:
        if not self._entries:
            return None
        return self._entries[-1].ts  # type: ignore[no-any-return]

    def _find_before(self, timestamp: float) -> tuple[float, T] | None:
        if not self._entries:
            return None
        pos = self._entries.bisect_key_left(timestamp)
        if pos > 0:
            e = self._entries[pos - 1]
            return (e.ts, e)
        return None

    def _find_after(self, timestamp: float) -> tuple[float, T] | None:
        if not self._entries:
            return None
        pos = self._entries.bisect_key_right(timestamp)
        if pos < len(self._entries):
            e = self._entries[pos]
            return (e.ts, e)
        return None
