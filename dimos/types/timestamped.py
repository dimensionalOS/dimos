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

import bisect
from datetime import datetime, timezone
from typing import Generic, Iterable, Optional, Tuple, TypedDict, TypeVar, Union

from sortedcontainers import SortedList

# any class that carries a timestamp should inherit from this
# this allows us to work with timeseries in consistent way, allign messages, replay etc
# aditional functionality will come to this class soon


class RosStamp(TypedDict):
    sec: int
    nanosec: int


TimeLike = Union[int, float, datetime, RosStamp]


def to_timestamp(ts: TimeLike) -> float:
    """Convert TimeLike to a timestamp in seconds."""
    if isinstance(ts, datetime):
        return ts.timestamp()
    if isinstance(ts, (int, float)):
        return float(ts)
    if isinstance(ts, dict) and "sec" in ts and "nanosec" in ts:
        return ts["sec"] + ts["nanosec"] / 1e9
    raise TypeError("unsupported timestamp type")


def to_ros_stamp(ts: TimeLike) -> RosStamp:
    """Convert TimeLike to a ROS-style timestamp dictionary."""
    if isinstance(ts, dict) and "sec" in ts and "nanosec" in ts:
        return ts

    timestamp = to_timestamp(ts)
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1_000_000_000)
    return {"sec": sec, "nanosec": nanosec}


def to_datetime(ts: TimeLike, tz=None) -> datetime:
    if isinstance(ts, datetime):
        if ts.tzinfo is None:
            # Assume UTC for naive datetime
            ts = ts.replace(tzinfo=timezone.utc)
        if tz is not None:
            return ts.astimezone(tz)
        return ts.astimezone()  # Convert to local tz

    # Convert to timestamp first
    timestamp = to_timestamp(ts)

    # Create datetime from timestamp
    if tz is not None:
        return datetime.fromtimestamp(timestamp, tz=tz)
    else:
        # Use local timezone by default
        return datetime.fromtimestamp(timestamp).astimezone()


class Timestamped:
    ts: float

    def __init__(self, ts: float):
        self.ts = ts

    def dt(self) -> datetime:
        return datetime.fromtimestamp(self.ts, tz=timezone.utc).astimezone()

    def ros_timestamp(self) -> dict[str, int]:
        """Convert timestamp to ROS-style dictionary."""
        sec = int(self.ts)
        nanosec = int((self.ts - sec) * 1_000_000_000)
        return [sec, nanosec]


T = TypeVar("T", bound=Timestamped)


class TimestampedCollection(Generic[T]):
    """A collection of timestamped objects with efficient time-based operations."""

    def __init__(self, items: Optional[Iterable[T]] = None):
        self._items = SortedList(items or [], key=lambda x: x.ts)

    def add(self, item: T) -> None:
        """Add a timestamped item to the collection."""
        self._items.add(item)

    def find_closest(self, timestamp: float) -> Optional[T]:
        """Find the timestamped object closest to the given timestamp."""
        if not self._items:
            return None

        # Find insertion point using binary search on timestamps
        timestamps = [item.ts for item in self._items]
        idx = bisect.bisect_left(timestamps, timestamp)

        # Check boundaries
        if idx == 0:
            return self._items[0]
        if idx == len(self._items):
            return self._items[-1]

        # Compare distances to neighbors
        left_diff = abs(timestamp - self._items[idx - 1].ts)
        right_diff = abs(self._items[idx].ts - timestamp)

        return self._items[idx - 1] if left_diff < right_diff else self._items[idx]

    def find_before(self, timestamp: float) -> Optional[T]:
        """Find the last item before the given timestamp."""
        timestamps = [item.ts for item in self._items]
        idx = bisect.bisect_left(timestamps, timestamp)
        return self._items[idx - 1] if idx > 0 else None

    def find_after(self, timestamp: float) -> Optional[T]:
        """Find the first item after the given timestamp."""
        timestamps = [item.ts for item in self._items]
        idx = bisect.bisect_right(timestamps, timestamp)
        return self._items[idx] if idx < len(self._items) else None

    def merge(self, other: "TimestampedCollection[T]") -> "TimestampedCollection[T]":
        """Merge two timestamped collections into a new one."""
        result = TimestampedCollection[T]()
        result._items = SortedList(self._items + other._items, key=lambda x: x.ts)
        return result

    def duration(self) -> float:
        """Get the duration of the collection in seconds."""
        if len(self._items) < 2:
            return 0.0
        return self._items[-1].ts - self._items[0].ts

    def time_range(self) -> Optional[Tuple[float, float]]:
        """Get the time range (start, end) of the collection."""
        if not self._items:
            return None
        return (self._items[0].ts, self._items[-1].ts)

    def slice_by_time(self, start: float, end: float) -> "TimestampedCollection[T]":
        """Get a subset of items within the given time range."""
        timestamps = [item.ts for item in self._items]
        start_idx = bisect.bisect_left(timestamps, start)
        end_idx = bisect.bisect_right(timestamps, end)
        return TimestampedCollection(self._items[start_idx:end_idx])

    @property
    def start_ts(self) -> Optional[float]:
        """Get the start timestamp of the collection."""
        return self._items[0].ts if self._items else None

    @property
    def end_ts(self) -> Optional[float]:
        """Get the end timestamp of the collection."""
        return self._items[-1].ts if self._items else None

    def __len__(self) -> int:
        return len(self._items)

    def __iter__(self):
        return iter(self._items)

    def __getitem__(self, idx: int) -> T:
        return self._items[idx]
