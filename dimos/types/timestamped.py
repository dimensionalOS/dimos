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

from datetime import datetime, timezone
from typing import Generic, Iterable, Optional, Tuple, TypedDict, TypeVar, Union

from dimos_lcm.builtin_interfaces import Time as ROSTime

# from dimos_lcm.std_msgs import Time as ROSTime
from reactivex.observable import Observable
from sortedcontainers import SortedKeyList

# any class that carries a timestamp should inherit from this
# this allows us to work with timeseries in consistent way, allign messages, replay etc
# aditional functionality will come to this class soon


# class RosStamp(TypedDict):
#     sec: int
#     nanosec: int


TimeLike = Union[int, float, datetime, ROSTime]


def to_timestamp(ts: TimeLike) -> float:
    """Convert TimeLike to a timestamp in seconds."""
    if isinstance(ts, datetime):
        return ts.timestamp()
    if isinstance(ts, (int, float)):
        return float(ts)
    if isinstance(ts, dict) and "sec" in ts and "nanosec" in ts:
        return ts["sec"] + ts["nanosec"] / 1e9
    raise TypeError("unsupported timestamp type")


def to_ros_stamp(ts: TimeLike) -> ROSTime:
    """Convert TimeLike to a ROS-style timestamp dictionary."""
    if isinstance(ts, dict) and "sec" in ts and "nanosec" in ts:
        return ts

    timestamp = to_timestamp(ts)
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1_000_000_000)
    return ROSTime(sec=sec, nanosec=nanosec)


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
        self._items = SortedKeyList(items or [], key=lambda x: x.ts)

    def add(self, item: T) -> None:
        """Add a timestamped item to the collection."""
        self._items.add(item)

    def find_closest(self, timestamp: float, tolerance: Optional[float] = None) -> Optional[T]:
        """Find the timestamped object closest to the given timestamp."""
        if not self._items:
            return None

        # Use binary search to find insertion point
        idx = self._items.bisect_key_left(timestamp)

        # Check exact match
        if idx < len(self._items) and self._items[idx].ts == timestamp:
            return self._items[idx]

        # Find candidates: item before and after
        candidates = []

        # Item before
        if idx > 0:
            candidates.append((idx - 1, abs(self._items[idx - 1].ts - timestamp)))

        # Item after
        if idx < len(self._items):
            candidates.append((idx, abs(self._items[idx].ts - timestamp)))

        if not candidates:
            return None

        # Find closest
        # When distances are equal, prefer the later item (higher index)
        closest_idx, closest_distance = min(candidates, key=lambda x: (x[1], -x[0]))

        # Check tolerance if provided
        if tolerance is not None and closest_distance > tolerance:
            return None

        return self._items[closest_idx]

    def find_before(self, timestamp: float) -> Optional[T]:
        """Find the last item before the given timestamp."""
        idx = self._items.bisect_key_left(timestamp)
        return self._items[idx - 1] if idx > 0 else None

    def find_after(self, timestamp: float) -> Optional[T]:
        """Find the first item after the given timestamp."""
        idx = self._items.bisect_key_right(timestamp)
        return self._items[idx] if idx < len(self._items) else None

    def merge(self, other: "TimestampedCollection[T]") -> "TimestampedCollection[T]":
        """Merge two timestamped collections into a new one."""
        result = TimestampedCollection[T]()
        result._items = SortedKeyList(self._items + other._items, key=lambda x: x.ts)
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
        start_idx = self._items.bisect_key_left(start)
        end_idx = self._items.bisect_key_right(end)
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


PRIMARY = TypeVar("PRIMARY", bound=Timestamped)
SECONDARY = TypeVar("SECONDARY", bound=Timestamped)


class TimestampedBufferCollection(TimestampedCollection[T]):
    """A timestamped collection that maintains a sliding time window, dropping old messages."""

    def __init__(self, window_duration: float, items: Optional[Iterable[T]] = None):
        """
        Initialize with a time window duration in seconds.

        Args:
            window_duration: Maximum age of messages to keep in seconds
            items: Optional initial items
        """
        super().__init__(items)
        self.window_duration = window_duration

    def add(self, item: T) -> None:
        """Add a timestamped item and remove any items outside the time window."""
        super().add(item)
        self._prune_old_messages(item.ts)

    def _prune_old_messages(self, current_ts: float) -> None:
        """Remove messages older than window_duration from the given timestamp."""
        cutoff_ts = current_ts - self.window_duration

        # Find the index of the first item that should be kept
        keep_idx = self._items.bisect_key_left(cutoff_ts)

        # Remove old items
        if keep_idx > 0:
            del self._items[:keep_idx]


def align_timestamped(
    primary_observable: Observable[PRIMARY],
    secondary_observable: Observable[SECONDARY],
    buffer_size: float = 1.0,  # seconds
    match_tolerance: float = 0.05,  # seconds
) -> Observable[Tuple[PRIMARY, SECONDARY]]:
    from reactivex import create

    def subscribe(observer, scheduler=None):
        secondary_collection: TimestampedBufferCollection[SECONDARY] = TimestampedBufferCollection(
            buffer_size
        )
        # Subscribe to secondary to populate the buffer
        secondary_sub = secondary_observable.subscribe(secondary_collection.add)

        def on_primary(primary_item: PRIMARY):
            secondary_item = secondary_collection.find_closest(
                primary_item.ts, tolerance=match_tolerance
            )
            if secondary_item is not None:
                observer.on_next((primary_item, secondary_item))

        # Subscribe to primary and emit aligned pairs
        primary_sub = primary_observable.subscribe(
            on_next=on_primary, on_error=observer.on_error, on_completed=observer.on_completed
        )

        # Return cleanup function
        def dispose():
            secondary_sub.dispose()
            primary_sub.dispose()

        return dispose

    return create(subscribe)
