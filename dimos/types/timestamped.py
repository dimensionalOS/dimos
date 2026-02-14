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
from collections import defaultdict
from datetime import datetime, timezone
from typing import Generic, TypeVar, Union

from dimos_lcm.builtin_interfaces import Time as ROSTime
from reactivex import create
from reactivex.disposable import CompositeDisposable

# from dimos_lcm.std_msgs import Time as ROSTime
from reactivex.observable import Observable

from dimos.memory.timeseries.inmemory import InMemoryStore
from dimos.types.weaklist import WeakList
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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
    if isinstance(ts, int | float):
        return float(ts)
    if isinstance(ts, dict) and "sec" in ts and "nanosec" in ts:
        return ts["sec"] + ts["nanosec"] / 1e9  # type: ignore[no-any-return]
    # Check for ROS Time-like objects by attributes
    if hasattr(ts, "sec") and (hasattr(ts, "nanosec") or hasattr(ts, "nsec")):
        # Handle both std_msgs.Time (nsec) and builtin_interfaces.Time (nanosec)
        if hasattr(ts, "nanosec"):
            return ts.sec + ts.nanosec / 1e9  # type: ignore[no-any-return]
        else:  # has nsec
            return ts.sec + ts.nsec / 1e9  # type: ignore[no-any-return]
    raise TypeError("unsupported timestamp type")


def to_ros_stamp(ts: TimeLike) -> ROSTime:
    """Convert TimeLike to a ROS-style timestamp dictionary."""
    if isinstance(ts, dict) and "sec" in ts and "nanosec" in ts:
        return ts

    timestamp = to_timestamp(ts)
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1_000_000_000)
    return ROSTime(sec=sec, nanosec=nanosec)


def to_human_readable(ts: float) -> str:
    """Convert timestamp to human-readable format with date and time."""
    import time

    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts))


def to_datetime(ts: TimeLike, tz=None) -> datetime:  # type: ignore[no-untyped-def]
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

    def __init__(self, ts: float) -> None:
        self.ts = ts

    def dt(self) -> datetime:
        return datetime.fromtimestamp(self.ts, tz=timezone.utc).astimezone()

    def ros_timestamp(self) -> list[int]:
        """Convert timestamp to ROS-style list [sec, nanosec]."""
        sec = int(self.ts)
        nanosec = int((self.ts - sec) * 1_000_000_000)
        return [sec, nanosec]


T = TypeVar("T", bound=Timestamped)


PRIMARY = TypeVar("PRIMARY", bound=Timestamped)
SECONDARY = TypeVar("SECONDARY", bound=Timestamped)


class TimestampedBufferCollection(InMemoryStore[T]):
    """A sliding time window buffer backed by InMemoryStore."""

    def __init__(self, window_duration: float) -> None:
        super().__init__()
        self.window_duration = window_duration

    def add(self, item: T) -> None:
        """Add a timestamped item and prune items outside the time window."""
        self.save(item)
        self.prune_old(item.ts - self.window_duration)

    def remove(self, item: T) -> bool:
        """Remove a timestamped item. Returns True if found and removed."""
        return self._delete(item.ts) is not None

    def remove_by_timestamp(self, timestamp: float) -> bool:
        """Remove an item by timestamp. Returns True if found and removed."""
        return self._delete(timestamp) is not None


class MatchContainer(Timestamped, Generic[PRIMARY, SECONDARY]):
    """
    This class stores a primary item along with its partial matches to secondary items,
    tracking which secondaries are still missing to avoid redundant searches.
    """

    def __init__(self, primary: PRIMARY, matches: list[SECONDARY | None]) -> None:
        super().__init__(primary.ts)
        self.primary = primary
        self.matches = matches  # Direct list with None for missing matches

    def message_received(self, secondary_idx: int, secondary_item: SECONDARY) -> None:
        """Process a secondary message and check if it matches this primary."""
        if self.matches[secondary_idx] is None:
            self.matches[secondary_idx] = secondary_item

    def is_complete(self) -> bool:
        """Check if all secondary matches have been found."""
        return all(match is not None for match in self.matches)

    def get_tuple(self) -> tuple[PRIMARY, ...]:
        """Get the result tuple for emission."""
        return (self.primary, *self.matches)  # type: ignore[arg-type]


def align_timestamped(
    primary_observable: Observable[PRIMARY],
    *secondary_observables: Observable[SECONDARY],
    buffer_size: float = 1.0,  # seconds
    match_tolerance: float = 0.1,  # seconds
) -> Observable[tuple[PRIMARY, ...]]:
    """Align a primary observable with one or more secondary observables.

    Args:
        primary_observable: The primary stream to align against
        *secondary_observables: One or more secondary streams to align
        buffer_size: Time window to keep messages in seconds
        match_tolerance: Maximum time difference for matching in seconds

    Returns:
        If single secondary observable: Observable that emits tuples of (primary_item, secondary_item)
        If multiple secondary observables: Observable that emits tuples of (primary_item, secondary1, secondary2, ...)
        Each secondary item is the closest match from the corresponding
        secondary observable, or None if no match within tolerance.
    """

    def subscribe(observer, scheduler=None):  # type: ignore[no-untyped-def]
        # Create a timed buffer collection for each secondary observable
        secondary_collections: list[TimestampedBufferCollection[SECONDARY]] = [
            TimestampedBufferCollection(buffer_size) for _ in secondary_observables
        ]

        # WeakLists to track subscribers to each secondary observable
        secondary_stakeholders = defaultdict(WeakList)  # type: ignore[var-annotated]

        # Buffer for unmatched MatchContainers - automatically expires old items
        primary_buffer: TimestampedBufferCollection[MatchContainer[PRIMARY, SECONDARY]] = (
            TimestampedBufferCollection(buffer_size)
        )

        # Subscribe to all secondary observables
        secondary_subs = []

        def has_secondary_progressed_past(secondary_ts: float, primary_ts: float) -> bool:
            """Check if secondary stream has progressed past the primary + tolerance."""
            return secondary_ts > primary_ts + match_tolerance

        def remove_stakeholder(stakeholder: MatchContainer) -> None:  # type: ignore[type-arg]
            """Remove a stakeholder from all tracking structures."""
            primary_buffer.remove(stakeholder)
            for weak_list in secondary_stakeholders.values():
                weak_list.discard(stakeholder)

        def on_secondary(i: int, secondary_item: SECONDARY) -> None:
            # Add the secondary item to its collection
            secondary_collections[i].add(secondary_item)

            # Check all stakeholders for this secondary stream
            for stakeholder in secondary_stakeholders[i]:
                # If the secondary stream has progressed past this primary,
                # we won't be able to match it anymore
                if has_secondary_progressed_past(secondary_item.ts, stakeholder.ts):
                    logger.debug(f"secondary progressed, giving up {stakeholder.ts}")

                    remove_stakeholder(stakeholder)
                    continue

                # Check if this secondary is within tolerance of the primary
                if abs(stakeholder.ts - secondary_item.ts) <= match_tolerance:
                    stakeholder.message_received(i, secondary_item)

                    # If all secondaries matched, emit result
                    if stakeholder.is_complete():
                        logger.debug(f"Emitting deferred match {stakeholder.ts}")
                        observer.on_next(stakeholder.get_tuple())
                        remove_stakeholder(stakeholder)

        for i, secondary_obs in enumerate(secondary_observables):
            secondary_subs.append(
                secondary_obs.subscribe(
                    lambda x, idx=i: on_secondary(idx, x),  # type: ignore[misc]
                    on_error=observer.on_error,
                )
            )

        def on_primary(primary_item: PRIMARY) -> None:
            # Try to find matches in existing secondary collections
            matches = [None] * len(secondary_observables)

            for i, collection in enumerate(secondary_collections):
                closest = collection.find_closest(primary_item.ts, tolerance=match_tolerance)
                if closest is not None:
                    matches[i] = closest  # type: ignore[call-overload]
                else:
                    # Check if this secondary stream has already progressed past this primary
                    if collection.end_ts is not None and has_secondary_progressed_past(
                        collection.end_ts, primary_item.ts
                    ):
                        # This secondary won't match, so don't buffer this primary
                        return

            # If all matched, emit immediately without creating MatchContainer
            if all(match is not None for match in matches):
                logger.debug(f"Immadiate match {primary_item.ts}")
                result = (primary_item, *matches)
                observer.on_next(result)
            else:
                logger.debug(f"Deferred match attempt {primary_item.ts}")
                match_container = MatchContainer(primary_item, matches)  # type: ignore[type-var]
                primary_buffer.add(match_container)  # type: ignore[arg-type]

                for i, match in enumerate(matches):
                    if match is None:
                        secondary_stakeholders[i].append(match_container)

        # Subscribe to primary observable
        primary_sub = primary_observable.subscribe(
            on_primary, on_error=observer.on_error, on_completed=observer.on_completed
        )

        # Return a CompositeDisposable for proper cleanup
        return CompositeDisposable(primary_sub, *secondary_subs)

    return create(subscribe)
