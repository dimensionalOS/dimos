from __future__ import annotations
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from itertools import tee
from typing import Iterable, Protocol, TypeVar, Generic, Tuple, Union, overload
from abc import ABC, abstractmethod


PAYLOAD = TypeVar("PAYLOAD")

EpochLike = Union[int, float, datetime]


def to_ns(ts: EpochLike) -> int:
    """Convert datetime/int/float → int nanoseconds since epoch (UTC)."""
    if isinstance(ts, datetime):
        if ts.tzinfo is None:
            # assume we are in the current tz
            ts = ts.astimezone()
        return int(ts.timestamp() * 1000)
    if isinstance(ts, float):
        return int(ts * 1000)  # float seconds → ns
    if isinstance(ts, int):
        return ts
    raise TypeError("unsupported timestamp type")


@dataclass(frozen=True, slots=True)
class TEvent(Generic[PAYLOAD]):
    _ts: int = field(init=False, repr=False)  # internal storage
    data: PAYLOAD

    def __init__(self, timestamp: EpochLike, data: PAYLOAD):
        object.__setattr__(self, "_ts", to_ns(timestamp))
        object.__setattr__(self, "data", data)

    @property
    def ts(self) -> int:
        return self._ts

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TEvent):
            return NotImplemented
        return self.ts == other.ts and self.data == other.data


class HasTimestamp(Protocol):  # structural constraint for tooling
    ts: int


EVENT = TypeVar("EVENT", bound=HasTimestamp)  # any object that has .ts: int


class Timeseries(ABC, Generic[EVENT]):
    """Abstract class for an iterable of events with timestamps."""

    @abstractmethod
    def __iter__(self) -> Iterable[EVENT]: ...

    @property
    def start_time(self) -> datetime:
        """Return the timestamp of the earliest event, assuming the data is sorted."""
        return next(iter(self)).ts

    @property
    def end_time(self) -> datetime:
        """Return the timestamp of the latest event, assuming the data is sorted."""
        return next(reversed(list(self))).ts

    def time_range(self) -> Tuple[datetime, datetime]:
        """Return (earliest_ts, latest_ts).  Empty input ⇒ ValueError."""
        return self.start_time, self.end_time

    def duration(self) -> timedelta:
        """Total time spanned by the iterable (Δ = last - first)."""
        return timedelta(milliseconds=self.end_time - self.start_time)

    def closest_to(self, timestamp: EpochLike) -> EVENT:
        """Return the event closest to the given timestamp."""
        target_ts = to_ns(timestamp)
        return min(self, key=lambda e: abs(e.ts - target_ts))

    @property
    def frequency(self) -> float:
        """Calculate the frequency of events in Hz."""
        return len(list(self)) / (self.duration().total_seconds() or 1)

    def __repr__(self) -> str:
        """Return a string representation of the Timeseries."""
        start_dt = tz_datetime(self.start_time)
        end_dt = tz_datetime(self.end_time)
        return f"Timeseries(date={start_dt.strftime('%Y-%m-%d')}, start={start_dt.strftime('%H:%M:%S')}, end={end_dt.strftime('%H:%M:%S')}, duration={self.duration()}, events={len(list(self))}, freq={self.frequency:.2f}Hz)"

    def __str__(self) -> str:
        """Return a string representation of the Timeseries."""
        return self.__repr__()


def castdt(event: EVENT) -> datetime:
    return tstodt(event.ts)


def tstodt(ts: int) -> datetime:
    return datetime.fromtimestamp(ts / 1e9, tz=timezone.utc)


def tz_datetime(ts: int, tz=None) -> datetime:
    """Convert a timestamp to a timezone-aware datetime."""
    return datetime.fromtimestamp(ts / 1000)


def human_readable(ts: int) -> str:
    """Convert a timestamp to a human-readable format."""
    return tz_datetime(ts).strftime("%Y-%m-%d %H:%M:%S")
