from __future__ import annotations
from datetime import datetime, timedelta, timezone
from typing import Iterable, TypeVar, Generic, Tuple, Union, TypedDict
from abc import ABC, abstractmethod


PAYLOAD = TypeVar("PAYLOAD")


class RosStamp(TypedDict):
    sec: int
    nanosec: int


EpochLike = Union[int, float, datetime, RosStamp]


def from_ros_stamp(stamp: dict[str, int], tz: timezone = None) -> datetime:
    """Convert ROS-style timestamp {'sec': int, 'nanosec': int} to datetime."""
    return datetime.fromtimestamp(stamp["sec"] + stamp["nanosec"] / 1e9, tz=tz)


def to_human_readable(ts: EpochLike) -> str:
    dt = to_datetime(ts)
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def to_datetime(ts: EpochLike, tz: timezone = None) -> datetime:
    if isinstance(ts, datetime):
        # if ts.tzinfo is None:
        #    ts = ts.astimezone(tz)
        return ts
    if isinstance(ts, (int, float)):
        return datetime.fromtimestamp(ts, tz=tz)
    if isinstance(ts, dict) and "sec" in ts and "nanosec" in ts:
        return datetime.fromtimestamp(ts["sec"] + ts["nanosec"] / 1e9, tz=tz)
    raise TypeError("unsupported timestamp type")


class Timestamped(ABC):
    """Abstract class for an event with a timestamp."""

    def __init__(self, timestamp: EpochLike):
        self.ts = to_datetime(timestamp)


class TEvent(Timestamped, Generic[PAYLOAD]):
    """Concrete class for an event with a timestamp and data."""

    def __init__(self, timestamp: EpochLike, data: PAYLOAD):
        super().__init__(timestamp)
        self.data = data

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TEvent):
            return NotImplemented
        return self.ts == other.ts and self.data == other.data

    def __repr__(self) -> str:
        return f"TEvent(ts={self.ts}, data={self.data})"


EVENT = TypeVar("EVENT", bound=Timestamped)  # any object that is a subclass of Timestamped


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

    @property
    def frequency(self) -> float:
        """Calculate the frequency of events in Hz."""
        return len(list(self)) / (self.duration().total_seconds() or 1)

    def time_range(self) -> Tuple[datetime, datetime]:
        """Return (earliest_ts, latest_ts).  Empty input ⇒ ValueError."""
        return self.start_time, self.end_time

    def duration(self) -> timedelta:
        """Total time spanned by the iterable (Δ = last - first)."""
        return self.end_time - self.start_time

    def closest_to(self, timestamp: EpochLike) -> EVENT:
        """Return the event closest to the given timestamp. Assumes timeseries is sorted."""
        print("closest to", timestamp)
        target = to_datetime(timestamp)
        print("converted to", target)
        target_ts = target.timestamp()

        closest = None
        min_dist = float("inf")

        for event in self:
            dist = abs(event.ts.timestamp() - target_ts)
            if dist > min_dist:
                break

            min_dist = dist
            closest = event

        print(f"closest: {closest}")
        return closest

    def __repr__(self) -> str:
        """Return a string representation of the Timeseries."""
        return f"Timeseries(date={self.start_time.strftime('%Y-%m-%d')}, start={self.start_time.strftime('%H:%M:%S')}, end={self.end_time.strftime('%H:%M:%S')}, duration={self.duration()}, events={len(list(self))}, freq={self.frequency:.2f}Hz)"

    def __str__(self) -> str:
        """Return a string representation of the Timeseries."""
        return self.__repr__()


class TList(list[EVENT], Timeseries[EVENT]):
    """A test class that inherits from both list and Timeseries."""

    def __repr__(self) -> str:
        """Return a string representation of the TList using Timeseries repr method."""
        return Timeseries.__repr__(self)
