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
from typing import Generic, Iterable, Tuple, TypedDict, TypeVar, Union

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
