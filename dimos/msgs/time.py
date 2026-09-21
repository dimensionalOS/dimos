# Copyright 2026 Dimensional Inc.
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

"""Explicit conversions for generated ROS time, duration, and header fields."""

import math
import time

from dimos_generated.builtin_interfaces.msg import Duration, Time
from dimos_generated.std_msgs.msg import Header

NANOSECONDS_PER_SECOND = 1_000_000_000


def to_nanoseconds(stamp: Time | Duration) -> int:
    """Convert a normalized ROS time or duration without losing integer precision."""
    if not 0 <= stamp.nanosec < NANOSECONDS_PER_SECOND:
        raise ValueError("nanosec must be in [0, 1000000000)")
    return stamp.sec * NANOSECONDS_PER_SECOND + stamp.nanosec


def to_seconds(stamp: Time | Duration) -> float:
    """Convert to floating seconds for APIs that require them."""
    return to_nanoseconds(stamp) / NANOSECONDS_PER_SECOND


def time_from_nanoseconds(value: int) -> Time:
    sec, nanosec = divmod(value, NANOSECONDS_PER_SECOND)
    return Time(sec=sec, nanosec=nanosec)


def duration_from_nanoseconds(value: int) -> Duration:
    sec, nanosec = divmod(value, NANOSECONDS_PER_SECOND)
    return Duration(sec=sec, nanosec=nanosec)


def _seconds_to_nanoseconds(value: float) -> int:
    if not math.isfinite(value):
        raise ValueError("seconds must be finite")
    sec = math.floor(value)
    return sec * NANOSECONDS_PER_SECOND + round((value - sec) * NANOSECONDS_PER_SECOND)


def time_from_seconds(value: float) -> Time:
    """Round to the nearest nanosecond, normalizing negative and carried fractions."""
    return time_from_nanoseconds(_seconds_to_nanoseconds(value))


def duration_from_seconds(value: float) -> Duration:
    return duration_from_nanoseconds(_seconds_to_nanoseconds(value))


def header_now(frame_id: str = "") -> Header:
    """Construct a wall-clock header; generated defaults themselves remain zero."""
    return Header(stamp=time_from_nanoseconds(time.time_ns()), frame_id=frame_id)
