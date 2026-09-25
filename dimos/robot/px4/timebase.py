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

"""The vehicle clock: boot time to UTC, and samples looked up at one vehicle instant.

PX4 stamps LOCAL_POSITION_NED and friends with time_boot_ms; SYSTEM_TIME carries boot and
unix time together, so unix - boot is the vehicle's own clock. Without a GPS clock the
fallback is the min-filtered receive_wall - boot: latency only ever makes that larger, so
the minimum over many samples is the least-biased estimate.
"""

from __future__ import annotations

import bisect
from collections import deque
from collections.abc import Iterable
from concurrent.futures import TimeoutError as FutureTimeoutError
import math
import statistics
from typing import TYPE_CHECKING, Any, Literal

from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import normalize_angle

if TYPE_CHECKING:
    from dimos.robot.px4.mavlink import MavlinkIO

logger = setup_logger()

TimebaseQuality = Literal["none", "receive_time", "system_time_partial", "system_time"]

# PX4 reports unix time as 0 / near-epoch until it has a source; anything before this
# (2020-01-01) is not a real wall clock.
_MIN_VALID_UNIX_S = 1577836800.0

_MSG_ID_SYSTEM_TIME = 2
_MAV_CMD_SET_MESSAGE_INTERVAL = 511
_MAV_RESULT_ACCEPTED = 0


def boot_s(msg: Any) -> float | None:
    """Vehicle boot time in seconds from ``time_boot_ms`` / ``time_usec``, if present."""
    ms = getattr(msg, "time_boot_ms", None)
    if ms is not None:
        return float(ms) / 1e3
    us = getattr(msg, "time_usec", None)
    if us is not None:
        return float(us) / 1e6
    return None


def request_system_time(io: MavlinkIO, hz: float, timeout_s: float) -> bool:
    """Ask PX4 to stream SYSTEM_TIME at ``hz`` (it defaults to 1 Hz)."""
    fut = io.send_command(_MAV_CMD_SET_MESSAGE_INTERVAL, float(_MSG_ID_SYSTEM_TIME), 1e6 / hz)
    try:
        return fut.result(timeout=timeout_s) == _MAV_RESULT_ACCEPTED
    except (TimeoutError, FutureTimeoutError):  # one class only from Python 3.11
        logger.warning("no ack for SET_MESSAGE_INTERVAL SYSTEM_TIME")
        return False


class Px4Timebase:
    def __init__(
        self,
        *,
        min_samples: int = 30,
        jump_guard_s: float = 0.5,
        window: int = 300,
    ) -> None:
        self._min_samples = min_samples
        self._jump_guard_s = jump_guard_s
        self._system: deque[float] = deque(maxlen=window)
        self._receive_min: float | None = None
        self._rejected_run = 0
        self.rejected = 0
        self.samples = 0

    def add_system_time(self, unix_s: float, boot_s: float, receive_wall_s: float) -> None:
        """One SYSTEM_TIME message: vehicle unix time, vehicle boot time, our receive time."""
        self.samples += 1
        fallback = receive_wall_s - boot_s
        self._receive_min = (
            fallback if self._receive_min is None else min(self._receive_min, fallback)
        )
        if unix_s < _MIN_VALID_UNIX_S:
            return
        offset = unix_s - boot_s
        if len(self._system) >= self._min_samples:
            # Jump guard: a wild sample (corrupted packet) must not drag the median; count
            # it and drop it. A run of them is the vehicle clock itself stepping (GPS time
            # arriving over an RTC): start again on the new clock.
            if abs(offset - statistics.median(self._system)) > self._jump_guard_s:
                self.rejected += 1
                self._rejected_run += 1
                if self._rejected_run < self._min_samples:
                    return
                self._system.clear()
        self._rejected_run = 0
        self._system.append(offset)

    @property
    def quality(self) -> TimebaseQuality:
        if len(self._system) >= self._min_samples:
            return "system_time"
        if self._system:
            return "system_time_partial"
        if self._receive_min is not None:
            return "receive_time"
        return "none"

    @property
    def offset_s(self) -> float:
        """Seconds to add to a vehicle boot time to get UTC."""
        if self._system:
            return statistics.median(self._system)
        if self._receive_min is not None:
            return self._receive_min
        raise RuntimeError("Px4Timebase has no samples")

    def to_utc(self, boot_s: float) -> float:
        return boot_s + self.offset_s


class TimedBuffer:
    """Ring buffer of ``(t, boot, value)`` with linear interpolation (angles wrap-aware).

    Interpolation clamps to the nearest sample outside the buffered range; it never
    extrapolates.
    """

    def __init__(self, seconds: float = 2.0, angular: Iterable[str] = ()) -> None:
        self.seconds = seconds
        self.angular = set(angular)
        self.t: deque[float] = deque()
        self.boot: deque[float | None] = deque()
        self.v: deque[dict[str, float]] = deque()

    def push(self, t: float, value: dict[str, float], boot: float | None = None) -> None:
        # Stamps step back when the timebase leaves its receive-time fallback, a clock is
        # set, or PX4 reboots. The lookups bisect and eviction needs order: start again.
        last_boot = self.boot[-1] if self.boot else None
        boot_back = boot is not None and last_boot is not None and boot < last_boot
        if self.t and (t < self.t[-1] or boot_back):
            self.t.clear()
            self.boot.clear()
            self.v.clear()
        self.t.append(t)
        self.boot.append(boot)
        self.v.append(value)
        while self.t and t - self.t[0] > self.seconds:
            self.t.popleft()
            self.boot.popleft()
            self.v.popleft()

    def at(self, t: float) -> dict[str, float] | None:
        """Interpolated value at receive time ``t``, or None if empty."""
        return self._interp(list(self.t), t)

    def at_boot(self, boot: float) -> dict[str, float] | None:
        """Interpolated value at vehicle boot time ``boot``; None if no boot stamps."""
        if any(b is None for b in self.boot):
            return None
        return self._interp([b for b in self.boot if b is not None], boot)

    def _interp(self, ts: list[float], t: float) -> dict[str, float] | None:
        if not ts:
            return None
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            return dict(self.v[0])
        if i >= len(ts):
            return dict(self.v[-1])
        t0, t1 = ts[i - 1], ts[i]
        a, b = self.v[i - 1], self.v[i]
        f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        out: dict[str, float] = {}
        for k in a:
            if k in self.angular:
                # The shortest way round, in degrees.
                out[k] = a[k] + f * math.degrees(normalize_angle(math.radians(b[k] - a[k])))
            else:
                out[k] = a[k] + f * (b[k] - a[k])
        return out
