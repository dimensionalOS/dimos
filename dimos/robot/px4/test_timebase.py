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

"""The vehicle clock: SYSTEM_TIME offset, its fallback, and the timed buffer."""

from __future__ import annotations

from concurrent.futures import Future
from unittest.mock import MagicMock

import pytest

from dimos.robot.px4.timebase import Px4Timebase, TimedBuffer, request_system_time

_UNIX = 1_800_000_000.0


def test_median_offset_and_jump_guard() -> None:
    tb = Px4Timebase(min_samples=3, jump_guard_s=0.5)
    assert tb.quality == "none"
    with pytest.raises(RuntimeError):
        tb.to_utc(0.0)
    for boot in (10.0, 11.0, 12.0):
        tb.add_system_time(_UNIX + boot, boot, receive_wall_s=_UNIX + boot + 0.02)
    assert tb.quality == "system_time"
    assert tb.offset_s == _UNIX
    # A wild sample is rejected, not averaged in.
    tb.add_system_time(_UNIX + 13.0 + 5.0, 13.0, receive_wall_s=_UNIX + 13.02)
    assert tb.rejected == 1
    assert tb.offset_s == _UNIX
    assert tb.to_utc(20.0) == _UNIX + 20.0


def test_a_clock_that_steps_for_good_is_adopted() -> None:
    tb = Px4Timebase(min_samples=3, jump_guard_s=0.5)
    for boot in (10.0, 11.0, 12.0):
        tb.add_system_time(_UNIX + boot, boot, receive_wall_s=_UNIX + boot)
    tb.add_system_time(_UNIX + 13.0 + 9.0, 13.0, receive_wall_s=_UNIX + 13.0)  # one outlier
    tb.add_system_time(_UNIX + 14.0, 14.0, receive_wall_s=_UNIX + 14.0)
    assert tb.offset_s == _UNIX and tb.rejected == 1
    for boot in (15.0, 16.0, 17.0, 18.0, 19.0):  # PX4 took GPS time: +2 s, and it stays
        tb.add_system_time(_UNIX + 2.0 + boot, boot, receive_wall_s=_UNIX + boot)
    assert tb.offset_s == _UNIX + 2.0
    assert tb.quality == "system_time"


def test_a_missing_ack_is_false_not_an_exception() -> None:
    io = MagicMock()
    io.send_command.return_value = Future()  # never resolved
    assert request_system_time(io, 10.0, timeout_s=0.01) is False
    command, msg_id, interval_us = io.send_command.call_args.args
    assert (command, msg_id, interval_us) == (511, 2.0, 1e5)


def test_partial_quality_below_min_samples() -> None:
    tb = Px4Timebase(min_samples=30)
    tb.add_system_time(_UNIX + 1.0, 1.0, receive_wall_s=_UNIX + 1.01)
    assert tb.quality == "system_time_partial"
    assert tb.offset_s == _UNIX


def test_receive_time_fallback_is_min_filtered() -> None:
    tb = Px4Timebase(min_samples=3)
    # No GPS: PX4 reports unix time 0. Latency varies 10..50 ms; min wins.
    for boot, latency in ((1.0, 0.05), (2.0, 0.01), (3.0, 0.03)):
        tb.add_system_time(0.0, boot, receive_wall_s=_UNIX + boot + latency)
    assert tb.quality == "receive_time"
    assert tb.offset_s == pytest.approx(_UNIX + 0.01)


def test_timed_buffer_interpolates_and_wraps_angles() -> None:
    buf = TimedBuffer(2.0, angular=("yaw",))
    buf.push(10.0, {"yaw": 170.0, "x": 0.0}, boot=100.0)
    buf.push(11.0, {"yaw": -170.0, "x": 2.0}, boot=101.0)
    mid = buf.at(10.5)
    assert mid is not None
    assert mid["x"] == 1.0
    assert mid["yaw"] == pytest.approx(180.0)  # shortest way round, not the -0 average
    by_boot = buf.at_boot(100.25)
    assert by_boot is not None
    assert by_boot["x"] == 0.5
    assert buf.at(5.0) == {"yaw": 170.0, "x": 0.0}  # clamped, never extrapolated
    assert buf.at(20.0) == {"yaw": -170.0, "x": 2.0}


def test_timed_buffer_evicts_old_samples() -> None:
    buf = TimedBuffer(1.0)
    buf.push(0.0, {"v": 0.0})
    buf.push(0.5, {"v": 1.0})
    buf.push(1.6, {"v": 2.0})
    assert list(buf.t) == [1.6]


def test_timed_buffer_starts_again_when_stamps_step_back() -> None:
    # The timebase leaving its receive-time fallback moves every later stamp back at once.
    buf = TimedBuffer(2.0)
    for i in range(10):
        buf.push(100.0 + 0.1 * i, {"v": float(i)})
    for i in range(10, 15):
        buf.push(99.0 + 0.1 * i, {"v": float(i)})  # one second back
    assert list(buf.t) == sorted(buf.t) and len(buf.t) == 5
    assert buf.at(100.25) == {"v": pytest.approx(12.5)}  # not a sample from before the step
    buf.push(100.5, {"v": 15.0}, boot=50.0)
    buf.push(100.6, {"v": 16.0}, boot=0.5)  # PX4 rebooted
    assert list(buf.boot) == [0.5] and buf.at_boot(0.5) == {"v": 16.0}
