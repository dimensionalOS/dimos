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

import pytest

from dimos.msgs.time import (
    duration_from_nanoseconds,
    duration_from_seconds,
    header_now,
    time_from_nanoseconds,
    time_from_seconds,
    to_nanoseconds,
    to_seconds,
)


@pytest.mark.parametrize("factory", [time_from_nanoseconds, duration_from_nanoseconds])
@pytest.mark.parametrize("value", [0, -1, -1_250_000_000, 1_700_000_000_123_456_789])
def test_integer_time_round_trip_preserves_every_nanosecond(factory, value):
    stamp = factory(value)

    assert 0 <= stamp.nanosec < 1_000_000_000
    assert to_nanoseconds(stamp) == value
    assert to_nanoseconds(type(stamp).decode(stamp.encode())) == value


@pytest.mark.parametrize("factory", [time_from_seconds, duration_from_seconds])
@pytest.mark.parametrize("value,expected", [(-1.25, (-2, 750_000_000)), (0.9999999996, (1, 0))])
def test_float_conversion_normalizes_negative_values_and_fractional_carry(factory, value, expected):
    stamp = factory(value)

    assert (stamp.sec, stamp.nanosec) == expected
    assert to_seconds(stamp) == pytest.approx(value, abs=1e-9)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_seconds_are_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        time_from_seconds(value)


def test_invalid_ros_fraction_is_rejected():
    stamp = time_from_nanoseconds(0)
    stamp.nanosec = 1_000_000_000

    with pytest.raises(ValueError, match="nanosec"):
        to_nanoseconds(stamp)


def test_now_reads_integer_clock_and_preserves_frame(monkeypatch):
    monkeypatch.setattr("dimos.msgs.time.time.time_ns", lambda: 1_700_000_000_123_456_789)

    header = header_now("camera")

    assert to_nanoseconds(header.stamp) == 1_700_000_000_123_456_789
    assert header.frame_id == "camera"
