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

from dimos.types.timestamped import Timestamped, to_ros_stamp, to_datetime


def test_timestamped_dt_method():
    ts = 1751075203.4120464
    timestamped = Timestamped(ts)
    dt = timestamped.dt()
    assert isinstance(dt, datetime)
    assert abs(dt.timestamp() - ts) < 1e-6
    assert dt.tzinfo is not None, "datetime should be timezone-aware"


def test_to_ros_stamp():
    """Test the to_ros_stamp function with different input types."""

    # Test with float timestamp
    ts_float = 1234567890.123456789
    result = to_ros_stamp(ts_float)
    assert result["sec"] == 1234567890
    # Float precision limitation - check within reasonable range
    assert abs(result["nanosec"] - 123456789) < 1000

    # Test with integer timestamp
    ts_int = 1234567890
    result = to_ros_stamp(ts_int)
    assert result["sec"] == 1234567890
    assert result["nanosec"] == 0

    # Test with datetime object
    dt = datetime(2009, 2, 13, 23, 31, 30, 123456, tzinfo=timezone.utc)
    result = to_ros_stamp(dt)
    assert result["sec"] == 1234567890
    assert abs(result["nanosec"] - 123456000) < 1000  # Allow small rounding error

    # Test with RosStamp (passthrough)
    ros_stamp = {"sec": 1234567890, "nanosec": 123456789}
    result = to_ros_stamp(ros_stamp)
    assert result is ros_stamp  # Should be the exact same object


def test_to_datetime():
    """Test the to_datetime function with different input types."""

    # Test with float timestamp
    ts_float = 1234567890.123456
    dt = to_datetime(ts_float)
    assert isinstance(dt, datetime)
    assert dt.tzinfo is not None  # Should have timezone
    assert abs(dt.timestamp() - ts_float) < 1e-6

    # Test with integer timestamp
    ts_int = 1234567890
    dt = to_datetime(ts_int)
    assert isinstance(dt, datetime)
    assert dt.tzinfo is not None
    assert dt.timestamp() == ts_int

    # Test with RosStamp
    ros_stamp = {"sec": 1234567890, "nanosec": 123456000}
    dt = to_datetime(ros_stamp)
    assert isinstance(dt, datetime)
    assert dt.tzinfo is not None
    expected_ts = 1234567890.123456
    assert abs(dt.timestamp() - expected_ts) < 1e-6

    # Test with datetime (already has timezone)
    dt_input = datetime(2009, 2, 13, 23, 31, 30, tzinfo=timezone.utc)
    dt_result = to_datetime(dt_input)
    assert dt_result.tzinfo is not None
    # Should convert to local timezone by default

    # Test with naive datetime (no timezone)
    dt_naive = datetime(2009, 2, 13, 23, 31, 30)
    dt_result = to_datetime(dt_naive)
    assert dt_result.tzinfo is not None

    # Test with specific timezone
    dt_utc = to_datetime(ts_float, tz=timezone.utc)
    assert dt_utc.tzinfo == timezone.utc
    assert abs(dt_utc.timestamp() - ts_float) < 1e-6
