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

from datetime import datetime
import time

from dimos.msgs.std_msgs import Header


def test_header_initialization_methods() -> None:
    """Test various ways to initialize a Header."""

    # Method 1: With timestamp and frame_id
    header1 = Header(123.456, "world")
    assert header1.seq == 1
    assert header1.stamp.sec == 123
    assert header1.stamp.nsec == 456000000
    assert header1.frame_id == "world"

    # Method 2: With just frame_id (uses current time)
    header2 = Header("base_link")
    assert header2.seq == 1
    assert header2.frame_id == "base_link"
    # Timestamp should be close to current time
    assert abs(header2.timestamp - time.time()) < 0.1

    # Method 3: Empty header (current time, empty frame_id)
    header3 = Header()
    assert header3.seq == 0
    assert header3.frame_id == ""

    # Method 4: With datetime object
    dt = datetime(2025, 1, 18, 12, 30, 45, 500000)  # 500ms
    header4 = Header(dt, "sensor")
    assert header4.seq == 1
    assert header4.frame_id == "sensor"
    expected_timestamp = dt.timestamp()
    assert abs(header4.timestamp - expected_timestamp) < 1e-6

    # Method 5: With custom seq number
    header5 = Header(999.123, "custom", seq=42)
    assert header5.seq == 42
    assert header5.stamp.sec == 999
    assert header5.stamp.nsec == 123000000
    assert header5.frame_id == "custom"

    # Method 6: Using now() class method
    header6 = Header.now("camera")
    assert header6.seq == 1
    assert header6.frame_id == "camera"
    assert abs(header6.timestamp - time.time()) < 0.1

    # Method 7: now() with custom seq
    header7 = Header.now("lidar", seq=99)
    assert header7.seq == 99
    assert header7.frame_id == "lidar"


def test_header_properties() -> None:
    """Test Header property accessors."""
    header = Header(1234567890.123456789, "test")

    # Test timestamp property
    assert abs(header.timestamp - 1234567890.123456789) < 1e-6

    # Test datetime property
    dt = header.datetime
    assert isinstance(dt, datetime)
    assert abs(dt.timestamp() - 1234567890.123456789) < 1e-6


def test_header_string_representation() -> None:
    """Test Header string representations."""
    header = Header(100.5, "map", seq=10)

    # Test __str__
    str_repr = str(header)
    assert "seq=10" in str_repr
    assert "time=100.5" in str_repr
    assert "frame_id='map'" in str_repr

    # Test __repr__
    repr_str = repr(header)
    assert "Header(" in repr_str
    assert "seq=10" in repr_str
    assert "Time(sec=100, nsec=500000000)" in repr_str
    assert "frame_id='map'" in repr_str
