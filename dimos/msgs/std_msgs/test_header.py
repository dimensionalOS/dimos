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

"""ROS2 headers use explicit time conversion; no legacy sequence counter."""

from datetime import datetime, timezone
import time

from dimos_generated.std_msgs.msg import Header
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import header_now, time_from_nanoseconds, time_from_seconds, to_seconds


def test_header_initialization_methods() -> None:
    header = Header(stamp=time_from_seconds(123.456), frame_id="world")
    assert header.stamp.sec == 123
    assert header.stamp.nanosec == 456000000
    assert header.frame_id == "world"
    before = time.time_ns()
    current = header_now("base_link")
    after = time.time_ns()
    assert before <= current.stamp.sec * 1000000000 + current.stamp.nanosec <= after
    assert current.frame_id == "base_link"
    empty = Header()
    assert empty.stamp.sec == empty.stamp.nanosec == 0
    assert empty.frame_id == ""
    dt = datetime(2025, 1, 18, 12, 30, 45, 500000, tzinfo=timezone.utc)
    dated = Header(stamp=time_from_seconds(dt.timestamp()), frame_id="sensor")
    assert dated.frame_id == "sensor"
    assert abs(to_seconds(dated.stamp) - dt.timestamp()) < 1e-6
    custom = Header(stamp=time_from_seconds(999.123), frame_id="custom")
    assert custom.stamp.sec == 999
    assert custom.stamp.nanosec == 123000000
    assert custom.frame_id == "custom"


def test_header_datetime_conversion() -> None:
    header = Header(stamp=time_from_nanoseconds(1234567890123456789), frame_id="test")
    dt = datetime.fromtimestamp(to_seconds(header.stamp), tz=timezone.utc)
    assert abs(dt.timestamp() - 1234567890.123456789) < 1e-6
    assert header.stamp.nanosec == 123456789


def test_header_independent_cdr_fields() -> None:
    header = Header(stamp=time_from_seconds(100.5), frame_id="map")
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(header.encode(), Header.msg_name)
    assert decoded.stamp.sec == 100
    assert decoded.stamp.nanosec == 500000000
    assert decoded.frame_id == "map"
    assert "seq" not in Header.schema
