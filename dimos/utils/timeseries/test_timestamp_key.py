# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from dimos.message_utils.time import to_seconds
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, PointStamped
from dimos.messages.std_msgs.msg import Header
from dimos.types.timestamped import TimestampedBufferCollection


def test_generated_values_use_selected_stamp_for_lookup_and_window():
    store = TimestampedBufferCollection[PointStamped](
        2.0, timestamp_key=lambda value: to_seconds(value.header.stamp)
    )
    first = PointStamped(Header(Time(1, 0), "sensor"), Point(1.0, 0.0, 0.0))
    middle = PointStamped(Header(Time(2, 0), "sensor"), Point(2.0, 0.0, 0.0))
    last = PointStamped(Header(Time(4, 0), "sensor"), Point(4.0, 0.0, 0.0))
    store.add(middle)
    store.add(first)
    assert store.find_closest(1.5) is middle
    store.add(last)
    assert store.last() is last
    assert store.load(1.0) is None
    assert store.load(2.0) is middle
    assert store.remove(middle)
    assert store.load(2.0) is None
