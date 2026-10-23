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


"""Timestamp retention and ownership, independently of transport/graph migration."""

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.tf.buffer import TBuffer


def test_buffer_retains_distinct_nanoseconds_and_owns_its_snapshots():
    buffer = TBuffer()
    first = TransformStamped(
        header=Header(frame_id="a", stamp=Time(sec=1700000000, nanosec=123456789)),
        child_frame_id="b",
        transform=Transform(
            translation=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    second = TransformStamped(
        header=Header(frame_id="a", stamp=Time(sec=1700000000, nanosec=123456790)),
        child_frame_id="b",
        transform=Transform(
            translation=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    buffer.add(first)
    buffer.add(second)
    first.header.stamp.nanosec = 0
    assert len(buffer) == 2
    stored_first = buffer.first()
    assert stored_first is not None
    assert stored_first.header.stamp.nanosec == 123456789
    stored_first.header.stamp.nanosec = 1
    unchanged = buffer.first()
    assert unchanged is not None
    assert unchanged.header.stamp.nanosec == 123456789
    latest = buffer.get()
    assert latest is not None
    assert latest.header.stamp.nanosec == 123456790


def test_buffer_drops_out_of_order_data_older_than_retained_window():
    buffer = TBuffer(buffer_size=1)
    for sec in (10, 11, 1):
        buffer.add(
            TransformStamped(
                header=Header(stamp=Time(sec=sec, nanosec=0), frame_id=""),
                child_frame_id="",
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )
        )
    assert len(buffer) == 2
    first = buffer.first()
    assert first is not None
    assert first.header.stamp.sec == 10


def test_buffer_preserves_negative_stamp():
    buffer = TBuffer()
    buffer.add(
        TransformStamped(
            header=Header(stamp=Time(sec=-1, nanosec=500000000), frame_id=""),
            child_frame_id="",
            transform=Transform(
                translation=Vector3(x=0.0, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
    )
    result = buffer.get(time_point=-0.5, time_tolerance=0)
    assert result is not None
    assert result.header.stamp == Time(sec=-1, nanosec=500000000)
