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

"""Explicit line messages replace the old Path/orientation weight encoding."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import (
    decode as cdr_decode,
    encode as cdr_encode,
    schema as cdr_schema,
)
import numpy as np
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds


@pytest.mark.parametrize("count", [0, 1, 50])
def test_cdr_segment_layout_and_independent_schema(count: int) -> None:
    source = LineSegments3D(
        header=Header(stamp=time_from_nanoseconds(12500000000), frame_id="odom"),
        segments=[
            LineSegment3D(
                start=Point(x=2 * i, y=i, z=-1),
                end=Point(x=2 * i + 1, y=(2 * i + 1) * 0.5, z=-1),
                weight=i * 0.1,
            )
            for i in range(count)
        ],
    )
    decoded = cdr_decode(cdr_encode(source), LineSegments3D)
    assert decoded.header.frame_id == "odom"
    assert to_nanoseconds(decoded.header.stamp) == 12500000000
    assert len(decoded.segments) == count
    coordinates = np.array(
        [[[s.start.x, s.start.y, s.start.z], [s.end.x, s.end.y, s.end.z]] for s in decoded.segments]
    ).reshape(-1, 2, 3)
    indices = np.arange(2 * count, dtype=np.float64)
    expected = np.stack([indices, indices * 0.5, np.full_like(indices, -1)], axis=1).reshape(
        -1, 2, 3
    )
    np.testing.assert_array_equal(coordinates, expected)
    np.testing.assert_allclose([s.weight for s in decoded.segments], np.arange(count) * 0.1)
    store = get_typestore(Stores.ROS2_JAZZY)
    store.register(
        get_types_from_msg(cdr_schema(LineSegments3D.__msgtype__), LineSegments3D.__msgtype__)
    )
    independent = store.deserialize_cdr(cdr_encode(source), LineSegments3D.__msgtype__)
    assert len(independent.segments) == count
    assert independent.header.frame_id == "odom"
    for i, segment in enumerate(independent.segments):
        assert segment.start.x == 2 * i
        assert segment.end.y == (2 * i + 1) * 0.5
        assert segment.weight == pytest.approx(i * 0.1)


@pytest.mark.parametrize("frame", ["odom", "map", "base_link"])
def test_explicit_segment_schema_accepts_variable_frame_lengths(frame: str) -> None:
    source = LineSegments3D(
        header=Header(frame_id=frame, stamp=Time(sec=0, nanosec=0)),
        segments=[
            LineSegment3D(
                weight=0.7, start=Point(x=0.0, y=0.0, z=0.0), end=Point(x=0.0, y=0.0, z=0.0)
            )
        ],
    )
    decoded = cdr_decode(cdr_encode(source), LineSegments3D)
    assert decoded.header.frame_id == frame
    assert decoded.segments[0].weight == pytest.approx(0.7)
