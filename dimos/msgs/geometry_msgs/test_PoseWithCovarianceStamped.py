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

"""Nested generated Pose covariance with explicit stamps and array conversion.

Legacy inheritance, custom repr, and implicit wall-clock defaults are retired;
field values, covariance matrices, and time conversion remain checked explicitly.
"""

import pickle
import time

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import header_now, time_from_nanoseconds, time_from_seconds, to_nanoseconds


def test_generated_defaults() -> None:
    source = PoseWithCovarianceStamped(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    assert source.header.frame_id == ""
    assert to_nanoseconds(source.header.stamp) == 0
    assert isinstance(source.pose, PoseWithCovariance)
    assert isinstance(source.pose.pose, Pose)
    np.testing.assert_array_equal(source.pose.covariance, np.zeros(36))
    value = source.pose.pose
    np.testing.assert_array_equal(
        [
            value.position.x,
            value.position.y,
            value.position.z,
            value.orientation.x,
            value.orientation.y,
            value.orientation.z,
            value.orientation.w,
        ],
        [0, 0, 0, 0, 0, 0, 1],
    )


@pytest.mark.parametrize("stamp", [0, 1234567890123456789])
@pytest.mark.parametrize(
    "covariance",
    [np.zeros(36), np.eye(6).ravel(), np.arange(36, dtype=float), np.diag(np.arange(1, 7)).ravel()],
)
def test_fields_covariance_and_independent_cdr(stamp: int, covariance: np.ndarray) -> None:
    source = PoseWithCovarianceStamped(
        header=Header(stamp=time_from_nanoseconds(stamp), frame_id="camera_link"),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=1, y=2, z=3), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
            ),
            covariance=np.asarray(covariance, dtype=np.float64),
        ),
    )
    decoded = cdr_decode(cdr_encode(source), PoseWithCovarianceStamped)
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        cdr_encode(source), PoseWithCovarianceStamped.__msgtype__
    )
    for result in (decoded, independent):
        assert result.header.frame_id == "camera_link"
        assert result.header.stamp.sec * 1000000000 + result.header.stamp.nanosec == stamp
        value = result.pose.pose
        np.testing.assert_array_equal(
            [
                value.position.x,
                value.position.y,
                value.position.z,
                value.orientation.x,
                value.orientation.y,
                value.orientation.z,
                value.orientation.w,
            ],
            [1, 2, 3, 0.1, 0.2, 0.3, 0.9],
        )
        np.testing.assert_array_equal(result.pose.covariance, covariance)
        matrix = np.asarray(result.pose.covariance).reshape(6, 6)
        assert matrix.shape == (6, 6)
        assert np.trace(matrix) == np.trace(covariance.reshape(6, 6))
    copied = pickle.loads(pickle.dumps(source))
    assert copied.encode() == cdr_encode(source)
    copied.pose.covariance[0] = 999
    assert source.pose.covariance[0] == covariance[0]


def test_explicit_current_header() -> None:
    before = time.time_ns()
    source = PoseWithCovarianceStamped(
        header=header_now("test"),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    assert before <= to_nanoseconds(source.header.stamp) <= time.time_ns()
    assert source.header.frame_id == "test"


@pytest.mark.parametrize(
    "seconds,sec,nanosec",
    [
        (1234567890.0, 1234567890, 0),
        (1234567890.123456789, 1234567890, 123456789),
        (0.000000001, 0, 1),
    ],
)
def test_explicit_seconds_conversion(seconds: float, sec: int, nanosec: int) -> None:
    stamp = time_from_seconds(seconds)
    assert stamp.sec == sec
    assert abs(stamp.nanosec - nanosec) < 100


def test_seconds_outside_ros_int32_range_rejected() -> None:
    with pytest.raises((TypeError, ValueError, OverflowError)):
        time_from_seconds(9999999999.999999999)
