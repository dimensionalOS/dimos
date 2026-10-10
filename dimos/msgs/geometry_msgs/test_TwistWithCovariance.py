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

"""Generated covariance values, with explicit NumPy matrix operations.

Legacy polymorphic positional constructors and presentation methods are retired.
"""

from dataclasses import asdict

from dimos_generated.geometry_msgs.msg import Twist, TwistWithCovariance, Vector3
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
from rosbags.serde import SerdeError
from rosbags.typesys import Stores, get_typestore


def test_default_fields_and_covariance() -> None:
    source = TwistWithCovariance(
        twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
        covariance=np.zeros(36, dtype=np.float64),
    )
    value = source.twist
    np.testing.assert_array_equal(
        [
            value.linear.x,
            value.linear.y,
            value.linear.z,
            value.angular.x,
            value.angular.y,
            value.angular.z,
        ],
        [0, 0, 0, 0, 0, 0],
    )
    assert np.asarray(source.covariance).shape == (36,)
    np.testing.assert_array_equal(source.covariance, np.zeros(36))


@pytest.mark.parametrize("as_list", [False, True])
@pytest.mark.parametrize(
    "covariance",
    [np.zeros(36), np.arange(36, dtype=float), np.eye(6).ravel(), np.diag(np.arange(1, 7)).ravel()],
)
def test_explicit_construction_and_independent_cdr(as_list: bool, covariance: np.ndarray) -> None:
    source = TwistWithCovariance(
        twist=Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3)),
        covariance=np.asarray(covariance.tolist() if as_list else covariance, dtype=np.float64),
    )
    decoded = cdr_decode(cdr_encode(source), TwistWithCovariance)
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        cdr_encode(source), TwistWithCovariance.__msgtype__
    )
    for result in (source, decoded, independent):
        value = result.twist
        np.testing.assert_array_equal(
            [
                value.linear.x,
                value.linear.y,
                value.linear.z,
                value.angular.x,
                value.angular.y,
                value.angular.z,
            ],
            [1, 2, 3, 0.1, 0.2, 0.3],
        )
        np.testing.assert_array_equal(result.covariance, covariance)
        matrix = np.asarray(result.covariance).reshape(6, 6)
        for row in range(6):
            for col in range(6):
                assert matrix[row, col] == covariance[row * 6 + col]
        assert np.trace(matrix) == np.trace(covariance.reshape(6, 6))


def test_copy_equality_and_independent_storage() -> None:
    original = TwistWithCovariance(
        twist=Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3)),
        covariance=np.asarray(np.arange(36, dtype=float), dtype=np.float64),
    )
    copied = cdr_decode(cdr_encode(original), TwistWithCovariance)
    np.testing.assert_equal(asdict(copied), asdict(original))
    assert copied is not original
    assert copied.twist is not original.twist
    assert copied.covariance is not original.covariance
    original.covariance[0] = 999
    assert copied.covariance[0] == 0
    assert not np.array_equal(copied.covariance, original.covariance)
    assert copied != "not a message"
    assert copied is not None


def test_matrix_assignment() -> None:
    source = TwistWithCovariance(
        covariance=np.asarray(np.arange(36, dtype=float), dtype=np.float64),
        twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
    )
    matrix = np.asarray(source.covariance).reshape(6, 6)
    assert matrix[0, 0] == 0
    assert matrix[5, 5] == 35
    source.covariance = (np.eye(6) * 2).ravel()
    np.testing.assert_array_equal(list(source.covariance)[:6], [2, 0, 0, 0, 0, 0])
    assert np.trace(np.asarray(source.covariance).reshape(6, 6)) == 12


@pytest.mark.parametrize("size", [0, 35, 37])
def test_cdr_rejects_invalid_covariance_length(size: int) -> None:
    source = TwistWithCovariance(
        covariance=np.asarray(np.zeros(size), dtype=np.float64),
        twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
    )
    with pytest.raises(SerdeError, match="Unexpected array length"):
        cdr_encode(source)


@pytest.mark.parametrize(
    "xyz,angular",
    [
        ((0, 0, 0), (0, 0, 0)),
        ((1, 2, 3), (0.1, 0.2, 0.3)),
        ((-1, -2, -3), (-0.1, -0.2, -0.3)),
        ((100, -100, 0), (3.14, -3.14, 0)),
    ],
)
def test_parameterized_values(
    xyz: tuple[float, float, float], angular: tuple[float, float, float]
) -> None:
    source = TwistWithCovariance(
        twist=Twist(
            linear=Vector3(x=xyz[0], y=xyz[1], z=xyz[2]),
            angular=Vector3(x=angular[0], y=angular[1], z=angular[2]),
        ),
        covariance=np.zeros(36, dtype=np.float64),
    )
    assert (source.twist.linear.x, source.twist.linear.y, source.twist.linear.z) == xyz
    assert (source.twist.angular.x, source.twist.angular.y, source.twist.angular.z) == angular
