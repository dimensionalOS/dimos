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

from copy import deepcopy

from dimos_generated.geometry_msgs.msg import Quaternion, Twist, Vector3
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.geometry import quaternion_euler, vector_array, vector_from_array


def test_twist_initialization() -> None:
    tw = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    assert tw.linear.x == 0.0
    assert tw.linear.y == 0.0
    assert tw.linear.z == 0.0
    assert tw.angular.x == 0.0
    assert tw.angular.y == 0.0
    assert tw.angular.z == 0.0
    lin = Vector3(x=1.0, y=2.0, z=3.0)
    ang = Vector3(x=0.1, y=0.2, z=0.3)
    tw2 = Twist(linear=lin, angular=ang)
    assert tw2.linear == lin
    assert tw2.angular == ang
    tw3 = deepcopy(tw2)
    assert tw3.linear == tw2.linear
    assert tw3.angular == tw2.angular
    assert tw3 == tw2
    tw3.linear.x = 10.0
    assert tw2.linear.x == 1.0
    source = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    source.linear = Vector3(x=4.0, y=5.0, z=6.0)
    source.angular = Vector3(x=0.4, y=0.5, z=0.6)
    tw4 = cdr_decode(cdr_encode(source), Twist)
    assert tw4.linear.x == 4.0
    assert tw4.linear.y == 5.0
    assert tw4.linear.z == 6.0
    assert tw4.angular.x == 0.4
    assert tw4.angular.y == 0.5
    assert tw4.angular.z == 0.6
    quat = Quaternion(x=0, y=0, z=0.707107, w=0.707107)
    tw5 = Twist(
        linear=Vector3(x=1.0, y=2.0, z=3.0), angular=vector_from_array(quaternion_euler(quat))
    )
    assert tw5.linear == Vector3(x=1.0, y=2.0, z=3.0)
    euler = vector_from_array(quaternion_euler(quat))
    assert np.allclose(tw5.angular.x, euler.x)
    assert np.allclose(tw5.angular.y, euler.y)
    assert np.allclose(tw5.angular.z, euler.z)
    tw7 = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3))
    assert tw7.linear == Vector3(x=1, y=2, z=3)
    assert tw7.angular == Vector3(x=0.1, y=0.2, z=0.3)
    tw8 = Twist(linear=Vector3(x=4, y=5, z=6), angular=Vector3(x=0.0, y=0.0, z=0.0))
    assert tw8.linear == Vector3(x=4, y=5, z=6)
    assert np.allclose(vector_array(tw8.angular), 0)
    tw9 = Twist(angular=Vector3(x=0.4, y=0.5, z=0.6), linear=Vector3(x=0.0, y=0.0, z=0.0))
    assert np.allclose(vector_array(tw9.linear), 0)
    assert tw9.angular == Vector3(x=0.4, y=0.5, z=0.6)
    tw10 = Twist(
        angular=vector_from_array(quaternion_euler(Quaternion(x=0, y=0, z=0.707107, w=0.707107))),
        linear=Vector3(x=0.0, y=0.0, z=0.0),
    )
    assert np.allclose(vector_array(tw10.linear), 0)
    euler = vector_from_array(quaternion_euler(Quaternion(x=0, y=0, z=0.707107, w=0.707107)))
    assert np.allclose(tw10.angular.x, euler.x)
    assert np.allclose(tw10.angular.y, euler.y)
    assert np.allclose(tw10.angular.z, euler.z)
    tw11 = Twist(
        linear=Vector3(x=1, y=0, z=0),
        angular=vector_from_array(quaternion_euler(Quaternion(x=0, y=0, z=0, w=1))),
    )
    assert tw11.linear == Vector3(x=1, y=0, z=0)
    assert np.allclose(vector_array(tw11.angular), 0)


def test_twist_zero() -> None:
    tw = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    assert np.allclose(vector_array(tw.linear), 0)
    assert np.allclose(vector_array(tw.angular), 0)
    assert np.allclose(np.concatenate((vector_array(tw.linear), vector_array(tw.angular))), 0)
    assert tw == Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))


def test_twist_equality() -> None:
    tw1 = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3))
    tw2 = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3))
    tw3 = Twist(linear=Vector3(x=1, y=2, z=4), angular=Vector3(x=0.1, y=0.2, z=0.3))
    tw4 = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.4))
    assert tw1 == tw2
    assert tw1 != tw3
    assert tw1 != tw4
    assert tw1 != "not a twist"


def test_twist_independent_cdr_fields() -> None:
    tw = Twist(linear=Vector3(x=1.5, y=-2.0, z=3.14), angular=Vector3(x=0.1, y=-0.2, z=0.3))
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        cdr_encode(tw), "geometry_msgs/msg/Twist"
    )
    assert [decoded.linear.x, decoded.linear.y, decoded.linear.z] == [1.5, -2.0, 3.14]
    assert [decoded.angular.x, decoded.angular.y, decoded.angular.z] == [0.1, -0.2, 0.3]


def test_twist_is_zero() -> None:
    tw1 = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    assert np.allclose(np.concatenate((vector_array(tw1.linear), vector_array(tw1.angular))), 0)
    tw2 = Twist(linear=Vector3(x=0.1, y=0, z=0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    assert not np.allclose(np.concatenate((vector_array(tw2.linear), vector_array(tw2.angular))), 0)
    tw3 = Twist(angular=Vector3(x=0, y=0, z=0.1), linear=Vector3(x=0.0, y=0.0, z=0.0))
    assert not np.allclose(np.concatenate((vector_array(tw3.linear), vector_array(tw3.angular))), 0)
    tw4 = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3))
    assert not np.allclose(np.concatenate((vector_array(tw4.linear), vector_array(tw4.angular))), 0)


def test_twist_explicit_nonzero_predicate() -> None:
    tw1 = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    assert not (np.any(vector_array(tw1.linear)) or np.any(vector_array(tw1.angular)))
    tw2 = Twist(linear=Vector3(x=1, y=0, z=0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    assert np.any(vector_array(tw2.linear)) or np.any(vector_array(tw2.angular))
    tw3 = Twist(angular=Vector3(x=0, y=0, z=0.1), linear=Vector3(x=0.0, y=0.0, z=0.0))
    assert np.any(vector_array(tw3.linear)) or np.any(vector_array(tw3.angular))
    tw4 = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3))
    assert np.any(vector_array(tw4.linear)) or np.any(vector_array(tw4.angular))


def test_twist_cdr_encoding() -> None:
    tw = Twist(linear=Vector3(x=1.5, y=2.5, z=3.5), angular=Vector3(x=0.1, y=0.2, z=0.3))
    encoded = cdr_encode(tw)
    assert isinstance(encoded, bytes)
    decoded = cdr_decode(encoded, Twist)
    assert decoded.linear == tw.linear
    assert decoded.angular == tw.angular
    assert isinstance(decoded.linear, Vector3)
    assert decoded == tw


def test_twist_with_lists() -> None:
    tw1 = Twist(linear=vector_from_array([1, 2, 3]), angular=vector_from_array([0.1, 0.2, 0.3]))
    assert tw1.linear == Vector3(x=1, y=2, z=3)
    assert tw1.angular == Vector3(x=0.1, y=0.2, z=0.3)
    tw2 = Twist(
        linear=vector_from_array(np.array([4, 5, 6])),
        angular=vector_from_array(np.array([0.4, 0.5, 0.6])),
    )
    assert tw2.linear == Vector3(x=4, y=5, z=6)
    assert tw2.angular == Vector3(x=0.4, y=0.5, z=0.6)
