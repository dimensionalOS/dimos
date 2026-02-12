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

from dimos_lcm.geometry_msgs import Twist as LCMTwist
import numpy as np

from dimos.msgs.geometry_msgs import Quaternion, Twist, Vector3


def test_twist_initialization() -> None:
    # Test default initialization (zero twist)
    tw = Twist()
    assert tw.linear.x == 0.0
    assert tw.linear.y == 0.0
    assert tw.linear.z == 0.0
    assert tw.angular.x == 0.0
    assert tw.angular.y == 0.0
    assert tw.angular.z == 0.0

    # Test initialization with Vector3 linear and angular
    lin = Vector3(1.0, 2.0, 3.0)
    ang = Vector3(0.1, 0.2, 0.3)
    tw2 = Twist(lin, ang)
    assert tw2.linear == lin
    assert tw2.angular == ang

    # Test copy constructor
    tw3 = Twist(tw2)
    assert tw3.linear == tw2.linear
    assert tw3.angular == tw2.angular
    assert tw3 == tw2
    # Ensure it's a deep copy
    tw3.linear.x = 10.0
    assert tw2.linear.x == 1.0

    # Test initialization from LCM Twist
    lcm_tw = LCMTwist()
    lcm_tw.linear = Vector3(4.0, 5.0, 6.0)
    lcm_tw.angular = Vector3(0.4, 0.5, 0.6)
    tw4 = Twist(lcm_tw)
    assert tw4.linear.x == 4.0
    assert tw4.linear.y == 5.0
    assert tw4.linear.z == 6.0
    assert tw4.angular.x == 0.4
    assert tw4.angular.y == 0.5
    assert tw4.angular.z == 0.6

    # Test initialization with linear and angular as quaternion
    quat = Quaternion(0, 0, 0.707107, 0.707107)  # 90 degrees around Z
    tw5 = Twist(Vector3(1.0, 2.0, 3.0), quat)
    assert tw5.linear == Vector3(1.0, 2.0, 3.0)
    # Quaternion should be converted to euler angles
    euler = quat.to_euler()
    assert np.allclose(tw5.angular.x, euler.x)
    assert np.allclose(tw5.angular.y, euler.y)
    assert np.allclose(tw5.angular.z, euler.z)

    # Test keyword argument initialization
    tw7 = Twist(linear=Vector3(1, 2, 3), angular=Vector3(0.1, 0.2, 0.3))
    assert tw7.linear == Vector3(1, 2, 3)
    assert tw7.angular == Vector3(0.1, 0.2, 0.3)

    # Test keyword with only linear
    tw8 = Twist(linear=Vector3(4, 5, 6))
    assert tw8.linear == Vector3(4, 5, 6)
    assert tw8.angular.is_zero()

    # Test keyword with only angular
    tw9 = Twist(angular=Vector3(0.4, 0.5, 0.6))
    assert tw9.linear.is_zero()
    assert tw9.angular == Vector3(0.4, 0.5, 0.6)

    # Test keyword with angular as quaternion
    tw10 = Twist(angular=Quaternion(0, 0, 0.707107, 0.707107))
    assert tw10.linear.is_zero()
    euler = Quaternion(0, 0, 0.707107, 0.707107).to_euler()
    assert np.allclose(tw10.angular.x, euler.x)
    assert np.allclose(tw10.angular.y, euler.y)
    assert np.allclose(tw10.angular.z, euler.z)

    # Test keyword with linear and angular as quaternion
    tw11 = Twist(linear=Vector3(1, 0, 0), angular=Quaternion(0, 0, 0, 1))
    assert tw11.linear == Vector3(1, 0, 0)
    assert tw11.angular.is_zero()  # Identity quaternion -> zero euler angles


def test_twist_zero() -> None:
    # Test zero class method
    tw = Twist.zero()
    assert tw.linear.is_zero()
    assert tw.angular.is_zero()
    assert tw.is_zero()

    # Zero should equal default constructor
    assert tw == Twist()


def test_twist_equality() -> None:
    tw1 = Twist(Vector3(1, 2, 3), Vector3(0.1, 0.2, 0.3))
    tw2 = Twist(Vector3(1, 2, 3), Vector3(0.1, 0.2, 0.3))
    tw3 = Twist(Vector3(1, 2, 4), Vector3(0.1, 0.2, 0.3))  # Different linear z
    tw4 = Twist(Vector3(1, 2, 3), Vector3(0.1, 0.2, 0.4))  # Different angular z

    assert tw1 == tw2
    assert tw1 != tw3
    assert tw1 != tw4
    assert tw1 != "not a twist"


def test_twist_string_representations() -> None:
    tw = Twist(Vector3(1.5, -2.0, 3.14), Vector3(0.1, -0.2, 0.3))

    # Test repr
    repr_str = repr(tw)
    assert "Twist" in repr_str
    assert "linear=" in repr_str
    assert "angular=" in repr_str
    assert "1.5" in repr_str
    assert "0.1" in repr_str

    # Test str
    str_str = str(tw)
    assert "Twist:" in str_str
    assert "Linear:" in str_str
    assert "Angular:" in str_str


def test_twist_is_zero() -> None:
    # Test zero twist
    tw1 = Twist()
    assert tw1.is_zero()

    # Test non-zero linear
    tw2 = Twist(linear=Vector3(0.1, 0, 0))
    assert not tw2.is_zero()

    # Test non-zero angular
    tw3 = Twist(angular=Vector3(0, 0, 0.1))
    assert not tw3.is_zero()

    # Test both non-zero
    tw4 = Twist(Vector3(1, 2, 3), Vector3(0.1, 0.2, 0.3))
    assert not tw4.is_zero()


def test_twist_bool() -> None:
    # Test zero twist is False
    tw1 = Twist()
    assert not tw1

    # Test non-zero twist is True
    tw2 = Twist(linear=Vector3(1, 0, 0))
    assert tw2

    tw3 = Twist(angular=Vector3(0, 0, 0.1))
    assert tw3

    tw4 = Twist(Vector3(1, 2, 3), Vector3(0.1, 0.2, 0.3))
    assert tw4


def test_twist_lcm_encoding() -> None:
    # Test encoding and decoding
    tw = Twist(Vector3(1.5, 2.5, 3.5), Vector3(0.1, 0.2, 0.3))

    # Encode
    encoded = tw.lcm_encode()
    assert isinstance(encoded, bytes)

    # Decode
    decoded = Twist.lcm_decode(encoded)
    assert decoded.linear == tw.linear
    assert decoded.angular == tw.angular

    assert isinstance(decoded.linear, Vector3)
    assert decoded == tw


def test_twist_with_lists() -> None:
    # Test initialization with lists instead of Vector3
    tw1 = Twist(linear=[1, 2, 3], angular=[0.1, 0.2, 0.3])
    assert tw1.linear == Vector3(1, 2, 3)
    assert tw1.angular == Vector3(0.1, 0.2, 0.3)

    # Test with numpy arrays
    tw2 = Twist(linear=np.array([4, 5, 6]), angular=np.array([0.4, 0.5, 0.6]))
    assert tw2.linear == Vector3(4, 5, 6)
    assert tw2.angular == Vector3(0.4, 0.5, 0.6)
