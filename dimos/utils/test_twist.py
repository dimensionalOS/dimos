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

from dimos_generated.geometry_msgs.msg import Twist, TwistStamped, Vector3
import numpy as np

from dimos.utils.transform_utils import twist_to_numpy


def test_twist_to_numpy_orders_linear_then_angular():
    twist = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=4, y=5, z=6))
    result = twist_to_numpy(Twist.decode(twist.encode()))
    assert result.dtype == np.float64
    np.testing.assert_array_equal(result, [1, 2, 3, 4, 5, 6])
    result[0] = 99
    assert twist.linear.x == 1


def test_stamped_twist_uses_explicit_nested_twist():
    stamped = TwistStamped(
        twist=Twist(linear=Vector3(x=-1, z=1), angular=Vector3(x=0.1, y=0.2, z=0.3))
    )
    result = twist_to_numpy(stamped.twist)
    np.testing.assert_array_equal(result, [-1, 0, 1, 0.1, 0.2, 0.3])
