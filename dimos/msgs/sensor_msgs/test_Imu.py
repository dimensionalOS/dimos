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

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu


def test_constructor_preserves_explicit_zero_components() -> None:
    angular_velocity = Vector3()
    linear_acceleration = Vector3()
    orientation = Quaternion(0.0, 0.0, 0.0, 0.0)

    imu = Imu(
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration,
        orientation=orientation,
    )

    assert imu.angular_velocity is angular_velocity
    assert imu.linear_acceleration is linear_acceleration
    assert imu.orientation is orientation
