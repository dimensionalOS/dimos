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

"""ROS -> LCM copying, testable without a ROS install."""

from __future__ import annotations

from dimos_lcm.sensor_msgs import CameraInfo

from dimos.protocol.pubsub.impl.rospubsub_conversion import _copy_ros_to_lcm_recursive


class _FakeNumericArray:
    """Stands in for the float64 ndarray ROS 2 uses for CameraInfo.k."""

    itemsize = 8

    def __init__(self, values):
        self._values = list(values)

    def tolist(self):
        return list(self._values)

    def tobytes(self):
        raise AssertionError("a numeric array must not be taken as raw bytes")


class _FakeByteArray:
    """Stands in for the array.array('B') ROS 2 uses for Image.data."""

    itemsize = 1

    def __init__(self, values):
        self._values = bytes(values)

    def tolist(self):
        raise AssertionError("image data must not be expanded into a list")

    def tobytes(self):
        return self._values


def test_a_ros_camera_info_lands_in_the_lcm_capitals_as_numbers():
    """ROS 2 lowercased CameraInfo's k/p; LCM kept K/P. Their float64 arrays also have
    tobytes(), which must not turn the matrices into raw IEEE bytes."""

    class Source:
        def get_fields_and_field_types(self):
            return {"width": "uint32", "k": "double[9]", "p": "double[12]"}

        width = 1920
        k = _FakeNumericArray([1012.59, 0.0, 962.21, 0.0, 1012.13, 765.67, 0.0, 0.0, 1.0])
        p = _FakeNumericArray(
            [1012.59, 0.0, 962.21, 0.0, 0.0, 1012.13, 765.67, 0.0, 0.0, 0.0, 1.0, 0.0]
        )

    target = CameraInfo()
    _copy_ros_to_lcm_recursive(Source(), target)
    assert target.width == 1920
    assert target.K[0] == 1012.59
    assert target.K[4] == 1012.13
    assert target.P[6] == 765.67


def test_byte_data_is_still_copied_as_bytes():
    class Target:
        def __init__(self):
            self.data = b""
            self.data_length = 0

    class Source:
        def get_fields_and_field_types(self):
            return {"data": "uint8[]"}

        data = _FakeByteArray([1, 2, 3, 4])

    target = Target()
    _copy_ros_to_lcm_recursive(Source(), target)
    assert target.data == b"\x01\x02\x03\x04"
    assert target.data_length == 4
