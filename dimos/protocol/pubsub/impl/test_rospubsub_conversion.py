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

"""ROS <-> LCM field matching and array copying, testable without a ROS install."""

from __future__ import annotations

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.protocol.pubsub.impl.rospubsub_conversion import _lcm_field_for


def test_camera_info_matrices_resolve_across_the_case_change():
    """ROS 2 lowercased CameraInfo's ``D K R P``; LCM kept the capitals."""
    info = CameraInfo()
    assert _lcm_field_for("k", info) == "K"
    assert _lcm_field_for("d", info) == "D"
    assert _lcm_field_for("r", info) == "R"
    assert _lcm_field_for("p", info) == "P"


def test_an_exact_match_always_wins():
    # The fallback must never pull a differently-named field into place.
    info = CameraInfo()
    assert _lcm_field_for("width", info) == "width"
    assert _lcm_field_for("height", info) == "height"
    assert _lcm_field_for("distortion_model", info) == "distortion_model"


def test_a_field_in_neither_spelling_is_skipped():
    assert _lcm_field_for("not_a_field_at_all", CameraInfo()) is None


def test_the_explicit_map_still_takes_precedence():
    """`nanosec` -> `nsec` is a rename, so it must go through the explicit map."""
    from dimos.msgs.std_msgs.Header import Header

    assert _lcm_field_for("nanosec", Header().stamp) == "nsec"


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


def test_a_float_matrix_is_copied_as_numbers_not_bytes():
    """A float64 array also has tobytes(), which must not turn K into raw IEEE bytes."""

    class Target:
        def __init__(self):
            self.K = [0.0] * 9

    class Source:
        def get_fields_and_field_types(self):
            return {"k": "double[9]"}

        k = _FakeNumericArray([1012.59, 0.0, 962.21, 0.0, 1012.13, 765.67, 0.0, 0.0, 1.0])

    from dimos.protocol.pubsub.impl.rospubsub_conversion import _copy_ros_to_lcm_recursive

    target = Target()
    _copy_ros_to_lcm_recursive(Source(), target)
    assert target.K[0] == 1012.59
    assert target.K[4] == 1012.13


def test_byte_data_is_still_copied_as_bytes():
    class Target:
        def __init__(self):
            self.data = b""
            self.data_length = 0

    class Source:
        def get_fields_and_field_types(self):
            return {"data": "uint8[]"}

        data = _FakeByteArray([1, 2, 3, 4])

    from dimos.protocol.pubsub.impl.rospubsub_conversion import _copy_ros_to_lcm_recursive

    target = Target()
    _copy_ros_to_lcm_recursive(Source(), target)
    assert target.data == b"\x01\x02\x03\x04"
    assert target.data_length == 4
