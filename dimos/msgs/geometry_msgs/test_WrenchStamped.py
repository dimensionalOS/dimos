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

"""Generated force/torque values and explicit external numeric operations."""

import pickle
import time

from dimos_generated.geometry_msgs.msg import Vector3, Wrench, WrenchStamped
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.geometry import wrench_array, wrench_from_array
from dimos.msgs.time import header_now, time_from_seconds, to_nanoseconds


def test_wrench_empty_is_zero() -> None:
    np.testing.assert_array_equal(wrench_array(Wrench()), np.zeros(6))


def test_wrench_from_array_and_vectors() -> None:
    source = wrench_from_array([1, 2, 3, 4, 5, 6])
    explicit = Wrench(force=Vector3(x=1, y=2, z=3), torque=Vector3(x=4, y=5, z=6))
    assert source == explicit
    np.testing.assert_array_equal(wrench_array(source), [1, 2, 3, 4, 5, 6])


def test_wrench_keywords() -> None:
    assert Wrench(force=Vector3(x=1, y=2, z=3)).torque == Vector3()
    assert Wrench(torque=Vector3(x=4, y=5, z=6)).force == Vector3()
    with pytest.raises(TypeError):
        Wrench(bogus=1)


def test_wrench_copy_storage() -> None:
    source = wrench_from_array([1, 2, 3, 4, 5, 6])
    copied = Wrench.decode(source.encode())
    assert copied == source
    copied.force.x = 10
    assert source.force.x == 1
    array = wrench_array(source)
    array[0] = 20
    assert source.force.x == 1


def test_wrench_array_roundtrip() -> None:
    values = [1, 2, 3, 0.1, 0.2, 0.3]
    np.testing.assert_allclose(wrench_array(wrench_from_array(values)), values)


@pytest.mark.parametrize("values", [[1, 2, 3], [], [[1, 2, 3], [4, 5, 6]]])
def test_wrench_from_array_wrong_shape_raises(values) -> None:
    with pytest.raises(ValueError, match="6 elements"):
        wrench_from_array(values)


def test_wrench_add_and_sub() -> None:
    a = wrench_from_array([1, 2, 3, 4, 5, 6])
    b = wrench_from_array([1, 1, 1, 1, 1, 1])
    added = wrench_from_array(wrench_array(a) + wrench_array(b))
    subtracted = wrench_from_array(wrench_array(a) - wrench_array(b))
    assert (added.force.x, added.force.y, added.force.z) == (2, 3, 4)
    assert (subtracted.torque.x, subtracted.torque.y, subtracted.torque.z) == (3, 4, 5)


def test_stamped_nested_wrench_and_explicit_header() -> None:
    source = WrenchStamped(
        header=Header(stamp=time_from_seconds(5), frame_id="tool0"),
        wrench=wrench_from_array([1, 2, 3, 0.1, 0.2, 0.3]),
    )
    assert isinstance(source.wrench, Wrench)
    assert to_nanoseconds(source.header.stamp) == 5000000000
    assert source.header.frame_id == "tool0"
    np.testing.assert_array_equal(wrench_array(source.wrench), [1, 2, 3, 0.1, 0.2, 0.3])


def test_explicit_current_time_and_zero_default() -> None:
    before = time.time_ns()
    source = WrenchStamped(header=header_now())
    assert before <= to_nanoseconds(source.header.stamp) <= time.time_ns()
    assert to_nanoseconds(WrenchStamped().header.stamp) == 0


@pytest.mark.parametrize("stamp", [0, 5.25])
def test_cdr_and_independent_decoding(stamp: float) -> None:
    source = WrenchStamped(
        header=Header(stamp=time_from_seconds(stamp), frame_id="ft_sensor"),
        wrench=wrench_from_array([1, 2, 3, 0.1, 0.2, 0.3]),
    )
    decoded = WrenchStamped.decode(source.encode())
    assert decoded is not source
    assert decoded == source
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        source.encode(), WrenchStamped.msg_name
    )
    assert independent.header.frame_id == "ft_sensor"
    assert independent.header.stamp.sec * 1000000000 + independent.header.stamp.nanosec == int(
        stamp * 1000000000
    )
    np.testing.assert_array_equal(wrench_array(independent.wrench), [1, 2, 3, 0.1, 0.2, 0.3])


def test_pickle_encode_decode() -> None:
    source = WrenchStamped(header=header_now(), wrench=wrench_from_array([1, 2, 3, 0.1, 0.2, 0.3]))
    decoded = pickle.loads(pickle.dumps(source))
    assert decoded is not source
    assert decoded == source
