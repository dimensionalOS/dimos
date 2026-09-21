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

import pickle
import time

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.Wrench import Wrench
from dimos.msgs.geometry_msgs.WrenchStamped import WrenchStamped


def test_wrench_empty_is_zero():
    w = Wrench()
    assert w.force.to_tuple() == (0.0, 0.0, 0.0)
    assert w.torque.to_tuple() == (0.0, 0.0, 0.0)
    assert w.is_zero()
    assert not w


def test_wrench_from_two_sequences():
    w = Wrench([1, 2, 3], [4, 5, 6])
    assert w.force.to_tuple() == (1.0, 2.0, 3.0)
    assert w.torque.to_tuple() == (4.0, 5.0, 6.0)


def test_wrench_from_two_vector3():
    w = Wrench(Vector3(1, 2, 3), Vector3(4, 5, 6))
    assert w.force.to_tuple() == (1.0, 2.0, 3.0)
    assert w.torque.to_tuple() == (4.0, 5.0, 6.0)


def test_wrench_keywords():
    assert Wrench(force=[1, 2, 3]).torque.to_tuple() == (0.0, 0.0, 0.0)
    assert Wrench(torque=[4, 5, 6]).force.to_tuple() == (0.0, 0.0, 0.0)


def test_wrench_copy_constructor():
    src = Wrench([1, 2, 3], [4, 5, 6])
    copy = Wrench(src)
    assert copy == src
    assert copy.force is not src.force


def test_wrench_unknown_keyword_raises():
    with pytest.raises(TypeError):
        Wrench(bogus=1)


def test_wrench_array_roundtrip():
    ft = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
    assert np.allclose(Wrench.from_array(ft).to_array(), ft)


def test_wrench_from_array_wrong_length_raises():
    with pytest.raises(ValueError, match="6 elements"):
        Wrench.from_array([1.0, 2.0, 3.0])


def test_wrench_add_and_sub():
    a = Wrench([1, 2, 3], [4, 5, 6])
    b = Wrench([1, 1, 1], [1, 1, 1])
    assert (a + b).force.to_tuple() == (2.0, 3.0, 4.0)
    assert (a - b).torque.to_tuple() == (3.0, 4.0, 5.0)


def test_stamped_is_a_wrench():
    assert isinstance(WrenchStamped(), Wrench)


def test_stamped_inherits_wrench_positional_form():
    ws = WrenchStamped([1, 2, 3], [4, 5, 6])
    assert ws.force.to_tuple() == (1.0, 2.0, 3.0)
    assert ws.torque.to_tuple() == (4.0, 5.0, 6.0)


def test_stamped_keeps_explicit_ts_and_frame():
    ws = WrenchStamped(ts=5.0, frame_id="ft_sensor", force=[1, 2, 3], torque=[4, 5, 6])
    assert (ws.ts, ws.frame_id) == (5.0, "ft_sensor")
    assert ws.force.to_tuple() == (1.0, 2.0, 3.0)


def test_stamped_positional_ts_and_frame():
    ws = WrenchStamped(5.0, "ft_sensor")
    assert (ws.ts, ws.frame_id) == (5.0, "ft_sensor")


def test_stamped_defaults_stamp_to_now():
    before = time.time()
    assert before <= WrenchStamped().ts <= time.time()


def test_stamped_zero_ts_is_a_real_timestamp():
    assert WrenchStamped(ts=0.0).ts == 0.0


def test_stamped_from_array():
    ws = WrenchStamped.from_array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3], frame_id="tool0", ts=5.0)
    assert (ws.ts, ws.frame_id) == (5.0, "tool0")
    assert ws.force.to_tuple() == (1.0, 2.0, 3.0)
    assert ws.torque.to_tuple() == (0.1, 0.2, 0.3)


def test_lcm_encode_decode():
    source = WrenchStamped(
        ts=5.25, frame_id="ft_sensor", force=(1.0, 2.0, 3.0), torque=(0.1, 0.2, 0.3)
    )
    dest = WrenchStamped.lcm_decode(source.lcm_encode())

    assert isinstance(dest, WrenchStamped)
    assert dest is not source
    assert dest == source
    assert dest.frame_id == "ft_sensor"
    assert dest.ts == pytest.approx(5.25)


def test_lcm_decode_of_an_unstamped_message_keeps_the_zero_stamp():
    decoded = WrenchStamped.lcm_decode(WrenchStamped(ts=0.0, frame_id="f").lcm_encode())
    assert decoded.ts == 0.0


def test_pickle_encode_decode():
    source = WrenchStamped(ts=time.time(), force=(1.0, 2.0, 3.0), torque=(0.1, 0.2, 0.3))
    dest = pickle.loads(pickle.dumps(source))
    assert isinstance(dest, WrenchStamped)
    assert dest is not source
    assert dest == source
