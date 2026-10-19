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

"""Generated control values retain the sparse frame contract over CDR."""

import time

from dimos_cdrgen.registry import decode, encode
from dimos_generated.dimos_msgs.msg import ControlValues
import numpy as np
import pytest

from dimos.msgs.control import control_values, control_values_dict, validate_control_values
from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.protocol import DimosMsg


def test_cdr_round_trip_preserves_every_field() -> None:
    frame = control_values(
        "xarm",
        source_ts=1758547200.25,
        epoch=7,
        sequence=1234,
        interface_names=["xarm/j1/position", "xarm/gripper/position"],
        values=[-1.25, 0.085],
    )
    decoded = decode(encode(frame), ControlValues)
    assert (decoded.source, decoded.source_ts, decoded.epoch, decoded.sequence) == (
        "xarm",
        1758547200.25,
        7,
        1234,
    )
    assert decoded.interface_names == ["xarm/j1/position", "xarm/gripper/position"]
    np.testing.assert_array_equal(decoded.values, [-1.25, 0.085])
    assert control_values_dict(decoded) == {
        "xarm/j1/position": -1.25,
        "xarm/gripper/position": 0.085,
    }


def test_empty_heartbeat_round_trip() -> None:
    decoded = decode(
        encode(control_values("coordinator", source_ts=99.5, epoch=3, sequence=42)), ControlValues
    )
    assert (decoded.source, decoded.source_ts, decoded.epoch, decoded.sequence) == (
        "coordinator",
        99.5,
        3,
        42,
    )
    assert decoded.interface_names == []
    np.testing.assert_array_equal(decoded.values, [])
    assert control_values_dict(decoded) == {}


@pytest.mark.parametrize("names,values", [(["a", "b"], [1.0]), ([], [1.0])])
def test_mismatched_lengths_raise(names, values) -> None:
    with pytest.raises(ValueError, match="same length"):
        control_values("arm", interface_names=names, values=values)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_values_name_the_interface(bad: float) -> None:
    with pytest.raises(ValueError, match="arm/j2/effort"):
        control_values(
            "arm", interface_names=["arm/j1/position", "arm/j2/effort"], values=[0.0, bad]
        )


def test_decode_boundary_rejects_ragged_generated_frame() -> None:
    frame = ControlValues(
        source="arm",
        source_ts=1.0,
        epoch=1,
        sequence=1,
        interface_names=["a", "b"],
        values=np.array([1.0]),
    )
    decoded = decode(encode(frame), ControlValues)
    with pytest.raises(ValueError, match="same length"):
        validate_control_values(decoded)


def test_constructor_copies_arrays() -> None:
    names, values = ["a"], np.array([2.0])
    frame = control_values("arm", interface_names=names, values=values)
    names[0], values[0] = "b", 99
    assert frame.interface_names == ["a"]
    np.testing.assert_array_equal(frame.values, [2.0])


def test_only_absent_timestamp_uses_current_time() -> None:
    before = time.time()
    frame = control_values("arm")
    assert before <= frame.source_ts <= time.time()
    decoded = decode(encode(control_values("arm", source_ts=0.0)), ControlValues)
    assert decoded.source_ts == 0.0


def test_generated_type_is_the_resolved_wire_type() -> None:
    assert isinstance(control_values("arm"), DimosMsg)
    assert resolve_msg_type("dimos_msgs/msg/ControlValues") is ControlValues
