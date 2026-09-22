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

"""Wire contract for control_msgs.ControlValues.

The byte-identity test is the load-bearing one: everything downstream of this
PR assumes the wrapper is a drop-in for the generated type on the wire.
"""

import time

from dimos_lcm.control_msgs import ControlValues as LCMControlValues
import pytest

from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.protocol import DimosMsg

FIXED_FRAME = {
    "source": "xarm",
    "source_ts": 1758547200.25,
    "epoch": 7,
    "sequence": 1234,
    "interface_names": ["xarm/j1/position", "xarm/j2/position", "xarm/gripper/position"],
    "values": [0.5, -1.25, 0.085],
}


def test_round_trip() -> None:
    """Every field survives encode/decode unchanged."""
    original = ControlValues(**FIXED_FRAME)  # type: ignore[arg-type]

    decoded = ControlValues.lcm_decode(original.lcm_encode())

    assert decoded.source == original.source
    assert decoded.source_ts == original.source_ts
    assert decoded.epoch == original.epoch
    assert decoded.sequence == original.sequence
    assert decoded.interface_names == original.interface_names
    assert decoded.values == original.values


def test_empty_frame_round_trip() -> None:
    """The coordinator heartbeat: an epoch and a sequence, commanding nothing."""
    heartbeat = ControlValues("coordinator", source_ts=99.5, epoch=3, sequence=42)
    assert heartbeat.interface_names == []
    assert heartbeat.values == []

    decoded = ControlValues.lcm_decode(heartbeat.lcm_encode())

    assert decoded.source == "coordinator"
    assert decoded.source_ts == 99.5
    assert decoded.epoch == 3
    assert decoded.sequence == 42
    assert decoded.interface_names == []
    assert decoded.values == []
    assert decoded.as_dict() == {}


def test_mismatched_lengths_raise() -> None:
    """Parallel arrays that are not parallel are rejected at construction."""
    with pytest.raises(ValueError, match="same length"):
        ControlValues("arm", interface_names=["a", "b"], values=[1.0])

    with pytest.raises(ValueError, match="same length"):
        ControlValues("arm", interface_names=[], values=[1.0])


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_raises(bad: float) -> None:
    """NaN and both infinities are rejected, and the message names the interface."""
    with pytest.raises(ValueError, match="arm/j2/effort"):
        ControlValues(
            "arm", interface_names=["arm/j1/position", "arm/j2/effort"], values=[0.0, bad]
        )


def test_decode_rejects_mismatched_wire_frame() -> None:
    """A frame whose arrays disagree on the wire does not become an object."""
    malformed = LCMControlValues(
        source="arm",
        source_ts=1.0,
        epoch=1,
        sequence=1,
        interface_names_length=2,
        values_length=1,
        interface_names=["a", "b"],
        values=[1.0],
    )

    with pytest.raises(ValueError, match="same length"):
        ControlValues.lcm_decode(malformed.lcm_encode())


def test_bytes_identical_to_generated_type() -> None:
    """The wrapper is a drop-in for the generated type on the wire."""
    generated = LCMControlValues(
        source=FIXED_FRAME["source"],
        source_ts=FIXED_FRAME["source_ts"],
        epoch=FIXED_FRAME["epoch"],
        sequence=FIXED_FRAME["sequence"],
        interface_names_length=len(FIXED_FRAME["interface_names"]),
        values_length=len(FIXED_FRAME["values"]),
        interface_names=list(FIXED_FRAME["interface_names"]),
        values=list(FIXED_FRAME["values"]),
    )

    assert ControlValues(**FIXED_FRAME).lcm_encode() == generated.lcm_encode()  # type: ignore[arg-type]


def test_as_dict() -> None:
    """as_dict pairs the two arrays positionally."""
    frame = ControlValues(**FIXED_FRAME)  # type: ignore[arg-type]

    assert frame.as_dict() == {
        "xarm/j1/position": 0.5,
        "xarm/j2/position": -1.25,
        "xarm/gripper/position": 0.085,
    }


def test_source_ts_defaults_to_now() -> None:
    """An unstamped frame is stamped at construction, not left at zero."""
    before = time.time()
    frame = ControlValues("arm")
    after = time.time()

    assert before <= frame.source_ts <= after


def test_source_ts_zero_is_kept_not_replaced() -> None:
    """0.0 is a real stamp: only None means "stamp me now"."""
    assert ControlValues("arm", source_ts=0.0).source_ts == 0.0
    assert (
        ControlValues.lcm_decode(ControlValues("arm", source_ts=0.0).lcm_encode()).source_ts == 0.0
    )


def test_repr_names_every_field() -> None:
    """A logged frame shows the epoch and sequence, which is what debugging needs."""
    text = repr(
        ControlValues(
            "arm", source_ts=1.5, epoch=7, sequence=3, interface_names=["a"], values=[2.0]
        )
    )

    assert text.startswith("ControlValues(")
    for fragment in ("source='arm'", "source_ts=1.5", "epoch=7", "sequence=3", "['a']", "[2.0]"):
        assert fragment in text, fragment


def test_satisfies_dimos_msg_protocol() -> None:
    """The wrapper is usable anywhere dimos.msgs.protocol.DimosMsg is required."""
    assert isinstance(ControlValues("arm"), DimosMsg)
    assert ControlValues.msg_name == "control_msgs.ControlValues"


def test_resolve_msg_type_returns_the_wrapper() -> None:
    """Name resolution prefers the wrapper over the generated type."""
    assert resolve_msg_type("control_msgs.ControlValues") is ControlValues


def test_transports_accept_the_type() -> None:
    """Both transports the control streams will use carry the type (no I/O)."""
    lcm = LCMTransport("/control_command", ControlValues)
    assert lcm.topic.lcm_type is ControlValues

    spec = ZenohTransport.spec("/control_state", ControlValues)
    assert spec.cls is ZenohTransport
    assert ControlValues in spec.args
