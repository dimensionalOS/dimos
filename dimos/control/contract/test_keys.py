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

"""Key spelling: three segments, and what each one means."""

from __future__ import annotations

import pickle

import pytest

from dimos.control.contract.keys import (
    POSITION,
    Unit,
    interface_of,
    is_valid_key,
    is_valid_segment,
    joint_of,
    make_key,
    source_of,
    split_key,
)


@pytest.mark.parametrize("segment", ["arm", "joint1", "left_hip_pitch", "A", "_", "9", "a_1_B"])
def test_valid_segments(segment: str) -> None:
    """Letters, digits and underscore, in any arrangement."""
    assert is_valid_segment(segment)


@pytest.mark.parametrize("segment", ["", "a-b", "a b", "a/b", "a.b", "ä", "a\n", "a:b"])
def test_invalid_segments(segment: str) -> None:
    """Anything else, including the separator and the empty string."""
    assert not is_valid_segment(segment)


def test_make_and_split_round_trip() -> None:
    """The three segments come back exactly as they went in."""
    key = make_key("arm", "joint1", POSITION)

    assert key == "arm/joint1/position"
    assert split_key(key) == ("arm", "joint1", "position")


def test_accessors() -> None:
    """Each accessor picks out its own segment; joint_of keeps the first two."""
    key = "g1/left_hip_pitch/kp"

    assert source_of(key) == "g1"
    assert joint_of(key) == "g1/left_hip_pitch"
    assert interface_of(key) == "kp"


@pytest.mark.parametrize(
    ("source", "resource", "interface", "offender"),
    [
        ("ar m", "joint1", "position", "ar m"),
        ("arm", "joint-1", "position", "joint-1"),
        ("arm", "joint1", "", ""),
        ("arm", "joint1", "pos/ition", "pos/ition"),
    ],
)
def test_make_key_names_the_offending_segment(
    source: str, resource: str, interface: str, offender: str
) -> None:
    """The error says which segment was wrong, not just that one was."""
    with pytest.raises(ValueError, match="invalid") as excinfo:
        make_key(source, resource, interface)

    assert repr(offender) in str(excinfo.value)


@pytest.mark.parametrize("key", ["arm/joint1", "arm/joint1/position/extra", "arm//position", ""])
def test_split_key_rejects_wrong_shape(key: str) -> None:
    """Two segments, four segments and an empty one are all malformed."""
    with pytest.raises(ValueError):
        split_key(key)


def test_split_key_error_quotes_the_key() -> None:
    """A malformed key appears in its own error, so a log line is actionable."""
    with pytest.raises(ValueError, match="arm/joint1"):
        split_key("arm/joint1")


@pytest.mark.parametrize(
    ("key", "valid"),
    [
        ("arm/joint1/position", True),
        ("go2/base/vx", True),
        ("arm/joint1", False),
        ("arm/joint1/position/x", False),
        ("arm/join t1/position", False),
    ],
)
def test_is_valid_key(key: str, valid: bool) -> None:
    """is_valid_key agrees with split_key on every shape."""
    assert is_valid_key(key) is valid


def test_accessors_reject_malformed_keys() -> None:
    """An accessor never quietly returns a segment of a broken key."""
    for accessor in (source_of, joint_of, interface_of):
        with pytest.raises(ValueError):
            accessor("arm/joint1")


def test_units_pickle_by_identity() -> None:
    """Units cross the RPC boundary and come back as the same member."""
    assert pickle.loads(pickle.dumps(Unit.RAD_PER_S)) is Unit.RAD_PER_S


def test_unit_values_are_stable_strings() -> None:
    """The wire spelling of a unit is part of the contract, not an accident."""
    assert Unit.RAD.value == "rad"
    assert Unit.M_PER_S2.value == "m/s^2"
    assert Unit.NORMALIZED.value == "normalized"
