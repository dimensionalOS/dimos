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


"""Key spelling: three parts, and what each one means."""

from __future__ import annotations

import pickle

import pytest

from dimos.control.contract.keys import POSITION, Key, Unit, is_valid_key, is_valid_segment


@pytest.mark.parametrize("segment", ["arm", "joint1", "left_hip_pitch", "A", "_", "9", "a_1_B"])
def test_valid_segments(segment: str) -> None:
    """Letters, digits and underscore, in any arrangement."""
    assert is_valid_segment(segment)


@pytest.mark.parametrize("segment", ["", "a-b", "a b", "a/b", "a.b", "\u00e4", "a\n", "a:b"])
def test_invalid_segments(segment: str) -> None:
    """Anything else, including the separator and the empty string."""
    assert not is_valid_segment(segment)


def test_build_from_parts() -> None:
    """Key.of joins three parts and checks each one."""
    key = Key.of("arm", "joint1", POSITION)

    assert key == "arm/joint1/position"


def test_parts_read_back() -> None:
    """Each part is available by name; joint is the first two."""
    key = Key("g1/left_hip_pitch/kp")

    assert key.source == "g1"
    assert key.resource == "left_hip_pitch"
    assert key.interface == "kp"
    assert key.joint == "g1/left_hip_pitch"


def test_a_key_is_a_string() -> None:
    """It goes on the wire and works as a dict key with no conversion."""
    key = Key.of("arm", "joint1", POSITION)

    assert isinstance(key, str)
    assert {key: 1.0}["arm/joint1/position"] == 1.0
    assert key.startswith("arm/")


@pytest.mark.parametrize(
    ("source", "resource", "interface", "offender"),
    [
        ("ar m", "joint1", "position", "ar m"),
        ("arm", "joint-1", "position", "joint-1"),
        ("arm", "joint1", "", ""),
        ("arm", "joint1", "pos/ition", "pos/ition"),
    ],
)
def test_of_names_the_offending_part(
    source: str, resource: str, interface: str, offender: str
) -> None:
    """The error says which part was wrong, not just that one was."""
    with pytest.raises(ValueError, match="invalid") as excinfo:
        Key.of(source, resource, interface)

    assert repr(offender) in str(excinfo.value)


@pytest.mark.parametrize("value", ["arm/joint1", "arm/joint1/position/extra", "arm//position", ""])
def test_wrong_shape_is_rejected(value: str) -> None:
    """Two parts, four parts and an empty one are all malformed."""
    with pytest.raises(ValueError):
        Key(value)


def test_error_quotes_the_key() -> None:
    """A malformed name appears in its own error, so a log line is actionable."""
    with pytest.raises(ValueError, match="arm/joint1"):
        Key("arm/joint1")


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
    """is_valid_key agrees with the type on every shape."""
    assert is_valid_key(key) is valid


def test_keys_pickle_with_their_parts() -> None:
    """Descriptions cross a process boundary, so the parts must survive."""
    restored = pickle.loads(pickle.dumps(Key.of("arm", "joint1", POSITION)))

    assert restored == "arm/joint1/position"
    assert restored.source == "arm"
    assert restored.interface == "position"


def test_units_pickle_by_identity() -> None:
    """Units cross the same boundary and come back as the same member."""
    assert pickle.loads(pickle.dumps(Unit.RAD_PER_S)) is Unit.RAD_PER_S


def test_unit_values_are_stable_strings() -> None:
    """The wire spelling of a unit is part of the contract, not an accident."""
    assert Unit.RAD.value == "rad"
    assert Unit.M_PER_S2.value == "m/s^2"
    assert Unit.NORMALIZED.value == "normalized"
