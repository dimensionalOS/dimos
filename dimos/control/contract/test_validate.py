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

"""What a command has to look like to be sent to a robot."""

from __future__ import annotations

import math

import pytest

from dimos.control.contract.limits import Limits
from dimos.control.contract.validate import CommandBatch, Rejected, validate_command

#: An arm told positions, one joint also a speed, and one that spins freely.
ARM = {
    "arm/joint1/position": Limits(-3.14, 3.14),
    "arm/joint2/position": Limits(-3.14, 3.14),
    "arm/joint1/velocity": Limits(-1.0, 1.0),
    "arm/spinner/position": Limits(),
}
#: A base whose speed limits clamp; it cannot move sideways.
CHASSIS = {
    "chassis/base/vx": Limits(-1.5, 1.5, clamp=True),
    "chassis/base/vy": Limits(0.0, 0.0, clamp=True),
    "chassis/base/wz": Limits(-2.0, 2.0, clamp=True),
}


def test_values_within_their_limits_are_sent_as_they_are() -> None:
    values = {"arm/joint1/position": 0.5, "arm/joint2/position": -0.5, "arm/joint1/velocity": 1.0}
    assert validate_command(values, ARM) == CommandBatch(values=values, clamped=())


def test_an_empty_command_is_accepted_and_sends_nothing() -> None:
    assert validate_command({}, ARM) == CommandBatch(values={}, clamped=())


def test_a_sparse_command_is_accepted() -> None:
    """Telling one joint of several is sparse, not incomplete."""
    result = validate_command({"arm/joint2/position": 0.1}, ARM)
    assert result == CommandBatch(values={"arm/joint2/position": 0.1}, clamped=())


def test_a_key_the_robot_does_not_accept_is_refused() -> None:
    for key in ("arm/joint1/effort", "arm/joint9/position", "g1/joint1/position"):
        result = validate_command({key: 0.0}, ARM)
        assert isinstance(result, Rejected)
        assert result.reason == "unknown_key"
        assert key in result.detail


@pytest.mark.parametrize("value", [99.0, -99.0])
def test_a_value_past_a_limit_is_refused(value: float) -> None:
    result = validate_command({"arm/joint1/position": value}, ARM)
    assert isinstance(result, Rejected)
    assert result.reason == "limit"
    assert "arm/joint1/position" in result.detail and str(value) in result.detail


def test_an_unbounded_limit_takes_anything_finite() -> None:
    for value in (1e9, -1e9):
        result = validate_command({"arm/spinner/position": value}, ARM)
        assert result == CommandBatch(values={"arm/spinner/position": value}, clamped=())


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_a_non_finite_value_is_refused(value: float) -> None:
    result = validate_command({"arm/spinner/position": value}, ARM)
    assert isinstance(result, Rejected)
    assert result.reason == "non-finite"


def test_a_clamping_limit_pulls_the_value_back_and_says_so() -> None:
    """A stick pushed too far drives the base at its top speed."""
    result = validate_command(
        {"chassis/base/vx": 9.0, "chassis/base/wz": -9.0, "chassis/base/vy": 0.0}, CHASSIS
    )
    assert result == CommandBatch(
        values={"chassis/base/vx": 1.5, "chassis/base/wz": -2.0, "chassis/base/vy": 0.0},
        clamped=("chassis/base/vx", "chassis/base/wz"),
    )


def test_a_zero_range_clamps_to_a_standstill() -> None:
    result = validate_command({"chassis/base/vy": 0.5}, CHASSIS)
    assert result == CommandBatch(values={"chassis/base/vy": 0.0}, clamped=("chassis/base/vy",))


def test_one_refused_value_refuses_the_whole_command() -> None:
    """All or nothing: one bad key does not let the good ones through."""
    result = validate_command({"arm/joint1/position": 0.1, "arm/joint2/position": 99.0}, ARM)
    assert isinstance(result, Rejected)
    result = validate_command({"arm/joint1/position": 0.1, "arm/joint1/effort": 1.0}, ARM)
    assert isinstance(result, Rejected)
