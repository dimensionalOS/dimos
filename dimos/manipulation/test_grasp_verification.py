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

"""Unit tests for gripper settle polling and grasp classification."""

from __future__ import annotations

from pydantic import ValidationError
import pytest

from dimos.manipulation.grasp_verification import (
    GraspVerificationConfig,
    GripperSettle,
    await_gripper_settle,
    grasp_failure,
    open_failure,
)


class FakeClock:
    """Monotonic clock advanced only by the sleeps the helper performs."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


def reader(values):
    """Yield each value once, then repeat the last one forever."""
    remaining = list(values)
    last = remaining[-1]

    def read():
        return remaining.pop(0) if remaining else last

    return read


def settle(values, target=0.0, clock=None, **overrides):
    config = GraspVerificationConfig(**overrides)
    clock = clock or FakeClock()
    return await_gripper_settle(
        reader(values), target, config, sleep=clock.sleep, clock=clock
    ), config


def test_close_onto_object_settles_in_the_held_band():
    values = [1.0, 0.90, 0.80, 0.74, 0.73, 0.73, 0.73]
    result, config = settle(values)
    assert result.settled
    assert result.moved
    assert result.position == pytest.approx(0.73)
    assert grasp_failure(result, config) is None


def test_free_air_close_settles_at_closed_and_reads_empty():
    values = [1.0, 0.60, 0.20, 0.05, 0.03, 0.03, 0.03]
    result, config = settle(values)
    assert result.settled
    assert "nothing in the jaws" in (grasp_failure(result, config) or "")


def test_jaws_that_never_travel_are_not_a_grasp():
    result, config = settle([1.0] * 6, target=0.0)
    assert not result.moved
    assert "did not settle" in (grasp_failure(result, config) or "")


def test_settle_waits_for_travel_before_declaring_rest():
    """A poll that starts before the jaws react must not settle at the start."""
    values = [1.0, 1.0, 1.0, 1.0, 0.80, 0.74, 0.73, 0.73, 0.73]
    result, config = settle(values, target=0.0)
    assert result.settled
    assert result.position == pytest.approx(0.73)
    assert grasp_failure(result, config) is None


def test_open_command_settles_immediately_when_already_open():
    """Arrival at the target settles without travel, so re-opening is free."""
    clock = FakeClock()
    result, config = settle([1.0] * 4, target=1.0, clock=clock)
    assert result.settled
    assert not result.moved
    assert clock.now < config.timeout


def test_a_gripper_that_never_stops_moving_times_out():
    drifting = [1.0 - 0.001 * i for i in range(10_000)]
    clock = FakeClock()
    result, config = settle(drifting, clock=clock, settle_tolerance=0.0005)
    assert not result.settled
    assert clock.now >= config.timeout
    assert "did not settle" in (grasp_failure(result, config) or "")


def test_missing_readback_reports_unavailable():
    result, config = settle([None] * 200)
    assert not result.settled
    assert result.position is None
    assert grasp_failure(result, config) == "gripper readback unavailable"


def test_readback_that_appears_late_still_settles():
    values = [None, None, 0.80, 0.74, 0.73, 0.73, 0.73]
    result, config = settle(values)
    assert result.settled
    assert grasp_failure(result, config) is None


def test_timeout_is_honoured_even_when_readback_never_settles():
    clock = FakeClock()
    result, _ = settle([0.5, 0.9] * 5000, clock=clock, timeout=1.0, poll_interval=0.02)
    assert not result.settled
    assert clock.now == pytest.approx(1.0, abs=0.02)


def test_band_edges_are_exclusive_of_the_held_verdict():
    config = GraspVerificationConfig()
    assert "nothing in the jaws" in (
        grasp_failure(GripperSettle(True, config.held_low, True, 0.1), config) or ""
    )
    assert "never closed" in (
        grasp_failure(GripperSettle(True, config.held_high, True, 0.1), config) or ""
    )


def test_defaults_classify_the_readings_measured_on_the_sim():
    """Free air settles at 0.023/0.043; a held 6cm block stalls at 0.618."""
    config = GraspVerificationConfig()
    for empty in (0.0, 0.023, 0.043):
        assert grasp_failure(GripperSettle(True, empty, True, 0.5), config) is not None
    for held in (0.618, 0.73):
        assert grasp_failure(GripperSettle(True, held, True, 0.4), config) is None


def test_config_rejects_an_inverted_or_empty_band():
    with pytest.raises(ValidationError):
        GraspVerificationConfig(closed_position=1.0, open_position=0.0)
    with pytest.raises(ValidationError):
        GraspVerificationConfig(empty_epsilon=0.6, open_margin=0.6)
    with pytest.raises(ValidationError):
        GraspVerificationConfig(poll_interval=5.0, timeout=1.0)


def test_calibrated_epsilon_moves_the_empty_boundary():
    reading = GripperSettle(True, 0.03, True, 0.1)
    assert grasp_failure(reading, GraspVerificationConfig(empty_epsilon=0.02)) is None
    assert grasp_failure(reading, GraspVerificationConfig()) is not None


def test_a_healthy_reopen_counts_as_open():
    """A reopen from closed settles at 0.947 on the xArm7 sim, not at 1.0."""
    config = GraspVerificationConfig()
    assert open_failure(GripperSettle(True, 0.9468, True, 0.53), config) is None
    assert open_failure(GripperSettle(True, 0.9966, False, 0.11), config) is None


def test_an_open_that_stalls_on_the_held_object_fails():
    """Releasing onto a stuck object leaves the jaws at the object's width."""
    config = GraspVerificationConfig()
    failure = open_failure(GripperSettle(True, 0.618, True, 0.4), config)
    assert failure is not None
    assert "short of open" in failure


def test_open_tolerance_sits_between_the_measured_reopen_and_a_held_object():
    config = GraspVerificationConfig()
    assert open_failure(GripperSettle(True, 0.87, True, 0.5), config) is None
    assert open_failure(GripperSettle(True, 0.83, True, 0.5), config) is not None


def test_an_open_that_never_settles_fails():
    config = GraspVerificationConfig()
    failure = open_failure(GripperSettle(False, 0.5, True, 3.0), config)
    assert failure is not None
    assert "did not settle" in failure
