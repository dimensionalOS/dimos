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

"""Tests for the shared maths helpers.

The two position helpers are tested against each other: work out where a robot
ends up, then work backwards from that to how fast it was going, and the answer
should be what you started with. Where it cannot be exactly that, the test says
by how much and why, rather than hiding it behind a loose tolerance.

``await_state`` is given a fake clock, so a test of a one-second timeout takes
microseconds and nothing here ever waits for real.
"""

from __future__ import annotations

import math
import random
import time

import pytest

from dimos.control.connection.helpers import (
    await_state,
    integrate_planar_twist,
    pd_torque,
    se2_body_twist,
    wrap_to_pi,
)


class FakeTime:
    """A clock that only moves when somebody sleeps on it."""

    def __init__(self) -> None:
        self.now = 0.0
        self.slept: list[float] = []

    def clock(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.slept.append(seconds)
        self.now += seconds


def test_wrap_keeps_the_positive_end_of_the_interval() -> None:
    # (-pi, pi], so both ends of the same heading land on +pi and a yaw
    # sitting behind the robot does not flip sign every sample.
    assert wrap_to_pi(math.pi) == math.pi
    assert wrap_to_pi(-math.pi) == math.pi
    assert wrap_to_pi(3 * math.pi) == math.pi


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (1.0, 1.0),
        (1.5 * math.pi, -0.5 * math.pi),
        (-1.5 * math.pi, 0.5 * math.pi),
        (2 * math.pi, 0.0),
        (100 * math.pi + 0.25, 0.25),
    ],
)
def test_wrap_folds_any_angle_into_the_interval(angle: float, expected: float) -> None:
    assert wrap_to_pi(angle) == pytest.approx(expected, abs=1e-12)
    assert -math.pi < wrap_to_pi(angle) <= math.pi


def test_a_still_base_does_not_move() -> None:
    assert integrate_planar_twist(1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 0.1) == (1.0, 2.0, 0.5)


def test_forward_is_body_forward_not_world_x() -> None:
    # Facing +x, vx goes to +x.
    x, y, _ = integrate_planar_twist(0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.5)
    assert (x, y) == pytest.approx((1.0, 0.0), abs=1e-12)
    # Facing +y, the same vx goes to +y. This is the bug a world-frame
    # integrator would have.
    x, y, _ = integrate_planar_twist(0.0, 0.0, math.pi / 2, 2.0, 0.0, 0.0, 0.5)
    assert (x, y) == pytest.approx((0.0, 1.0), abs=1e-12)


def test_strafe_is_to_the_left_of_the_heading() -> None:
    x, y, _ = integrate_planar_twist(0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.5)
    assert (x, y) == pytest.approx((0.0, 1.0), abs=1e-12)


def test_yaw_wraps_past_pi_by_default() -> None:
    _, _, yaw = integrate_planar_twist(0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 0.5)
    # 3.0 + 0.5 = 3.5, which is past pi.
    assert yaw == pytest.approx(3.5 - 2 * math.pi, abs=1e-12)
    assert -math.pi < yaw <= math.pi


def test_yaw_can_be_left_unwrapped() -> None:
    # R1Pro counts turns; the mock base wraps. Both are declared in the
    # description's yaw_convention, and this flag is the difference.
    _, _, yaw = integrate_planar_twist(0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 0.5, wrap=False)
    assert yaw == pytest.approx(3.5, abs=1e-12)


def test_unwrapped_yaw_accumulates_over_many_turns() -> None:
    yaw = 0.0
    for _ in range(100):
        _, _, yaw = integrate_planar_twist(0.0, 0.0, yaw, 0.0, 0.0, 1.0, 0.1, wrap=False)
    assert yaw == pytest.approx(10.0, abs=1e-9)


@pytest.mark.parametrize("dt", [0.0, -0.001, -5.0])
def test_a_non_positive_dt_leaves_the_pose_alone(dt: float) -> None:
    # Timestamps do go backwards -- a replayed bag, a resynchronized clock --
    # and the pose should stand still rather than integrate in reverse.
    pose = (1.0, 2.0, 0.5)
    assert integrate_planar_twist(*pose, 9.0, 9.0, 9.0, dt) == pose


def test_differentiating_a_pure_rotation() -> None:
    assert se2_body_twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.2), 0.5) == pytest.approx(
        (0.0, 0.0, 0.4), abs=1e-12
    )


def test_differentiating_a_translation_under_a_heading() -> None:
    # Facing +y and having moved to +y: that is forward, not strafe.
    vx, vy, wz = se2_body_twist((0.0, 0.0, math.pi / 2), (0.0, 1.0, math.pi / 2), 0.5)
    assert (vx, vy, wz) == pytest.approx((2.0, 0.0, 0.0), abs=1e-12)


def test_a_yaw_crossing_pi_reports_the_small_rate_it_actually_turned_at() -> None:
    # Naively subtracting would report -2pi/dt: a base spinning at 6000 deg/s.
    _, _, wz = se2_body_twist((0.0, 0.0, math.pi - 0.05), (0.0, 0.0, -math.pi + 0.05), 0.1)
    assert wz == pytest.approx(1.0, abs=1e-12)


def test_a_yaw_that_counts_turns_keeps_its_whole_turn() -> None:
    # R1Pro never wraps. Between two samples more than half a turn apart,
    # wrapping would throw away the 2 pi: four radians in a second comes back
    # as -2.28 rad/s, a base reported as turning backwards.
    prev = (0.0, 0.0, 0.0)
    curr = integrate_planar_twist(*prev, 1.0, 0.0, 4.0, 1.0, wrap=False)
    assert se2_body_twist(prev, curr, 1.0, wrap=False)[2] == pytest.approx(4.0)
    assert se2_body_twist(prev, curr, 1.0, wrap=True)[2] == pytest.approx(4.0 - 2 * math.pi)


def test_the_wrap_flag_tilts_the_heading_too_not_just_the_rate() -> None:
    # The mid-heading comes from the same difference, so the wrong flag rotates
    # the body frame as well: vx and vy land somewhere else entirely.
    prev = (0.0, 0.0, 0.0)
    curr = integrate_planar_twist(*prev, 1.0, 0.0, 4.0, 1.0, wrap=False)
    right = se2_body_twist(prev, curr, 1.0, wrap=False)
    wrong = se2_body_twist(prev, curr, 1.0, wrap=True)
    assert right[:2] != pytest.approx(wrong[:2], abs=1e-3)


def test_a_wrapped_yaw_still_needs_wrapping() -> None:
    # The other half of the contract: on a wrapped yaw the samples are only
    # known modulo 2 pi, so the shortest rotation is the best anyone can do and
    # wrap=False would report the 2 pi/dt spike instead.
    prev = (0.0, 0.0, math.pi - 0.05)
    curr = (0.0, 0.0, -math.pi + 0.05)
    assert se2_body_twist(prev, curr, 0.1, wrap=True)[2] == pytest.approx(1.0, abs=1e-12)
    assert se2_body_twist(prev, curr, 0.1, wrap=False)[2] == pytest.approx(
        (0.1 - 2 * math.pi) / 0.1
    )


def test_a_big_unwrapped_turn_round_trips_with_matching_flags() -> None:
    # Several turns in one step: only meaningful unwrapped, and exact there
    # because the pair agree on the convention.
    rng = random.Random(99)
    for _ in range(200):
        pose = (rng.uniform(-9, 9), rng.uniform(-9, 9), rng.uniform(-20.0, 20.0))
        wz, dt = rng.uniform(-30, 30), rng.uniform(0.5, 2.0)
        moved = integrate_planar_twist(*pose, 0.0, 0.0, wz, dt, wrap=False)
        assert se2_body_twist(pose, moved, dt, wrap=False)[2] == pytest.approx(wz, abs=1e-9)


@pytest.mark.parametrize("dt", [0.0, -0.001])
def test_differentiating_over_no_time_raises(dt: float) -> None:
    with pytest.raises(ValueError, match="dt must be positive"):
        se2_body_twist((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), dt)


def test_integrate_then_differentiate_round_trips_a_straight_run() -> None:
    # With no rotation the mid-heading is the heading, so the pair are exact
    # inverses.
    rng = random.Random(20260923)
    for _ in range(500):
        pose = (rng.uniform(-9, 9), rng.uniform(-9, 9), rng.uniform(-math.pi, math.pi))
        vx, vy = rng.uniform(-2, 2), rng.uniform(-2, 2)
        dt = rng.uniform(0.001, 0.2)
        moved = integrate_planar_twist(*pose, vx, vy, 0.0, dt, wrap=False)
        assert se2_body_twist(pose, moved, dt, wrap=False) == pytest.approx((vx, vy, 0.0), abs=1e-9)


def test_a_turning_round_trip_differs_by_exactly_the_midpoint_correction() -> None:
    # Integration is Euler about the starting heading; differentiation uses the
    # mid-heading. Over a step that also turned, the recovered twist is the
    # original rotated by -wz*dt/2 -- which is the correction the midpoint rule
    # is making, not an error in either function.
    rng = random.Random(4243)
    for _ in range(500):
        pose = (rng.uniform(-9, 9), rng.uniform(-9, 9), rng.uniform(-math.pi, math.pi))
        vx, vy = rng.uniform(-2, 2), rng.uniform(-2, 2)
        wz, dt = rng.uniform(-2, 2), rng.uniform(0.001, 0.2)
        moved = integrate_planar_twist(*pose, vx, vy, wz, dt, wrap=False)
        half = wz * dt / 2.0
        expected = (
            vx * math.cos(half) + vy * math.sin(half),
            -vx * math.sin(half) + vy * math.cos(half),
            wz,
        )
        assert se2_body_twist(pose, moved, dt, wrap=False) == pytest.approx(expected, abs=1e-9)


def test_the_round_trip_is_tight_at_a_real_state_rate() -> None:
    # A chassis runs at 50 Hz. At 2 rad/s that correction is a milliradian of
    # rotation, so the recovered twist is the commanded one to 4 decimals.
    pose = (0.0, 0.0, 0.3)
    moved = integrate_planar_twist(*pose, 1.0, 0.0, 2.0, 0.02, wrap=False)
    assert se2_body_twist(pose, moved, 0.02, wrap=False) == pytest.approx((1.0, 0.0, 2.0), abs=2e-2)


def test_pd_torque_sums_its_three_terms() -> None:
    # 10 * (1.0 - 0.5) + 3 * (2.0 - 0.5) + 1.0
    assert pd_torque(1.0, 2.0, 0.5, 0.5, 10.0, 3.0, 1.0) == pytest.approx(10.5)


def test_the_target_velocity_term_is_the_one_mujoco_is_missing() -> None:
    # Today's emulation computes kd * (0 - dq). Asking for a steady 2.0 rad/s
    # should add kd * dq_target on top of that, and this is the difference.
    tracking = pd_torque(1.0, 2.0, 0.5, 0.5, 10.0, 3.0, 0.0)
    damping_only = pd_torque(1.0, 0.0, 0.5, 0.5, 10.0, 3.0, 0.0)
    assert damping_only == pytest.approx(3.5)
    assert tracking == pytest.approx(9.5)
    assert tracking - damping_only == pytest.approx(3.0 * 2.0)


def test_a_joint_already_on_target_produces_only_the_feedforward() -> None:
    assert pd_torque(1.0, 2.0, 1.0, 2.0, 50.0, 5.0, 0.75) == pytest.approx(0.75)


def test_zero_gains_leave_the_feedforward_alone() -> None:
    assert pd_torque(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, -4.0) == pytest.approx(-4.0)


def test_a_condition_already_true_costs_nothing() -> None:
    fake = FakeTime()
    assert await_state(lambda: True, 1.0, clock=fake.clock, sleep=fake.sleep) is True
    assert fake.slept == []


def test_a_condition_that_comes_true_on_the_third_poll() -> None:
    fake = FakeTime()
    calls = {"n": 0}

    def ready() -> bool:
        calls["n"] += 1
        return calls["n"] >= 3

    assert await_state(ready, 1.0, poll_s=0.25, clock=fake.clock, sleep=fake.sleep) is True
    assert calls["n"] == 3
    assert fake.slept == [0.25, 0.25]


def test_a_condition_that_never_comes_true_times_out() -> None:
    fake = FakeTime()
    started = time.perf_counter()
    assert await_state(lambda: False, 1.0, poll_s=0.25, clock=fake.clock, sleep=fake.sleep) is False
    # A simulated second, spent entirely on the injected clock: the real
    # time.sleep default is never reached.
    assert fake.slept == [0.25, 0.25, 0.25, 0.25]
    assert fake.now == pytest.approx(1.0)
    assert time.perf_counter() - started < 0.1


def test_the_last_poll_is_shortened_to_land_on_the_deadline() -> None:
    fake = FakeTime()
    assert await_state(lambda: False, 1.0, poll_s=0.3, clock=fake.clock, sleep=fake.sleep) is False
    assert fake.slept == pytest.approx([0.3, 0.3, 0.3, 0.1])
    # A bounded hook must not overrun the bound it was given.
    assert fake.now == pytest.approx(1.0)


@pytest.mark.parametrize("timeout_s", [0.0, -1.0])
def test_a_non_positive_timeout_still_gets_one_look(timeout_s: float) -> None:
    fake = FakeTime()
    assert await_state(lambda: True, timeout_s, clock=fake.clock, sleep=fake.sleep) is True
    assert await_state(lambda: False, timeout_s, clock=fake.clock, sleep=fake.sleep) is False
    assert fake.slept == []


@pytest.mark.parametrize("poll_s", [0.0, -0.01])
def test_a_non_positive_poll_would_spin_and_raises(poll_s: float) -> None:
    with pytest.raises(ValueError, match="poll_s must be positive"):
        await_state(lambda: False, 1.0, poll_s=poll_s)
