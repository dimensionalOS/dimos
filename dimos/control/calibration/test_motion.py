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

"""Ramps, paths and the per-tick checks a calibration moves an arm with."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.control.calibration.gravity_poses import ArmModel, generate_poses
from dimos.control.calibration.motion import (
    MAX_SPEED_DEG_S,
    SegmentGuard,
    check_step,
    joint_limits,
    limit_error,
    limit_margin_rad,
    min_jerk,
    plan_path,
    ramp,
    ramp_duration_s,
    ramp_lag_limits_deg,
    speed_rad_s,
    tracking_error,
)
from dimos.control.calibration.spec import ArmCalibrationSpec

PERIOD_S = 0.02


@pytest.fixture(scope="module")
def arm(left_spec: ArmCalibrationSpec) -> ArmModel:
    return ArmModel(left_spec)


def test_min_jerk_starts_and_ends_at_rest():
    assert min_jerk(0.0) == 0.0 and min_jerk(1.0) == 1.0 and min_jerk(0.5) == 0.5
    assert min_jerk(-1.0) == 0.0 and min_jerk(2.0) == 1.0
    slope = (min_jerk(1e-4) - min_jerk(0.0)) / 1e-4
    assert slope < 1e-6
    assert (min_jerk(1.0) - min_jerk(1.0 - 1e-4)) / 1e-4 < 1e-6


def test_a_ramp_ends_on_its_target_and_no_joint_beats_the_speed_cap():
    start = np.radians([0.0, 60.0, 60.0])
    end = np.radians([30.0, 20.0, 60.0])
    speed = math.radians(10.0)
    targets = list(ramp(start, end, speed, PERIOD_S))
    assert np.allclose(targets[-1], end)
    assert len(targets) == math.ceil(ramp_duration_s(start, end, speed) / PERIOD_S)
    # 40 deg at a 10 deg/s peak: 1.875 x 4 s.
    assert ramp_duration_s(start, end, speed) == pytest.approx(7.5)
    path = np.vstack([start, *targets])
    peak = np.max(np.abs(np.diff(path, axis=0)), axis=0) / PERIOD_S
    assert np.degrees(peak).max() <= 10.0 + 1e-6
    assert np.degrees(peak).max() > 9.9
    # Every joint moves along the same line: the unmoved one stays put.
    assert np.allclose(path[:, 2], start[2])


def test_a_ramp_to_where_the_arm_is_is_one_tick():
    q = np.radians([10.0, 20.0])
    assert [list(t) for t in ramp(q, q, 1.0, PERIOD_S)] == [list(q)]


@pytest.mark.parametrize("speed", [0.0, -5.0, MAX_SPEED_DEG_S + 1.0, float("nan")])
def test_speed_is_capped_at_the_bench_limit(speed):
    with pytest.raises(ValueError, match="speed"):
        speed_rad_s(speed)


def test_speed_up_to_the_cap_is_taken():
    assert speed_rad_s(MAX_SPEED_DEG_S) == pytest.approx(math.radians(30.0))


@pytest.mark.parametrize("delta", [0.0, 5.5, -6.0, float("inf")])
def test_steps_are_small_and_finite(delta):
    with pytest.raises(ValueError):
        check_step(delta)


def test_tracking_error_names_the_lagging_joints():
    names = ["a", "b", "c"]
    commanded = np.radians([0.0, 10.0, 20.0])
    assert tracking_error(names, commanded, commanded + np.radians([9.0, -9.0, 0.0])) is None
    lag = tracking_error(names, commanded, commanded + np.radians([0.0, -12.5, 0.0]))
    assert lag is not None
    assert "b -12.5 deg" in lag and "a " not in lag and "10 deg" in lag


def test_a_ramping_joint_may_lag_5_deg_more_than_a_pd_follower_does():
    # A wrist at kp 10, kd 1.5 (0.15 s) and an elbow at kp 80, kd 5, at 20 deg/s,
    # the wrist starting on a 2 deg sag.
    limits = ramp_lag_limits_deg(
        np.array([2.0, 0.0]), np.array([20.0, -20.0]), np.array([0.15, 0.0625])
    )
    assert limits == pytest.approx([2.0 + 3.0 + 5.0, 1.25 + 5.0])
    # At rest 5 deg over the starting sag; never more than the hold limit.
    assert ramp_lag_limits_deg(np.zeros(2), np.zeros(2), np.ones(2)) == pytest.approx([5.0, 5.0])
    assert ramp_lag_limits_deg(np.array([9.0]), np.array([30.0]), np.array([1.0]))[0] == 10.0
    names = ["wrist", "elbow"]
    lag = tracking_error(names, np.zeros(2), np.radians([9.0, -6.5]), limits)
    assert lag is not None and "more than 6.2 deg from its target (elbow -6.5 deg)" in lag
    both = tracking_error(names, np.zeros(2), np.radians([10.5, -6.5]), limits)
    assert both is not None and "wrist +10.5 deg (limit 10.0), elbow -6.5 deg (limit 6.2)" in both


def test_limits_say_which_joint_is_past_or_near_one(arm: ArmModel):
    lower, upper = joint_limits(arm.spec)
    home = np.asarray(arm.home)
    assert limit_error(arm.joint_names, home, lower, upper) is None
    q = home.copy()
    q[1] = upper[1] + 0.01
    assert limit_error(arm.joint_names, q, lower, upper).startswith(f"{arm.joint_names[1]} at ")
    assert limit_margin_rad(q, lower, upper) == pytest.approx(-0.01)
    near = limit_error(arm.joint_names, home, lower, upper, math.radians(61.0))
    assert near is not None and arm.joint_names[1] in near


def test_a_path_goes_straight_when_it_can(arm: ArmModel):
    pose = np.asarray(generate_poses(arm, count=3).identification[0].joints_rad)
    path = plan_path(arm, arm.home, pose)
    assert path is not None and len(path) == 1 and np.allclose(path[0], pose)


def test_a_path_that_would_dip_goes_through_home(arm: ArmModel, mocker):
    a = np.asarray(arm.home) + np.radians([0.0, 10.0, 0.0, 0.0, 0.0, 0.0])
    b = np.asarray(arm.home) + np.radians([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])

    def clears(x, y):
        # The straight a -> b ramp dips; each leg through home clears the table.
        return not (np.allclose(x, a) and np.allclose(y, b))

    mocker.patch.object(arm, "transition_ok", side_effect=clears)
    path = plan_path(arm, a, b)
    assert path is not None and [list(p) for p in path] == [list(arm.home), list(b)]


def test_no_path_when_even_home_does_not_clear(arm: ArmModel, mocker):
    mocker.patch.object(arm, "transition_ok", return_value=False)
    assert plan_path(arm, arm.home, arm.home) is None


def test_the_segment_guard_stops_a_target_on_the_table(arm: ArmModel):
    home = np.asarray(arm.home)
    guard = SegmentGuard(arm, home, home)
    assert guard.check_target(home) is None and guard.check_measured(home) is None
    # Shoulder and elbow folded down to put the gripper into the table.
    low = home.copy()
    low[1] = math.radians(150.0)
    low[2] = math.radians(10.0)
    refused = guard.check_target(low)
    assert refused is not None and "too close to the table" in refused


def test_the_segment_guard_keeps_the_arm_on_its_side_of_the_midline(arm: ArmModel):
    home = np.asarray(arm.home)
    guard = SegmentGuard(arm, home, home)
    across = home.copy()
    across[0] = math.radians(-90.0)  # the left arm's base yaw toward the right arm
    refused = guard.check_target(across)
    assert refused is not None and "midline to dual_openyam/right" in refused
