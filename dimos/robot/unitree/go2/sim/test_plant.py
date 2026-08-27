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

"""The board's PD law, the torque envelope, and the current-loop lag."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.plant import (
    TORQUE_ENVELOPES,
    TORQUE_LIMITS,
    TorqueEnvelope,
    actuator_step,
    pd_torque,
)


def test_dq_des_is_part_of_the_pd_law_not_an_assumed_zero():
    """Unitree's built-in controller commands up to 20.7 rad/s with kd 1.8 —
    tens of Nm. Dropping dq_des silently halves the damping term on exactly
    the recordings that carry the jumps."""
    q = np.zeros(12)
    dq = np.zeros(12)
    kp = np.zeros(12)
    kd = np.full(12, 1.8)
    with_dq_des = pd_torque(q, np.full(12, 20.7), kp, kd, np.zeros(12), q, dq)
    without = pd_torque(q, np.zeros(12), kp, kd, np.zeros(12), q, dq)
    assert with_dq_des[0] == pytest.approx(1.8 * 20.7)  # ~37 Nm of difference
    assert without[0] == 0.0


def test_the_pd_law_is_the_boards_kp_kd_tau_ff_form():
    tau = pd_torque(
        q_des=np.full(12, 0.5),
        dq_des=np.full(12, 1.0),
        kp=np.full(12, 40.0),
        kd=np.full(12, 2.0),
        tau_ff=np.full(12, 3.0),
        q=np.full(12, 0.4),
        dq=np.full(12, -1.0),
    )
    assert tau[0] == pytest.approx(40.0 * 0.1 + 2.0 * 2.0 + 3.0)


def test_zero_tau_is_the_ideal_motor_that_delivers_on_the_same_step():
    req = np.full(12, 10.0)
    assert np.array_equal(actuator_step(np.zeros(12), req, 0.002, 0.0), req)


def test_the_lag_reaches_1_minus_1_over_e_after_tau_seconds():
    dt, tau = 0.002, 0.01
    applied = np.zeros(12)
    req = np.ones(12)
    for _ in range(round(tau / dt)):
        applied = actuator_step(applied, req, dt, tau)
    assert applied[0] == pytest.approx(1 - 1 / np.e, abs=0.05)


def test_the_envelope_lands_before_the_lag_so_a_clip_saturates_the_current_loop():
    env = TORQUE_ENVELOPES["central"]
    dq = np.full(12, 12.0)  # above the 10 rad/s ceiling knee
    req = np.full(12, 20.0)
    stepped = actuator_step(np.zeros(12), req, 0.002, 0.0, dq=dq, envelope=env)
    assert stepped[0] == pytest.approx(5.5)  # the ceiling, not the request


def test_an_envelope_without_joint_speeds_is_an_error():
    with pytest.raises(ValueError, match="needs the joint speeds"):
        actuator_step(np.zeros(12), np.ones(12), 0.002, 0.0, envelope=TORQUE_ENVELOPES["central"])


def test_braking_a_limb_keeps_more_authority_than_driving_it():
    """Back-EMF subtracts from drive voltage only when torque acts WITH the
    rotation; the measured brake/drive ratio is 1.10-1.33."""
    env = TORQUE_ENVELOPES["central-signed"]
    dq = np.full(12, 6.0)
    driving = env.deliverable(np.full(12, 3.0), dq)  # torque with the motion
    braking = env.deliverable(np.full(12, -3.0), dq)  # torque against it
    assert abs(braking[0]) > abs(driving[0])


def test_the_named_variants_bracket_the_droop():
    dq = np.full(1, 5.0)
    req = np.full(1, 4.0)
    got = {n: float(TORQUE_ENVELOPES[n].deliverable(req, dq)[0]) for n in TORQUE_ENVELOPES}
    assert got["aggressive"] < got["central"] < got["conservative"]


def test_a_rising_gain_curve_is_rejected():
    with pytest.raises(ValueError, match="non-increasing"):
        TorqueEnvelope("bad", (0.0, 5.0), (0.5, 0.9), (0.0, 5.0), (10.0, 10.0))


def test_torque_limits_are_per_joint_hip_thigh_calf():
    assert TORQUE_LIMITS.shape == (12,)
    assert list(TORQUE_LIMITS[:3]) == [23.0, 23.0, 35.0]
