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

import math

import numpy as np
import pinocchio as pin
import pytest

from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration._toy_arm import toy_arm
from dimos.control.calibration.gravity_fit import (
    Compensation,
    DitherHold,
    GravityFit,
    Measurement,
    _solve,
    dither_friction_nm,
    fit_gravity,
    fit_stats,
    measure_dither,
    refusals,
    regressions,
    stuck_pairs,
    totals,
)
from dimos.control.calibration.gravity_poses import ArmModel, generate_poses
from dimos.control.calibration.step_analysis import RestNoise, StepEvent

KP, DITHER_DEG, FRICTION_NM, HOLD_S = 40.0, 1.0, 0.05, 1.0
NOISE = RestNoise(window_s=0.3, band_deg=0.01, speed_peak_deg_s=0.1)
TRUE_LINKS = {"fore": LinkInertial(mass_kg=0.7, com_m=(0.13, 0.0, 0.02))}


def true_model(arm: ArmModel) -> pin.Model:
    model = arm.model.copy()
    apply_inertial_overrides(model, TRUE_LINKS)
    return model


def hold(arm: ArmModel, joint: str, q_rest: np.ndarray, target_deg: float, sign: int) -> DitherHold:
    t = np.arange(round(HOLD_S * 100) + 1) / 100.0
    event = StepEvent(
        joint, sign * DITHER_DEG, KP, 1.0, 0.0, HOLD_S, target_deg - sign * DITHER_DEG, target_deg
    )
    return DitherHold(event, t, np.tile(q_rest, (t.size, 1)))


def dithers(
    arm: ArmModel, compensation: Compensation, poses: list[np.ndarray]
) -> list[Measurement]:
    """Dither pairs of a true arm whose PD rests where it balances the gravity left over and friction."""
    truth, compensated = true_model(arm), arm.model.copy()
    apply_inertial_overrides(compensated, compensation.links)

    def left_over(q: np.ndarray) -> np.ndarray:
        torque = pin.computeGeneralizedGravity(truth, truth.createData(), q)
        if compensation.gravity:
            torque = torque - pin.computeGeneralizedGravity(
                compensated, compensated.createData(), q
            )
        return np.asarray(torque)

    measurements = []
    for index, q in enumerate(poses):
        for joint in arm.measured_joints:
            row = arm.joint_names.index(joint)
            pair = []
            for sign in (1, -1):
                target = math.degrees(q[row]) + sign * DITHER_DEG
                rest = q.copy()
                for _ in range(50):
                    rest[row] = (
                        math.radians(target) - (left_over(rest)[row] + sign * FRICTION_NM) / KP
                    )
                pair.append(hold(arm, joint, rest, target, sign))
            measurements.append(
                measure_dither(
                    f"pose_{index}", arm.joint_names, pair[0], pair[1], NOISE, stuck_fraction=0.5
                )
            )
    return measurements


@pytest.fixture(scope="module")
def arm() -> ArmModel:
    return toy_arm()


@pytest.fixture(scope="module")
def poses(arm: ArmModel) -> list[np.ndarray]:
    pose_set = generate_poses(
        arm, count=8, margin_rad=math.radians(2 * DITHER_DEG), grid_points=400
    )
    assert len(pose_set.identification) == 8 and not pose_set.notes
    return [np.asarray(pose.joints_rad) for pose in pose_set.identification]


def test_a_dither_pair_cancels_friction(arm: ArmModel, poses: list[np.ndarray]) -> None:
    measurement = dithers(arm, Compensation(gravity=True), poses[:1])[0]
    q = arm.q(measurement.q_rad)
    truth = pin.computeGeneralizedGravity(true_model(arm), true_model(arm).createData(), q)
    row = arm.joint_names.index(measurement.joint)
    assert measurement.tau_pd_nm == pytest.approx(truth[row] - arm.gravity(q)[row], abs=1e-3)
    # The half difference also holds half the gravity change across the dither.
    assert measurement.friction_nm == pytest.approx(FRICTION_NM, abs=0.02)


@pytest.mark.parametrize(
    "compensation",
    [
        Compensation(gravity=False),
        Compensation(gravity=True),
        Compensation(True, {"upper": LinkInertial(1.2, (0.1, 0.0, 0.0))}),
    ],
)
def test_the_fit_recovers_the_true_gravity_and_is_offered(
    arm: ArmModel, poses: list[np.ndarray], compensation: Compensation
) -> None:
    fit = fit_gravity(arm, dithers(arm, compensation, poses), compensation)
    assert refusals(fit) == [] and regressions(fit) == []
    fitted = arm.model.copy()
    apply_inertial_overrides(fitted, fit.links)
    for q in poses:
        expected = pin.computeGeneralizedGravity(true_model(arm), true_model(arm).createData(), q)
        assert pin.computeGeneralizedGravity(fitted, fitted.createData(), q) == pytest.approx(
            expected, abs=1e-3
        )
    assert (
        max(fit.rms(fit.held_out_error).values())
        < 1e-3
        < 0.01
        < min(fit.rms(fit.urdf_error).values())
    )
    assert fit_stats(fit)["rms_nm"].keys() == {"shoulder", "elbow"}


def test_a_fit_no_better_than_the_running_model_lists_its_regressions(
    arm: ArmModel, poses: list[np.ndarray]
) -> None:
    running = Compensation(True, dict(TRUE_LINKS))
    measurements = dithers(arm, running, poses)
    noise = np.random.default_rng(0).normal(0.0, 0.02, len(measurements))
    noisy = [
        Measurement(m.pose, m.joint, m.kp, m.dither_deg, m.q_rad, m.tau_pd_nm + n, m.friction_nm)
        for m, n in zip(measurements, noise, strict=True)
    ]
    fit = fit_gravity(arm, noisy, running)
    assert refusals(fit) == []
    assert totals(fit)["held_out"] > totals(fit)["running"]
    assert [line.split()[0] for line in regressions(fit)] == ["shoulder", "elbow"]


def test_a_pair_the_joint_followed_less_than_halfway_is_kept_stuck(
    arm: ArmModel, poses: list[np.ndarray]
) -> None:
    joint, row = arm.measured_joints[0], arm.joint_names.index(arm.measured_joints[0])
    target = math.degrees(poses[0][row])

    def pair(moved_deg: float) -> Measurement:
        plus, minus = (
            hold(
                arm,
                joint,
                poses[0] + np.eye(3)[row] * math.radians(s * moved_deg / 2),
                target + s,
                s,
            )
            for s in (1, -1)
        )
        return measure_dither("p", arm.joint_names, plus, minus, NOISE, stuck_fraction=0.5)

    assert pair(0.9).stuck and pair(0.0).stuck and not pair(1.1).stuck
    held = pair(0.0)
    assert held.friction_nm == pytest.approx(KP * math.radians(DITHER_DEG))
    assert dither_friction_nm([held, pair(1.1)]) == {joint: pytest.approx(KP * math.radians(0.45))}
    assert stuck_pairs([held, pair(1.1)]) == (["p shoulder"], [])
    assert stuck_pairs([held]) == (["p shoulder"], ["shoulder"])
    plus = hold(arm, joint, poses[0], target + 1, 1)
    with pytest.raises(ValueError, match="not a"):
        measure_dither("p", arm.joint_names, plus, plus, NOISE, stuck_fraction=0.5)


def test_a_joint_that_was_not_dithered_keeps_its_urdf_torque(
    arm: ArmModel, poses: list[np.ndarray]
) -> None:
    shoulder = [m for m in dithers(arm, Compensation(True), poses) if m.joint == "shoulder"]
    held = fit_gravity(arm, shoulder, Compensation(True), ["shoulder"])
    free = fit_gravity(arm, shoulder, Compensation(True), ["shoulder", "elbow"])

    def elbow_change(fit: GravityFit) -> float:
        fitted = arm.model.copy()
        apply_inertial_overrides(fitted, fit.links)
        return max(
            abs(
                float(
                    pin.computeGeneralizedGravity(fitted, fitted.createData(), q)[2]
                    - arm.gravity(q)[2]
                )
            )
            for q in poses
        )

    assert held.held_joints == {"elbow": pytest.approx(0.0, abs=1e-9)} and free.held_joints == {}
    assert elbow_change(held) < 1e-9 < 0.1 < elbow_change(free)
    assert held.rms(held.held_out_error)["shoulder"] < 0.6 * held.rms(held.urdf_error)["shoulder"]
    assert fit_stats(held)["held_joints"] == held.held_joints
    assert "elbow has no measurement" in refusals(free)


def test_too_few_poses_raise(arm: ArmModel, poses: list[np.ndarray]) -> None:
    with pytest.raises(ValueError, match="three or more poses"):
        fit_gravity(arm, dithers(arm, Compensation(True), poses[:2]), Compensation(True))


def test_the_bounded_solve_keeps_its_constraints() -> None:
    a = np.random.default_rng(0).normal(size=(20, 4))
    b = a @ np.array([1.0, -2.0, 0.5, 0.0])
    free = _solve(a, b, 0.0, np.zeros((1, 4)), np.ones(1))
    bounded = _solve(a, b, 0.0, np.array([[0.0, -1.0, 0.0, 0.0]]), np.ones(1))
    assert free == pytest.approx([1.0, -2.0, 0.5, 0.0])
    assert bounded[1] == pytest.approx(-1.0)
