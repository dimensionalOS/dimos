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

from dataclasses import replace
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
from typer.testing import CliRunner

from dimos.control.calibration import gravity_fit
from dimos.control.calibration._calibration_test_helpers import (
    PERTURBATION,
    perturbed_model,
)
from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration.gravity_fit import (
    COM_BOX_M,
    COMPENSATION_KEY,
    FIT_RANK_TOLERANCE,
    AssumedCompensation,
    Compensation,
    GravityRecording,
    Measurement,
    PoseMarker,
    calibration_from_fit,
    default_holdout,
    extract_measurements,
    find_previous_json,
    fit_gravity,
    format_report,
    friction_flags,
    generate_gravity_report,
    load_gravity_recording,
    median_friction,
    pose_windows,
    report_json_name,
    save_refusals,
    to_urdf_error,
)
from dimos.control.calibration.gravity_poses import (
    ArmModel,
    Pose,
    PoseSet,
    generate_poses,
    home_pose,
)
from dimos.control.calibration.schema import (
    ArmCalibration,
    JointGains,
    bake_into_urdf,
    load_calibration,
    save_calibration,
)
from dimos.control.calibration.spec import ArmCalibrationSpec
from dimos.control.calibration.step_analysis import StepEvent
from dimos.control.calibration.step_report import JointTrace, Recording

OLD_RECORDING = (
    Path(__file__).resolve().parents[4]
    / "recordings"
    / "20261015-235349-091c-dual-openyam-step-test"
)
LEFT_MEASURED = tuple(f"left_joint{i}" for i in (2, 3, 4, 5))
# The 2026-10-16 bench gravity runs, one per arm, under this directory, and the
# fit they gave (calibration/reference/gravity_both_2026-10-16.json).
RECORDINGS_ENV = "DIMOS_CALIBRATION_RECORDINGS"
BENCH_RECORDINGS = {
    "left": "20261016-010922-4571-dual-openyam-step-test",
    "right": "20261016-014420-3612-dual-openyam-step-test",
}
REFERENCE_FIT = Path(__file__).resolve().parent / "reference" / "gravity_both_2026-10-16.json"
# Held-out RMS (before, after) the October fit reported, Nm, by joint number.
BENCH_HOLDOUT_RMS_NM = {
    "left": {2: (2.41, 0.31), 3: (1.48, 0.07)},
    "right": {2: (2.40, 0.20), 3: (1.71, 0.25)},
}


@pytest.fixture(scope="module")
def arm(left_spec: ArmCalibrationSpec) -> ArmModel:
    return ArmModel(left_spec)


@pytest.fixture(scope="module")
def pose_set(arm: ArmModel) -> PoseSet:
    return generate_poses(arm)


@pytest.fixture(scope="module")
def right_arm(right_spec: ArmCalibrationSpec) -> ArmModel:
    return ArmModel(right_spec)


@pytest.fixture(scope="module")
def right_pose_set(right_arm: ArmModel) -> PoseSet:
    return generate_poses(right_arm)


@pytest.fixture(scope="module")
def b601_arm(b601_spec: ArmCalibrationSpec) -> ArmModel:
    return ArmModel(b601_spec)


def synthetic_measurements(
    arm: ArmModel,
    truth: pin.Model,
    poses: list[Pose],
    *,
    noise_nm: float,
    friction_nm: float,
    seed: int = 0,
) -> list[Measurement]:
    """tau_pd = g_true(q) - g_model(q) + noise, as the +/- dither average would give."""
    rng = np.random.default_rng(seed)
    data = truth.createData()
    out = []
    for pose in poses:
        q = arm.q(pose.joints_rad)
        error = np.asarray(pin.computeGeneralizedGravity(truth, data, q)) - arm.gravity(q)
        for joint, row in zip(arm.measured_joints, arm.measured_rows, strict=True):
            out.append(
                Measurement(
                    pose=pose.name,
                    joint=joint,
                    kp=80.0,
                    kd=5.0,
                    dither_deg=1.0,
                    q_rad=pose.as_targets(),
                    tau_pd_nm=float(error[row] + rng.normal(0.0, noise_nm)),
                    friction_nm=float(friction_nm * (1.0 + 0.1 * rng.normal())),
                )
            )
    return out


def corrected_gravity(arm: ArmModel, links: dict[str, LinkInertial], q: np.ndarray) -> np.ndarray:
    model = arm.model.copy()
    apply_inertial_overrides(model, links)
    return np.asarray(pin.computeGeneralizedGravity(model, model.createData(), q))


def assert_recovers(
    arm: ArmModel, truth: pin.Model, pose_set: PoseSet, *, tolerance_nm: float
) -> None:
    """Fit synthetic data with friction, then check the held-out error and the true torque."""
    poses = [home_pose(arm), *pose_set.identification]
    measurements = synthetic_measurements(arm, truth, poses, noise_nm=0.02, friction_nm=0.4)
    result = fit_gravity(arm, measurements)
    assert result.bounds_ok and result.arm_id == arm.arm_id
    assert set(result.links) == set(arm.fitted_links)
    before = result.rms_by_joint(holdout=True, after=False)
    after = result.rms_by_joint(holdout=True, after=True)
    assert set(before) == set(arm.measured_joints)
    assert max(before.values()) > 0.5
    for joint in arm.measured_joints:
        assert after[joint] < tolerance_nm, (joint, after[joint])
        assert after[joint] < 0.5 * before[joint] or before[joint] < 0.1
    rows = list(arm.measured_rows)
    truth_data = truth.createData()
    for pose in pose_set.identification:
        if pose.name not in result.holdout_poses:
            continue
        q = arm.q(pose.joints_rad)
        expected = np.asarray(pin.computeGeneralizedGravity(truth, truth_data, q))[rows]
        got = corrected_gravity(arm, result.links, q)[rows]
        assert np.abs(got - expected).max() < tolerance_nm + 0.02


# Fit on synthetic data


def test_fit_recovers_an_injected_perturbation_on_held_out_poses(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    poses = [home_pose(arm), *pose_set.identification]
    measurements = synthetic_measurements(arm, truth, poses, noise_nm=0.02, friction_nm=0.4)
    result = fit_gravity(arm, measurements)

    assert result.holdout_poses == ["pose_04", "pose_09", "pose_14"]
    assert len(result.train_poses) == 13
    assert result.bounds_ok
    assert result.measured_joints == LEFT_MEASURED
    assert (
        max(
            result.rms_by_joint(holdout=True, after=False)[j]
            for j in ("left_joint2", "left_joint3")
        )
        > 0.5
    )
    assert_recovers(arm, truth, pose_set, tolerance_nm=0.06)
    assert result.rank >= 6 and result.condition_number < 50.0
    determined = [p for p in result.parameters if p.determined]
    assert determined and all(p.std_err > 0.0 for p in determined)
    assert all(p.observable <= 1.0 + 1e-9 for p in result.parameters)


def test_right_arm_fit_recovers_a_right_side_perturbation(
    right_arm: ArmModel, right_pose_set: PoseSet
) -> None:
    truth = perturbed_model(
        right_arm,
        {
            "right_gripper": (0.25, np.array([-0.02, 0.0, -0.01])),
            "right_link3": (0.15, np.array([0.0, 0.0, -0.02])),
        },
    )
    poses = [home_pose(right_arm), *right_pose_set.identification]
    measurements = synthetic_measurements(right_arm, truth, poses, noise_nm=0.02, friction_nm=0.4)
    assert {m.joint for m in measurements} == {f"right_joint{i}" for i in (2, 3, 4, 5)}
    result = fit_gravity(right_arm, measurements)
    assert result.arm_id == "dual_openyam/right" and result.bounds_ok
    assert set(result.links) == {
        "right_link2",
        "right_link3",
        "right_link4",
        "right_link5",
        "right_gripper",
    }
    assert_recovers(right_arm, truth, right_pose_set, tolerance_nm=0.06)
    # Left-arm measurements do not fit a right-arm model.
    left_measurement = Measurement(
        "x", "left_joint2", 80.0, 5.0, 1.0, home_pose(right_arm).as_targets(), 0.0, 0.3
    )
    with pytest.raises(ValueError, match="dual_openyam/right measured joints"):
        fit_gravity(right_arm, [left_measurement])


def test_fit_recovers_a_perturbation_on_a_second_robot(b601_arm: ArmModel) -> None:
    # The Seeed B601-DM with model-derived defaults: a heavier tool on the wrist
    # and an extra mass on the forearm.
    truth = perturbed_model(
        b601_arm,
        {
            "link6": (0.3, np.array([0.01, 0.0, 0.015])),
            "link3": (0.2, np.array([0.02, 0.0, 0.0])),
        },
    )
    assert b601_arm.measured_joints == tuple(f"joint{i}" for i in (2, 3, 4, 5))
    assert_recovers(b601_arm, truth, generate_poses(b601_arm), tolerance_nm=0.08)


def test_bake_into_urdf_reproduces_the_fitted_gravity(
    arm: ArmModel, pose_set: PoseSet, left_spec: ArmCalibrationSpec
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    measurements = synthetic_measurements(
        arm, truth, [home_pose(arm), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    result = fit_gravity(arm, measurements)
    calibration = calibration_from_fit(
        result, left_spec, created=datetime(2026, 11, 18, tzinfo=timezone.utc)
    )
    assert calibration.gravity is not None
    assert calibration.gravity.links["left_gripper"].merged_links == [
        "left_tip_left",
        "left_tip_right",
        "left_grasp_frame",
    ]
    assert calibration.friction_nm is not None and set(calibration.friction_nm) == set(
        LEFT_MEASURED
    )
    baked = ArmModel(
        left_spec,
        model=_reduced_from_xml(bake_into_urdf(calibration, left_spec.model.load().xml), arm),
    )
    for pose in pose_set.identification:
        q = arm.q(pose.joints_rad)
        assert np.allclose(baked.gravity(q), corrected_gravity(arm, result.links, q), atol=1e-9)


def _reduced_from_xml(xml: str, arm: ArmModel) -> pin.Model:
    full = pin.buildModelFromXML(xml)
    locked = [i for i, n in enumerate(full.names) if i and str(n) not in arm.joint_names]
    return pin.buildReducedModel(full, locked, pin.neutral(full))


def test_fit_stays_physical_and_flags_the_bound_when_the_data_want_more(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    # A 2 kg "camera" far out on the gripper: the bounded fit cannot follow it, so
    # the solution sits on a bound, says so, and the error it cannot explain remains.
    truth = perturbed_model(arm, {"left_gripper": (2.0, np.array([0.0, 0.0, -0.4]))})
    measurements = synthetic_measurements(
        arm, truth, [home_pose(arm), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    result = fit_gravity(arm, measurements, ridge_nm=0.05)
    assert result.bounds_ok
    assert any("mass sits at its bound (upper)" in v for v in result.at_bounds)
    unseen = [p for p in result.parameters if p.observable < 1e-12]
    assert ("left_link2", "m") in {(p.link, p.name) for p in unseen}
    for p in unseen:
        assert p.delta == pytest.approx(0.0, abs=1e-12), (p.link, p.name)
    for link, fitted in result.links.items():
        urdf = result.urdf_links[link]
        assert np.abs(np.asarray(fitted.com_m) - np.asarray(urdf.com_m)).max() <= COM_BOX_M + 1e-9
        assert 0.5 * urdf.mass_kg - 1e-9 <= fitted.mass_kg <= 2.0 * urdf.mass_kg + 1e-9
    after = result.rms_by_joint(holdout=True, after=True)
    assert max(after.values()) > 0.5


def test_directions_below_the_rank_tolerance_keep_the_urdf_values(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    base = pose_set.identification[3]
    poses = [base]
    for number, row in enumerate(arm.measured_rows):
        joints = list(base.joints_rad)
        joints[row] += math.radians(0.3)
        poses.append(Pose(f"near_{number}", tuple(joints), arm.joint_names))
    measurements = synthetic_measurements(arm, arm.model, poses, noise_nm=0.02, friction_nm=0.3)
    result = fit_gravity(arm, measurements, holdout=[], ridge_nm=0.05)
    rows = np.vstack(
        [
            arm.scaled_regressor(arm.q(m.q_rad))[arm.measured_joints.index(m.joint)]
            for m in measurements
        ]
    )
    _, s, vt = np.linalg.svd(rows, full_matrices=False)
    weak = (s <= FIT_RANK_TOLERANCE * s[0]) & (s > 1e-9 * s[0])
    assert weak.any() and result.rank == int((s > FIT_RANK_TOLERANCE * s[0]).sum())
    correction = np.asarray([p.delta for p in result.parameters]) / arm.column_scale
    assert np.abs(vt[result.rank :] @ correction).max() < 1e-9
    for p in result.parameters:
        if p.observable < 1e-12:
            assert p.value == pytest.approx(p.urdf, abs=1e-12), (p.link, p.name)


def test_a_failed_fit_produces_no_calibration(
    arm: ArmModel, pose_set: PoseSet, left_spec: ArmCalibrationSpec
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    measurements = synthetic_measurements(
        arm, truth, [home_pose(arm), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    result = fit_gravity(arm, measurements)
    failed = type(result)(**{**result.__dict__, "bound_violations": ["left_link2: made up"]})
    with pytest.raises(ValueError, match="failed its physical bounds"):
        calibration_from_fit(failed, left_spec)


def test_fit_rejects_unknown_holdout_and_everything_held_out(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    measurements = synthetic_measurements(
        arm,
        arm.model,
        [home_pose(arm), *pose_set.identification[:2]],
        noise_nm=0.0,
        friction_nm=0.3,
    )
    with pytest.raises(ValueError, match="not in the recording"):
        fit_gravity(arm, measurements, holdout=["pose_99"])
    with pytest.raises(ValueError, match="every pose is held out"):
        fit_gravity(arm, measurements, holdout=["home", "pose_01", "pose_02"])
    with pytest.raises(ValueError, match="nothing to fit"):
        fit_gravity(arm, [])


def test_default_holdout_is_every_fifth_pose_when_there_are_enough() -> None:
    poses = [f"pose_{i:02d}" for i in range(1, 16)]
    assert default_holdout(poses) == ["pose_05", "pose_10", "pose_15"]
    assert default_holdout(poses[:5]) == []
    assert default_holdout(poses, every=0) == []


def test_friction_flags_mark_outliers_and_negative_values(arm: ArmModel) -> None:
    q = home_pose(arm).as_targets()

    def m(pose: str, friction: float, kp: float = 400.0) -> Measurement:
        return Measurement(pose, "left_joint3", kp, 5.0, 1.0, q, 0.0, friction)

    measurements = [m("a", 0.5), m("b", 0.55), m("c", 0.45), m("d", 1.6), m("e", -0.1)]
    flags = friction_flags(measurements)
    assert set(flags) == {3, 4}
    assert "median" in flags[3] and "negative" in flags[4]
    # Friction of exactly kp*d/2 means the joint never moved for the dither pair.
    stuck = friction_flags([*measurements, m("f", 80.0 * math.radians(1.0) / 2.0, kp=80.0)])
    assert "did not move" in stuck[5]


def test_median_friction_uses_only_unflagged_pairs_and_is_never_negative(arm: ArmModel) -> None:
    q = home_pose(arm).as_targets()
    saturated = 80.0 * math.radians(1.0) / 2.0

    def m(joint: str, friction: float) -> Measurement:
        return Measurement("p", joint, 80.0, 5.0, 1.0, q, 0.0, friction)

    # Three of five left_joint3 pairs never moved: their "friction" is kp*d/2.
    measurements = [
        m("left_joint3", 0.40),
        m("left_joint3", 0.42),
        *(m("left_joint3", saturated) for _ in range(3)),
        m("left_joint2", -0.2),
        m("left_joint2", -0.1),
    ]
    assert median_friction(measurements) == {"left_joint3": pytest.approx(0.41)}


# Recording pipeline: markers + steps -> measurements -> report


def _synthetic_recording(
    arm: ArmModel,
    poses: list[Pose],
    prefix: str,
    truth: pin.Model | None = None,
    compensated: bool | None = True,
) -> GravityRecording:
    """A +1/-1 deg dither of each measured joint at each pose, settling short of its target.

    Friction leaves each dither 0.1 deg short; a ``truth`` model heavier than the
    arm's model sags each rest by the extra gravity torque over kp. Uncompensated
    (``compensated`` False) each rest sags by the whole gravity torque over kp;
    ``None`` leaves the compensation out of the markers.
    """
    dt, hold, kp = 0.01, 1.0, 80.0
    truth_data = truth.createData() if truth is not None else None
    held_up = truth if truth is not None else arm.model
    held_up_data = held_up.createData()
    data = {} if compensated is None else {COMPENSATION_KEY: Compensation(compensated).as_data()}
    t_all: list[float] = []
    q_all: list[np.ndarray] = []
    events: list[StepEvent] = []
    markers: list[PoseMarker] = []
    t = 0.0
    for pose in poses:
        markers.append(PoseMarker(pose.name, t, {}, data))
        q_pose = np.degrees(np.asarray(pose.joints_rad))
        error = np.zeros(arm.dof)
        if truth is not None:
            q = arm.q(pose.joints_rad)
            error = pin.computeGeneralizedGravity(truth, truth_data, q) - arm.gravity(q)
        for joint in arm.measured_joints:
            index = arm.joint_names.index(joint)
            sag_deg = math.degrees(error[index] / kp)
            for delta in (1.0, -1.0):
                target = q_pose.copy()
                target[index] += delta
                events.append(
                    StepEvent(
                        joint=f"{prefix}{joint}",
                        delta_deg=delta,
                        kp=kp,
                        kd=5.0,
                        t0=t,
                        # End the window before the next step's first sample.
                        hold_s=hold - 2 * dt,
                        target_before_deg=float(q_pose[index]),
                        target_after_deg=float(target[index]),
                    )
                )
                # The joint stops 0.1 deg short of each target: friction 80 * 0.1 deg.
                reached = target.copy()
                reached[index] -= 0.1 * delta + sag_deg
                for _ in range(0 if compensated is not False else 6):
                    load = pin.computeGeneralizedGravity(held_up, held_up_data, np.radians(reached))
                    reached[index] = target[index] - 0.1 * delta - math.degrees(load[index] / kp)
                for k in range(int(hold / dt)):
                    t_all.append(t + k * dt)
                    q_all.append(reached)
                t += hold
    times = np.asarray(t_all)
    angles = np.asarray(q_all)
    traces = {
        f"{prefix}{name}": JointTrace(t=times, q_deg=angles[:, i], dq_deg_s=np.zeros(times.size))
        for i, name in enumerate(arm.joint_names)
    }
    return GravityRecording(
        recording=Recording(traces=traces, events=events, t_end=float(times[-1])),
        markers=markers,
    )


@pytest.mark.parametrize("prefix", ["", "arm/"])
def test_extract_measurements_pairs_dithers_by_joint_name(
    arm: ArmModel, pose_set: PoseSet, left_spec: ArmCalibrationSpec, prefix: str
) -> None:
    poses = pose_set.identification[:3]
    measurements = extract_measurements(_synthetic_recording(arm, poses, prefix), left_spec)
    assert len(measurements) == len(poses) * len(LEFT_MEASURED)
    assert {m.joint for m in measurements} == set(LEFT_MEASURED)
    for m in measurements:
        assert set(m.q_rad) == set(left_spec.joint_names)
        pose = next(p for p in poses if p.name == m.pose)
        # The dither average sits at the pose; the friction is kp * 0.1 deg.
        assert m.q_rad[m.joint] == pytest.approx(pose.as_targets()[m.joint], abs=1e-9)
        assert m.friction_nm == pytest.approx(80.0 * math.radians(0.1), rel=1e-6)
        assert m.tau_pd_nm == pytest.approx(0.0, abs=1e-9)


def test_the_fit_takes_the_resting_torque_against_what_was_compensated(
    tmp_path: Path,
    arm: ArmModel,
    pose_set: PoseSet,
    left_spec: ArmCalibrationSpec,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    poses = [home_pose(arm), *pose_set.identification]
    run = tmp_path / "run"
    run.mkdir()
    held_by_pd = _synthetic_recording(arm, poses, "", compensated=False)
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: held_by_pd)
    result, _, calibration = generate_gravity_report(run, left_spec, arm=arm)
    raw = extract_measurements(held_by_pd, left_spec)
    assert max(abs(m.tau_pd_nm) for m in raw) > 1.0
    assert np.abs(result.residual_before).max() < 0.01
    assert calibration is not None and not result.at_bounds
    for link, fitted in result.links.items():
        assert fitted.mass_kg == pytest.approx(result.urdf_links[link].mass_kg, abs=0.01), link
    unknown = _synthetic_recording(arm, poses, "", compensated=None)
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: unknown)
    with pytest.raises(ValueError, match="does not record the gravity the connection compensated"):
        generate_gravity_report(run, left_spec, arm=arm)


def test_compensation_is_supplied_for_a_recording_that_does_not_record_it(
    tmp_path: Path,
    arm: ArmModel,
    pose_set: PoseSet,
    left_spec: ArmCalibrationSpec,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    poses = [home_pose(arm), *pose_set.identification]
    monkeypatch.setattr(gravity_fit, "calibration_path", lambda arm_id: tmp_path / "none.json")
    run = tmp_path / "run"
    run.mkdir()
    held_by_urdf = _synthetic_recording(arm, poses, "", compensated=None)
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: held_by_urdf)
    args = [str(run), "--robot", "dual-openyam", "--arm", "left"]

    refused = CliRunner().invoke(gravity_fit.app, args)
    assert refused.exit_code == 1 and isinstance(refused.exception, ValueError)
    assert "--compensation urdf or none" in str(refused.exception)
    assert not (run / "gravity_report").exists()
    assert CliRunner().invoke(gravity_fit.app, [*args, "--compensation", "file"]).exit_code == 2
    supplied = CliRunner().invoke(gravity_fit.app, [*args, "--compensation", "urdf"])
    assert supplied.exit_code == 0, supplied.output
    assert (run / "gravity_report" / "report.md").exists()

    urdf = AssumedCompensation.urdf.as_compensation()
    none = AssumedCompensation.none.as_compensation()
    assert (urdf, none) == (Compensation(True), Compensation(False))
    result, _, _ = generate_gravity_report(run, left_spec, arm=arm, assumed_compensation=urdf)
    assert np.abs(result.residual_before).max() < 0.01
    result, _, _ = generate_gravity_report(run, left_spec, arm=arm, assumed_compensation=none)
    assert np.abs(result.residual_before).max() > 1.0
    pd_only = _synthetic_recording(arm, poses, "", compensated=False)
    unrecorded = replace(pd_only, markers=[replace(m, data={}) for m in pd_only.markers])
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: unrecorded)
    result, _, _ = generate_gravity_report(run, left_spec, arm=arm, assumed_compensation=none)
    assert np.abs(result.residual_before).max() < 0.01
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: pd_only)
    result, _, _ = generate_gravity_report(run, left_spec, arm=arm, assumed_compensation=urdf)
    assert np.abs(result.residual_before).max() < 0.01


def test_urdf_error_adds_what_was_compensated_beyond_the_urdf(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    measurements = synthetic_measurements(
        arm, arm.model, pose_set.identification[:3], noise_nm=0.0, friction_nm=0.3
    )
    forearm = arm.model.inertias[arm.fitted_joint_ids[1]]
    heavy = {
        arm.fitted_links[1]: LinkInertial(
            mass_kg=float(forearm.mass) + 1.0, com_m=tuple(float(v) for v in forearm.lever)
        )
    }
    model = arm.model.copy()
    apply_inertial_overrides(model, heavy)
    data = model.createData()
    file = Compensation.from_data(json.loads(json.dumps(Compensation(True, heavy).as_data())))
    assert file == Compensation(True, heavy)
    urdf = to_urdf_error(arm, measurements, Compensation(True))
    none = to_urdf_error(arm, measurements, Compensation(False))
    ran_file = to_urdf_error(arm, measurements, file)
    for m, u, n, f in zip(measurements, urdf, none, ran_file, strict=True):
        q = arm.q(m.q_rad)
        row = arm.joint_names.index(m.joint)
        assert u.tau_pd_nm == m.tau_pd_nm
        assert n.tau_pd_nm == pytest.approx(m.tau_pd_nm - arm.gravity(q)[row], abs=1e-12)
        heavier = pin.computeGeneralizedGravity(model, data, q)[row] - arm.gravity(q)[row]
        assert f.tau_pd_nm == pytest.approx(m.tau_pd_nm + heavier, abs=1e-12)
    with pytest.raises(ValueError, match="unknown link"):
        to_urdf_error(
            arm, measurements, Compensation(True, {"no_such_link": heavy[arm.fitted_links[1]]})
        )


def test_report_pipeline_writes_the_calibration_and_keeps_stored_gains(
    tmp_path: Path,
    arm: ArmModel,
    pose_set: PoseSet,
    left_spec: ArmCalibrationSpec,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    recording = _synthetic_recording(arm, [home_pose(arm), *pose_set.identification], "", truth)
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: recording)
    stored = tmp_path / "state" / "dual_openyam" / "left"
    save_calibration(
        stored,
        ArmCalibration.for_spec(
            left_spec,
            gains={"left_joint2": JointGains(kp=90.0, kd=4.0)},
            checks={"float": {"passed": True}},
        ),
    )
    run = tmp_path / "20261118-run"
    run.mkdir()
    now = datetime(2026, 11, 18, 12, 0, tzinfo=timezone.utc)
    result, out_dir, calibration = generate_gravity_report(
        run, left_spec, arm=arm, now=now, previous=stored / "calibration.json", save_to=stored
    )
    assert result.bounds_ok and calibration is not None
    before = result.rms_by_joint(holdout=True, after=False)
    after = result.rms_by_joint(holdout=True, after=True)
    assert max(after.values()) < 0.1 < max(before.values())
    written = out_dir / "gravity_dual_openyam_left_2026-11-18.json"
    assert load_calibration(written, expected_model_sha256=left_spec.model_sha256()) == calibration
    assert {"report.md", "measurements.csv", "resting_torque.png"} <= {
        p.name for p in out_dir.iterdir()
    }
    assert "Compared with" in (out_dir / "report.md").read_text()
    saved = load_calibration(stored)
    assert saved.gravity == calibration.gravity and saved.created == now
    assert saved.gains == {"left_joint2": JointGains(kp=90.0, kd=4.0)}
    # The float check ran with the old masses; the new ones drop it.
    assert saved.checks is None
    assert saved.friction_nm == pytest.approx(
        dict.fromkeys(LEFT_MEASURED, 80.0 * math.radians(0.1)), rel=1e-6
    )
    assert len(list((stored / "history").iterdir())) == 1


def test_save_needs_a_held_out_pose_and_no_worse_held_out_error(
    tmp_path: Path,
    arm: ArmModel,
    pose_set: PoseSet,
    left_spec: ArmCalibrationSpec,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    recording = _synthetic_recording(arm, [home_pose(arm), *pose_set.identification], "", truth)
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: recording)
    stored = tmp_path / "left"
    run = tmp_path / "run"
    run.mkdir()
    # Nothing held out: the fit was never scored on poses it did not see.
    result, out_dir, calibration = generate_gravity_report(
        run, left_spec, arm=arm, holdout=[], save_to=stored
    )
    assert result.bounds_ok and calibration is not None and not result.holdout_poses
    assert save_refusals(result) == [
        "no pose was held out, so the fit was never scored on poses it did not see"
    ]
    assert not (stored / "calibration.json").exists()
    assert "Not saved as the arm's calibration (save anyway with --force)" in (
        (out_dir / "report.md").read_text()
    )
    generate_gravity_report(run, left_spec, arm=arm, holdout=[], save_to=stored, force=True)
    assert load_calibration(stored).gravity == calibration.gravity
    # A held-out error that rises on one joint is refused too.
    fitted = fit_gravity(arm, extract_measurements(recording, left_spec))
    assert save_refusals(fitted) == []
    worse = fitted.residual_after.copy()
    rows = [
        i
        for i, (m, held) in enumerate(zip(fitted.measurements, fitted.is_holdout, strict=True))
        if held and m.joint == "left_joint4"
    ]
    worse[rows] = 10.0 * fitted.residual_before[rows] + 0.1
    (reason,) = save_refusals(replace(fitted, residual_after=worse))
    assert reason.startswith("left_joint4 held-out RMS rose from")
    missing = [m for m in fitted.measurements if not (m.joint == "left_joint5")]
    assert "left_joint5 has no held-out measurement" in save_refusals(fit_gravity(arm, missing))
    # Better than the URDF is not enough when the arm already runs better masses.
    after = fitted.rms_by_joint(holdout=True, after=True)
    running = {joint: 0.5 * value for joint, value in after.items()}
    running["left_joint2"] = 2.0 * after["left_joint2"]
    refused = save_refusals(fitted, running)
    assert len(refused) == len(after) - 1
    assert refused[0].startswith("left_joint3 held-out RMS ")
    assert "is worse than the running calibration's" in refused[0]
    assert fitted.rms_of(fitted.residual_after, holdout=True) == after


def test_save_cli_refuses_without_force_and_shows_the_bounds(
    tmp_path: Path,
    arm: ArmModel,
    pose_set: PoseSet,
    left_spec: ArmCalibrationSpec,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    truth = perturbed_model(arm, {"left_gripper": (2.0, np.array([0.0, 0.0, -0.4]))})
    recording = _synthetic_recording(arm, [home_pose(arm), *pose_set.identification], "", truth)
    monkeypatch.setattr(gravity_fit, "load_gravity_recording", lambda _: recording)
    target = tmp_path / "state" / "calibration.json"
    monkeypatch.setattr(gravity_fit, "calibration_path", lambda arm_id: target)
    run = tmp_path / "run"
    run.mkdir()
    args = [str(run), "--robot", "dual-openyam", "--arm", "left", "--save"]
    saved = CliRunner().invoke(gravity_fit.app, args)
    # Every fifth pose held out and the error down: saved, the bounds it hit shown.
    assert saved.exit_code == 0, saved.output
    assert "Saved the calibration of dual_openyam/left" in saved.output
    (bounds,) = [line for line in saved.output.splitlines() if line.startswith("  at bounds: ")]
    assert "mass sits at its bound (upper)" in bounds
    target.unlink()
    labels = ",".join(p.name for p in pose_set.identification[:1])
    monkeypatch.setattr(gravity_fit, "save_refusals", lambda result: ["made up"])
    refused = CliRunner().invoke(gravity_fit.app, [*args, "--holdout", labels])
    assert refused.exit_code == 1
    assert "Did not save the calibration of dual_openyam/left: made up" in refused.output
    assert not target.exists()
    forced = CliRunner().invoke(gravity_fit.app, [*args, "--holdout", labels, "--force"])
    assert forced.exit_code == 0, forced.output
    assert "despite (--force): made up" in forced.output and target.exists()


def test_stored_checks_survive_a_refit_with_the_same_masses(
    tmp_path: Path, arm: ArmModel, pose_set: PoseSet, left_spec: ArmCalibrationSpec
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    measurements = synthetic_measurements(
        arm, truth, [home_pose(arm), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    new = calibration_from_fit(fit_gravity(arm, measurements), left_spec)
    stored = save_calibration(
        tmp_path, new.model_copy(update={"checks": {"float": {"passed": True}}, "notes": "x"})
    )
    kept = gravity_fit.merge_with_stored(stored, new, left_spec)
    assert kept.checks == {"float": {"passed": True}} and kept.notes == "x"


def test_report_compares_with_a_previous_calibration(
    tmp_path: Path, arm: ArmModel, pose_set: PoseSet, left_spec: ArmCalibrationSpec
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    measurements = synthetic_measurements(
        arm, truth, [home_pose(arm), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    result = fit_gravity(arm, measurements)
    created = datetime(2026, 10, 1, 12, 0, tzinfo=timezone.utc)
    previous = tmp_path / report_json_name(left_spec.arm_id, created)
    assert previous.name == "gravity_dual_openyam_left_2026-10-01.json"
    previous.write_text(
        calibration_from_fit(result, left_spec, created=created).model_dump_json(indent=2)
    )
    assert find_previous_json([tmp_path], left_spec.arm_id) == previous
    assert find_previous_json([tmp_path], left_spec.arm_id, exclude=previous) is None
    assert find_previous_json([tmp_path], "dual_openyam/right") is None
    assert find_previous_json([tmp_path / "missing"], left_spec.arm_id) is None
    text = format_report(result, previous, None)
    assert f"## Compared with {previous}" in text
    assert text.startswith("# Gravity-model fit: dual_openyam/left")
    # A legacy {"links": ...} file still compares.
    legacy = tmp_path / "legacy.json"
    legacy.write_text(
        json.dumps({"links": {"left_gripper": {"mass_kg": 0.9, "com_m": [0.0, 0.0, -0.05]}}})
    )
    assert "| left_gripper | 0.900 |" in format_report(result, legacy, None)


def test_saved_calibration_round_trips_with_the_fit(
    tmp_path: Path, arm: ArmModel, pose_set: PoseSet, left_spec: ArmCalibrationSpec
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    measurements = synthetic_measurements(
        arm, truth, [home_pose(arm), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    result = fit_gravity(arm, measurements)
    calibration = calibration_from_fit(result, left_spec)
    path = save_calibration(tmp_path / "dual_openyam" / "left", calibration)
    loaded = load_calibration(path, expected_model_sha256=left_spec.model_sha256())
    assert isinstance(loaded, ArmCalibration) and loaded == calibration
    assert loaded.gravity is not None
    assert loaded.gravity.fit["source"] == "dimos.control.calibration.gravity_fit"
    assert loaded.gravity.fit["holdout_poses"] == result.holdout_poses


def test_pose_windows_without_markers_cover_everything() -> None:
    assert pose_windows([], 10.0) == [("all", -math.inf, math.inf)]
    markers = [PoseMarker("a", 1.0, {}, {}), PoseMarker("b", 5.0, {}, {})]
    assert pose_windows(markers, 9.0) == [("a", 1.0, 5.0), ("b", 5.0, 9.0)]


@pytest.mark.skipif(not OLD_RECORDING.exists(), reason="rig recording not present")
def test_fit_runs_on_the_single_pose_step_test_recording(
    arm: ArmModel, left_spec: ArmCalibrationSpec
) -> None:
    # Rank deficient by construction (one pose): it must still run and say so.
    grec = load_gravity_recording(OLD_RECORDING)
    assert not grec.markers
    measurements = extract_measurements(grec, left_spec)
    assert {m.joint for m in measurements} == set(LEFT_MEASURED)
    result = fit_gravity(arm, measurements)
    assert result.train_poses == ["all"] and not result.holdout_poses
    assert result.rank < len(result.parameters)
    assert sum(p.determined for p in result.parameters) <= result.rank


def _bench_recording(side: str) -> Path:
    root = os.environ.get(RECORDINGS_ENV)
    if not root:
        pytest.skip(f"set {RECORDINGS_ENV} to the directory of the 2026-10-16 bench recordings")
    recording = Path(root) / BENCH_RECORDINGS[side]
    if not (recording / "memory.db").exists():
        pytest.skip(f"no bench recording at {recording}")
    return recording


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_bench_recordings_reproduce_the_october_fit(
    side: str, dual_openyam_specs: list[ArmCalibrationSpec]
) -> None:
    # Reads the recording only; no report is written next to it.
    recording = _bench_recording(side)
    spec = next(s for s in dual_openyam_specs if s.arm_id == f"dual_openyam/{side}")
    grec = load_gravity_recording(recording)
    measurements = extract_measurements(grec, spec)
    assert len(measurements) == 60  # 15 poses, 4 measured joints
    result = fit_gravity(ArmModel(spec), measurements)
    assert result.holdout_poses == ["pose_05", "pose_10", "pose_15"]
    reference = json.loads(REFERENCE_FIT.read_text())["links"]
    assert set(result.links) == set(spec.gravity_links)
    for link, fitted in result.links.items():
        assert fitted.mass_kg == pytest.approx(reference[link]["mass_kg"], abs=1e-3), link
    before = result.rms_by_joint(holdout=True, after=False)
    after = result.rms_by_joint(holdout=True, after=True)
    for number, (rms_before, rms_after) in BENCH_HOLDOUT_RMS_NM[side].items():
        joint = f"{side}_joint{number}"
        assert before[joint] == pytest.approx(rms_before, abs=0.02), joint
        assert after[joint] == pytest.approx(rms_after, abs=0.02), joint
    # The known-good fits pass the save check, the left one with a COM at its bound.
    assert save_refusals(result) == []
    if side == "left":
        assert result.at_bounds == ["left_link3 COM z sits at its bound (upper)"]
