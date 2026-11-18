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

from datetime import datetime
import json
import math
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest

from dimos.robot.manipulators.calibration._inertial import (
    LinkInertial,
    apply_inertial_overrides,
    load_inertial_overrides,
)
from dimos.robot.manipulators.calibration.gravity_fit import (
    COM_BOX_M,
    Measurement,
    PoseMarker,
    default_holdout,
    extract_measurements,
    find_previous_json,
    fit_gravity,
    friction_flags,
    load_gravity_recording,
    merge_inertial_json,
    pose_windows,
)
from dimos.robot.manipulators.calibration.gravity_poses import (
    MEASURED_JOINTS,
    ArmModel,
    Pose,
    PoseSet,
    generate_poses,
    home_pose,
)

OLD_RECORDING = (
    Path(__file__).resolve().parents[5]
    / "recordings"
    / "20261015-235349-091c-dual-openyam-step-test"
)
# A camera and bracket on the gripper, a heavier forearm: what the rig likely has.
PERTURBATION = {
    "left_gripper": (0.25, np.array([-0.02, 0.0, -0.01])),  # d mass (kg), d(m*c) (kg m)
    "left_link3": (0.15, np.array([0.0, 0.0, -0.02])),
}


@pytest.fixture(scope="module")
def arm() -> ArmModel:
    return ArmModel()


@pytest.fixture(scope="module")
def pose_set(arm: ArmModel) -> PoseSet:
    return generate_poses(arm)


@pytest.fixture(scope="module")
def right_arm() -> ArmModel:
    return ArmModel(side="right")


@pytest.fixture(scope="module")
def right_pose_set(right_arm: ArmModel) -> PoseSet:
    return generate_poses(right_arm)


def perturbed_model(arm: ArmModel, perturbation: dict[str, tuple[float, np.ndarray]]) -> pin.Model:
    model = arm.model.copy()
    for link, (d_mass, d_moment) in perturbation.items():
        joint_id = model.getFrameId(link)
        joint_id = int(model.frames[joint_id].parentJoint)
        inertia = model.inertias[joint_id]
        mass = inertia.mass + d_mass
        com = (inertia.mass * inertia.lever + d_moment) / mass
        model.inertias[joint_id] = pin.Inertia(mass, com, inertia.inertia)
    return model


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
        q = arm.full_q(pose.joints_rad)
        error = np.asarray(pin.computeGeneralizedGravity(truth, data, q)) - arm.gravity(q)
        for joint, row in zip(arm.measured_joints, arm.measured_rows, strict=True):
            out.append(
                Measurement(
                    pose=pose.name,
                    joint=joint,
                    kp=80.0,
                    kd=5.0,
                    dither_deg=1.0,
                    q_rad=tuple(float(v) for v in q),
                    tau_pd_nm=float(error[row] + rng.normal(0.0, noise_nm)),
                    friction_nm=float(friction_nm * (1.0 + 0.1 * rng.normal())),
                )
            )
    return out


def corrected_gravity(arm: ArmModel, links: dict[str, LinkInertial], q: np.ndarray) -> np.ndarray:
    model = arm.model.copy()
    apply_inertial_overrides(model, links)
    return np.asarray(pin.computeGeneralizedGravity(model, model.createData(), q))


# Fit on synthetic data


def test_fit_recovers_an_injected_perturbation_on_held_out_poses(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    truth = perturbed_model(arm, PERTURBATION)
    poses = [home_pose(), *pose_set.identification]
    measurements = synthetic_measurements(arm, truth, poses, noise_nm=0.02, friction_nm=0.4)
    result = fit_gravity(arm, measurements)

    assert result.holdout_poses == ["pose_04", "pose_09", "pose_14"]
    assert len(result.train_poses) == 13
    assert result.bounds_ok
    before = result.rms_by_joint(holdout=True, after=False)
    after = result.rms_by_joint(holdout=True, after=True)
    assert set(before) == set(MEASURED_JOINTS)
    assert max(before["left_joint2"], before["left_joint3"]) > 0.5
    for joint in MEASURED_JOINTS:
        assert after[joint] < 0.06, (joint, after[joint])
        assert after[joint] < 0.5 * before[joint] or before[joint] < 0.1
    # The corrected model predicts the true gravity torque at poses it never saw.
    rows = list(arm.measured_rows)
    truth_data = truth.createData()
    for pose in pose_set.identification:
        if pose.name not in result.holdout_poses:
            continue
        q = arm.full_q(pose.joints_rad)
        expected = np.asarray(pin.computeGeneralizedGravity(truth, truth_data, q))[rows]
        got = corrected_gravity(arm, result.links, q)[rows]
        assert np.abs(got - expected).max() < 0.08
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
    poses = [home_pose("right"), *right_pose_set.identification]
    measurements = synthetic_measurements(right_arm, truth, poses, noise_nm=0.02, friction_nm=0.4)
    assert {m.joint for m in measurements} == {f"right_joint{i}" for i in (2, 3, 4, 5)}
    result = fit_gravity(right_arm, measurements)
    assert result.side == "right" and result.bounds_ok
    assert set(result.links) == {
        "right_link2",
        "right_link3",
        "right_link4",
        "right_link5",
        "right_gripper",
    }
    before = result.rms_by_joint(holdout=True, after=False)
    after = result.rms_by_joint(holdout=True, after=True)
    assert set(after) == {f"right_joint{i}" for i in (2, 3, 4, 5)}
    assert max(before.values()) > 0.5
    assert max(after.values()) < 0.06
    # The left arm, held at home, is untouched by the correction.
    q = right_arm.full_q(right_pose_set.identification[0].joints_rad)
    assert np.allclose(corrected_gravity(right_arm, result.links, q)[:6], right_arm.gravity(q)[:6])
    # Left-arm measurements do not fit a right-arm model.
    left_measurement = Measurement("x", "left_joint2", 80.0, 5.0, 1.0, tuple(q), 0.0, 0.3)
    with pytest.raises(ValueError, match="right measured joints"):
        fit_gravity(right_arm, [left_measurement])


def test_merge_joins_both_arms_into_one_override_file(tmp_path: Path) -> None:
    left = tmp_path / "gravity_left_2026-10-16.json"
    right = tmp_path / "gravity_right_2026-10-17.json"
    left.write_text(
        json.dumps(
            {
                "created": "2026-10-16T10:00:00",
                "side": "left",
                "links": {
                    "left_gripper": {"mass_kg": 0.9, "com_m": [0.0, 0.0, -0.04]},
                    "right_link3": {"mass_kg": 9.0, "com_m": [0.0, 0.0, 0.0]},  # stale, overridden
                },
            }
        )
    )
    right.write_text(
        json.dumps(
            {
                "created": "2026-10-17T10:00:00",
                "side": "right",
                "links": {"right_link3": {"mass_kg": 1.1, "com_m": [-0.05, -0.03, -0.12]}},
            }
        )
    )
    merged = merge_inertial_json([left, right], datetime(2026, 10, 17, 12, 0))
    assert merged["side"] == "both" and merged["created"] == "2026-10-17T12:00:00"
    assert merged["links"] == {
        "left_gripper": {"mass_kg": 0.9, "com_m": [0.0, 0.0, -0.04]},
        "right_link3": {"mass_kg": 1.1, "com_m": [-0.05, -0.03, -0.12]},
    }
    assert [s["side"] for s in merged["merged_from"]] == ["left", "right"]
    out = tmp_path / "gravity_both.json"
    out.write_text(json.dumps(merged))
    assert set(load_inertial_overrides(out)) == {"left_gripper", "right_link3"}


def test_fit_stays_physical_and_flags_the_bound_when_the_data_want_more(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    # A 2 kg "camera" far out on the gripper: the bounded fit cannot follow it, so
    # the solution sits on a bound, says so, and the error it cannot explain remains.
    truth = perturbed_model(arm, {"left_gripper": (2.0, np.array([0.0, 0.0, -0.4]))})
    measurements = synthetic_measurements(
        arm, truth, [home_pose(), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    result = fit_gravity(arm, measurements, ridge_nm=0.05)
    assert result.bounds_ok
    assert any("left_gripper" in v and "bound" in v for v in result.at_bounds)
    for link, fitted in result.links.items():
        urdf = result.urdf_links[link]
        assert np.abs(np.asarray(fitted.com_m) - np.asarray(urdf.com_m)).max() <= COM_BOX_M + 1e-9
        assert 0.5 * urdf.mass_kg - 1e-9 <= fitted.mass_kg <= 2.0 * urdf.mass_kg + 1e-9
    after = result.rms_by_joint(holdout=True, after=True)
    assert max(after.values()) > 0.5


def test_fit_rejects_unknown_holdout_and_everything_held_out(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    measurements = synthetic_measurements(
        arm, arm.model, [home_pose(), *pose_set.identification[:2]], noise_nm=0.0, friction_nm=0.3
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
    q = tuple(arm.full_q(home_pose().joints_rad))

    def m(pose: str, friction: float, kp: float = 400.0) -> Measurement:
        return Measurement(pose, "left_joint3", kp, 5.0, 1.0, q, 0.0, friction)

    measurements = [m("a", 0.5), m("b", 0.55), m("c", 0.45), m("d", 1.6), m("e", -0.1)]
    flags = friction_flags(measurements)
    assert set(flags) == {3, 4}
    assert "median" in flags[3] and "negative" in flags[4]
    # Friction of exactly kp*d/2 means the joint never moved for the dither pair.
    stuck = friction_flags([*measurements, m("f", 80.0 * math.radians(1.0) / 2.0, kp=80.0)])
    assert "did not move" in stuck[5]


# Recording pipeline: markers + steps -> memory.db -> report


def test_report_compares_with_a_previous_json(
    tmp_path: Path, arm: ArmModel, pose_set: PoseSet
) -> None:
    previous = tmp_path / "gravity_2026-10-01.json"
    previous.write_text(
        json.dumps({"links": {"left_gripper": {"mass_kg": 0.9, "com_m": [0.0, 0.0, -0.05]}}})
    )
    assert find_previous_json([tmp_path]) == previous
    assert find_previous_json([tmp_path], exclude=previous) is None
    assert find_previous_json([tmp_path / "missing"]) is None
    truth = perturbed_model(arm, PERTURBATION)
    measurements = synthetic_measurements(
        arm, truth, [home_pose(), *pose_set.identification], noise_nm=0.0, friction_nm=0.3
    )
    result = fit_gravity(arm, measurements)
    from dimos.robot.manipulators.calibration.gravity_fit import (
        format_report,
    )

    text = format_report(result, previous, None)
    assert f"## Compared with {previous}" in text
    assert "| left_gripper | 0.900 |" in text


def test_pose_windows_without_markers_cover_everything() -> None:
    assert pose_windows([], 10.0) == [("all", -math.inf, math.inf)]
    markers = [PoseMarker("a", 1.0, {}, {}), PoseMarker("b", 5.0, {}, {})]
    assert pose_windows(markers, 9.0) == [("a", 1.0, 5.0), ("b", 5.0, 9.0)]


@pytest.mark.skipif(not OLD_RECORDING.exists(), reason="rig recording not present")
def test_fit_runs_on_the_single_pose_step_test_recording(arm: ArmModel, tmp_path: Path) -> None:
    # Rank deficient by construction (one pose): it must still run and say so.
    grec = load_gravity_recording(OLD_RECORDING)
    assert not grec.markers
    measurements = extract_measurements(grec)
    assert {m.joint for m in measurements} == set(MEASURED_JOINTS)
    result = fit_gravity(arm, measurements)
    assert result.train_poses == ["all"] and not result.holdout_poses
    assert result.rank < len(result.parameters)
    assert sum(p.determined for p in result.parameters) <= result.rank
