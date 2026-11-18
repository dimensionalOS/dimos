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
import math

import numpy as np
import pinocchio as pin
import pytest

from dimos.robot.manipulators.calibration.gravity_poses import (
    GRAVITY_FREE_LIMIT_NM,
    GRAVITY_PARAMS_PER_BODY,
    ArmModel,
    PoseSet,
    body_joint_id,
    format_pose_set,
    generate_poses,
    grid_axes,
    home_pose,
)
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec, HalfSpace
from dimos.robot.manipulators.dual_openyam.calibration_workspace import (
    DUAL_OPENYAM_BENCH_BASE_SPACING_M,
    DUAL_OPENYAM_CALIBRATION_RANGES_DEG,
    DUAL_OPENYAM_MIDLINE_MARGIN_M,
)
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL

# The checks the bench pose filter applied before the spec existed, on the full
# 12-joint model, kept here to show the spec-driven poses still pass them.
OLD_MIN_POSE_Z_M = 0.12
OLD_MIN_POSE_X_M = 0.15
OLD_MIN_TRANSITION_Z_M = 0.08
OLD_MAX_MODEL_TORQUE_NM = 10.0
OLD_FINGERTIP_OFFSET_M = 0.046
OLD_CLEARANCE_SUFFIXES = ("tip_left", "tip_right", "gripper", "link5", "link4")
OLD_RIG_HALF_SPACING_M = 0.430 / 2.0
OLD_MAX_MIDLINE_CROSSING_M = 0.05
LEFT_FITTED_LINKS = ("left_link2", "left_link3", "left_link4", "left_link5", "left_gripper")


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


class OldBenchChecks:
    """The pre-spec pose and transition checks, on the full two-arm model."""

    def __init__(self, side: str) -> None:
        self.model = pin.buildModelFromXML(DUAL_OPENYAM_MODEL.load().xml)
        self.data = self.model.createData()
        self.side = side
        self.offset = 0 if side == "left" else 6
        self.grasp = self.model.getFrameId(f"{side}_grasp_frame")
        self.others = [self.model.getFrameId(f"{side}_{s}") for s in OLD_CLEARANCE_SUFFIXES]
        self.base_y = float(
            self.model.frames[self.model.getFrameId(f"{side}_base")].placement.translation[1]
        )
        self.toward = -1.0 if side == "left" else 1.0

    def full_q(self, joints: tuple[float, ...] | np.ndarray) -> np.ndarray:
        q = np.zeros(self.model.nq)
        q[self.offset : self.offset + 6] = joints
        return q

    def points(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pin.framesForwardKinematics(self.model, self.data, q)
        tip = self.data.oMf[self.grasp].act(np.array([0.0, 0.0, -OLD_FINGERTIP_OFFSET_M]))
        return tip, np.stack([self.data.oMf[f].translation for f in self.others])

    def midline_ok(self, tip: np.ndarray) -> bool:
        toward = self.toward * (float(tip[1]) - self.base_y)
        return toward <= OLD_RIG_HALF_SPACING_M + OLD_MAX_MIDLINE_CROSSING_M

    def pose_ok(self, joints: tuple[float, ...]) -> bool:
        q = self.full_q(joints)
        tip, others = self.points(q)
        gravity = pin.computeGeneralizedGravity(self.model, self.data, q)
        return bool(
            tip[2] > OLD_MIN_POSE_Z_M
            and (others[:, 2] > OLD_MIN_POSE_Z_M).all()
            and tip[0] > OLD_MIN_POSE_X_M
            and self.midline_ok(tip)
            and np.abs(gravity).max() <= OLD_MAX_MODEL_TORQUE_NM
        )

    def transition_ok(self, a: tuple[float, ...], b: tuple[float, ...], samples: int) -> bool:
        start, end = np.asarray(a), np.asarray(b)
        for s in np.linspace(0.0, 1.0, samples + 1):
            tip, others = self.points(self.full_q(start + s * (end - start)))
            if tip[2] <= OLD_MIN_TRANSITION_Z_M or (others[:, 2] <= OLD_MIN_TRANSITION_Z_M).any():
                return False
            if not self.midline_ok(tip):
                return False
        return True


def test_regressor_reproduces_pinocchio_gravity(arm: ArmModel) -> None:
    # Y(q) @ phi on the fitted columns equals the gravity torque of the fitted bodies
    # alone, i.e. the full model minus a copy with those bodies made massless.
    q = arm.q(home_pose(arm).joints_rad)
    phi = np.concatenate(
        [
            [
                arm.model.inertias[j].mass,
                *(arm.model.inertias[j].mass * arm.model.inertias[j].lever),
            ]
            for j in arm.fitted_joint_ids
        ]
    )
    rest = arm.model.copy()
    for j in arm.fitted_joint_ids:
        rest.inertias[j] = pin.Inertia(0.0, np.zeros(3), np.zeros((3, 3)))
    g_rest = pin.computeGeneralizedGravity(rest, rest.createData(), q)
    rows = list(arm.measured_rows)
    expected = arm.gravity(q)[rows] - np.asarray(g_rest)[rows]
    assert np.allclose(arm.regressor(q) @ phi, expected, atol=1e-9)


def test_generates_the_requested_number_of_poses(pose_set: PoseSet) -> None:
    assert len(pose_set.identification) == 15
    assert [p.name for p in pose_set.identification[:2]] == ["pose_01", "pose_02"]
    assert pose_set.candidates > 500
    assert pose_set.arm_id == "dual_openyam/left"
    assert set(pose_set.grid_step_deg) == {f"left_joint{i}" for i in (2, 3, 4, 5)}


def test_bench_poses_come_from_the_bench_grid(arm: ArmModel, pose_set: PoseSet) -> None:
    # The hand-set bench grid, not the derived one (about 9.6, 13.5, 30, 50 deg).
    assert pose_set.grid_step_deg == pytest.approx(
        {"left_joint2": 12.5, "left_joint3": 13.5, "left_joint4": 25.0, "left_joint5": 37.5}
    )
    axes, _ = grid_axes(arm)
    assert [len(axis) for axis in axes] == [1, 11, 11, 7, 5, 1]
    for number, axis in enumerate(axes[1:5], 2):
        low, high = DUAL_OPENYAM_CALIBRATION_RANGES_DEG[number]
        assert np.degrees(axis) == pytest.approx(np.arange(low, high + 1e-9, axis_step(number)))
    for pose in pose_set.identification:
        for number, value in enumerate(pose.joints_deg[1:5], 2):
            low = DUAL_OPENYAM_CALIBRATION_RANGES_DEG[number][0]
            steps = (value - low) / axis_step(number)
            assert steps == pytest.approx(round(steps), abs=1e-9), (pose.name, number)


def axis_step(number: int) -> float:
    return {2: 12.5, 3: 13.5, 4: 25.0, 5: 37.5}[number]


def test_given_grid_steps_replace_only_their_joints_derived_steps(
    b601_spec: ArmCalibrationSpec,
) -> None:
    derived = grid_axes(ArmModel(b601_spec))[1]
    given = ArmModel(replace(b601_spec, grid_step_rad={"joint3": math.radians(20.0)}))
    axes, steps = grid_axes(given)
    assert steps["joint3"] == pytest.approx(20.0, rel=0.1)
    assert len(axes[given.joint_names.index("joint3")]) == (
        round(math.degrees(np.ptp(b601_spec.joint_ranges["joint3"])) / 20.0) + 1
    )
    assert set(steps) == set(derived)
    with pytest.raises(ValueError, match="not measured"):
        replace(b601_spec, grid_step_rad={"joint1": 0.1})
    with pytest.raises(ValueError, match="positive"):
        replace(b601_spec, grid_step_rad={"joint2": 0.0})


def test_poses_stay_inside_the_joint_ranges(arm: ArmModel, pose_set: PoseSet) -> None:
    for pose in pose_set.identification:
        assert pose.joint_names == tuple(DUAL_OPENYAM_LEFT_ARM_JOINTS)
        for number, (joint, value) in enumerate(
            zip(pose.joint_names, pose.joints_deg, strict=True), 1
        ):
            low, high = DUAL_OPENYAM_CALIBRATION_RANGES_DEG[number]
            assert low - 1e-9 <= value <= high + 1e-9, (pose.name, joint, value)
            lo_rad, hi_rad = arm.spec.joint_ranges[joint]
            assert lo_rad - 1e-12 <= pose.as_targets()[joint] <= hi_rad + 1e-12
        # The yaw and the wrist roll carry no gravity signal and stay at home.
        assert pose.joints_deg[0] == 0.0 and pose.joints_deg[5] == 0.0


def test_poses_pass_the_old_bench_checks(arm: ArmModel, pose_set: PoseSet) -> None:
    old = OldBenchChecks("left")
    previous = home_pose(arm).joints_rad
    for pose in pose_set.identification:
        assert old.pose_ok(pose.joints_rad), pose.name
        assert old.transition_ok(previous, pose.joints_rad, samples=100), pose.name
        previous = pose.joints_rad
    for found in pose_set.gravity_free.values():
        assert old.pose_ok(found.pose.joints_rad), found.pose.name


def test_poses_clear_the_table_and_sit_in_front(arm: ArmModel, pose_set: PoseSet) -> None:
    guard = arm.workspace
    assert guard.min_pose_z_m == pytest.approx(OLD_MIN_POSE_Z_M)
    assert guard.min_transition_z_m == pytest.approx(OLD_MIN_TRANSITION_Z_M)
    for pose in pose_set.identification:
        q = arm.q(pose.joints_rad)
        tip, others = arm.points(q)
        assert tip[2] > guard.min_pose_z_m and tip[0] > OLD_MIN_POSE_X_M, pose.name
        assert (others[:, 2] > guard.min_pose_z_m).all(), pose.name
        assert (np.abs(arm.gravity(q)) <= arm.max_torque).all(), pose.name


def test_poses_are_reachable_in_sequence_from_home(arm: ArmModel, pose_set: PoseSet) -> None:
    previous = home_pose(arm).joints_rad
    for pose in pose_set.identification:
        assert arm.transition_ok(previous, pose.joints_rad), pose.name
        # The interpolation check is what guards the table: sample it densely too.
        a, b = np.asarray(previous), np.asarray(pose.joints_rad)
        for s in np.linspace(0.0, 1.0, 101):
            tip, others = arm.points(a + s * (b - a))
            limit = arm.workspace.min_transition_z_m
            assert tip[2] > limit and (others[:, 2] > limit).all()
        previous = pose.joints_rad


def test_poses_are_distinct_and_vary_every_measured_joint(pose_set: PoseSet) -> None:
    angles = np.asarray([p.joints_deg for p in pose_set.identification])
    assert len({tuple(np.round(row, 3)) for row in angles}) == len(angles)
    # The greedy pick optimises conditioning, not spread; every measured joint
    # still sweeps well over a third of its box.
    for column in (1, 2, 3, 4):
        assert angles[:, column].max() - angles[:, column].min() > 50.0


def test_regressor_is_well_conditioned_on_its_identifiable_directions(pose_set: PoseSet) -> None:
    assert pose_set.rank == 8
    assert len(pose_set.singular_values) == pose_set.rank
    assert math.isfinite(pose_set.condition_number)
    assert pose_set.condition_number < 20.0


def test_gravity_free_poses_null_or_minimise_their_joint_torque(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    assert set(pose_set.gravity_free) == set(arm.measured_joints)
    for joint, found in pose_set.gravity_free.items():
        row = arm.joint_names.index(joint)
        torque = arm.gravity(arm.q(found.pose.joints_rad))[row]
        assert torque == pytest.approx(found.torque_nm, abs=1e-9)
        assert arm.pose_ok(found.pose.joints_rad)
        if found.exact:
            assert abs(torque) < GRAVITY_FREE_LIMIT_NM
        else:
            # Best effort is reported with its torque and explained in the notes.
            assert any(joint in note and "best effort" in note for note in pose_set.notes)
    # Joints 2 and 4 can be nulled inside the boxes; joint 3 only by reaching back.
    assert pose_set.gravity_free["left_joint2"].exact
    assert pose_set.gravity_free["left_joint4"].exact
    assert any("left_joint3" in note and "behind" in note for note in pose_set.notes)


def test_body_joint_id_requires_a_link_frame_on_its_joint(arm: ArmModel) -> None:
    assert arm.fitted_links == LEFT_FITTED_LINKS
    assert [body_joint_id(arm.model, link) for link in LEFT_FITTED_LINKS] == [2, 3, 4, 5, 6]
    with pytest.raises(ValueError, match="coincident"):
        body_joint_id(arm.model, "left_grasp_frame")
    with pytest.raises(ValueError, match="no frame"):
        body_joint_id(arm.model, "left_link9")
    assert len(arm.param_columns) == len(LEFT_FITTED_LINKS) * GRAVITY_PARAMS_PER_BODY


def test_format_lists_every_pose_in_degrees(arm: ArmModel, pose_set: PoseSet) -> None:
    text = format_pose_set(arm, pose_set)
    assert "pose_15" in text and "gravity_free_left_joint2" in text
    assert "condition number" in text and "    60.0    60.0" in text
    assert text.startswith("dual_openyam/left: 15 identification poses")


# Right arm


def test_right_arm_model_holds_only_the_right_chain(arm: ArmModel, right_arm: ArmModel) -> None:
    assert right_arm.joint_names == tuple(DUAL_OPENYAM_RIGHT_ARM_JOINTS)
    assert right_arm.fitted_links == tuple(
        link.replace("left", "right") for link in LEFT_FITTED_LINKS
    )
    assert right_arm.fitted_joint_ids == (2, 3, 4, 5, 6)
    assert right_arm.measured_rows == (1, 2, 3, 4)
    assert home_pose(right_arm).as_targets() == dict(
        zip(DUAL_OPENYAM_RIGHT_ARM_JOINTS, home_pose(arm).joints_rad, strict=True)
    )
    # Same angles, identical arm, mounted the same way: the same fingertip, each
    # measured from its own base.
    pose = np.radians([0.0, 90.0, 90.0, 0.0, 0.0, 0.0])
    assert np.allclose(arm.fingertip(pose), right_arm.fingertip(pose), atol=1e-9)
    assert arm.origin[1] > 0.0 > right_arm.origin[1]


def test_right_arm_poses_match_the_left_and_stay_on_their_side(
    right_arm: ArmModel, pose_set: PoseSet, right_pose_set: PoseSet
) -> None:
    assert right_pose_set.arm_id == "dual_openyam/right"
    assert [p.joints_deg for p in right_pose_set.identification] == [
        p.joints_deg for p in pose_set.identification
    ]
    assert right_pose_set.condition_number == pytest.approx(pose_set.condition_number)
    old = OldBenchChecks("right")
    limit = DUAL_OPENYAM_BENCH_BASE_SPACING_M / 2.0 + DUAL_OPENYAM_MIDLINE_MARGIN_M
    for pose in right_pose_set.identification:
        assert set(pose.as_targets()) == set(DUAL_OPENYAM_RIGHT_ARM_JOINTS)
        tip = right_arm.fingertip(pose.joints_rad)
        # y toward the midline from the right base, never past it by more than the margin.
        assert tip[1] <= limit
        assert right_arm.pose_ok(pose.joints_rad)
        assert old.pose_ok(pose.joints_rad), pose.name
    for joint, found in right_pose_set.gravity_free.items():
        assert joint.startswith("right_")
        row = right_arm.joint_names.index(joint)
        assert right_arm.gravity(right_arm.q(found.pose.joints_rad))[row] == pytest.approx(
            found.torque_nm, abs=1e-9
        )


def test_midline_keep_out_faces_the_other_arm(arm: ArmModel, right_arm: ArmModel) -> None:
    # The bench midline; the second keep-out, halfway between the URDF's bases, is looser.
    left_midline, left_model_midline = arm.workspace.keep_out
    right_midline, right_model_midline = right_arm.workspace.keep_out
    assert left_model_midline.offset_m == right_model_midline.offset_m > left_midline.offset_m
    limit = DUAL_OPENYAM_BENCH_BASE_SPACING_M / 2.0 + DUAL_OPENYAM_MIDLINE_MARGIN_M
    assert left_midline.offset_m == right_midline.offset_m == pytest.approx(limit)
    assert not left_midline.violated_by(np.array([0.3, -limit + 0.01, 0.3]))
    assert left_midline.violated_by(np.array([0.3, -limit - 0.01, 0.3]))
    assert not left_midline.violated_by(np.array([0.3, 0.5, 0.3]))  # away from the midline
    assert not right_midline.violated_by(np.array([0.3, limit - 0.01, 0.3]))
    assert right_midline.violated_by(np.array([0.3, limit + 0.01, 0.3]))
    # A yaw toward the midline is caught by pose_ok and by the transition check.
    reaching = np.radians([-90.0, 100.0, 120.0, 0.0, 0.0, 0.0])
    if not left_midline.violated_by(arm.fingertip(reaching)):
        reaching[0] = -reaching[0]
    assert left_midline.violated_by(arm.fingertip(reaching))
    assert not arm.pose_ok(reaching)
    assert not arm.transition_ok(home_pose(arm).joints_rad, reaching)


def test_keep_out_half_space_checks_every_point() -> None:
    space = HalfSpace(normal=(0.0, 0.0, -1.0), offset_m=-0.2, name="below 0.2 m")
    assert not space.violated_by(np.array([[0.0, 0.0, 0.3], [1.0, 1.0, 0.25]]))
    assert space.violated_by(np.array([[0.0, 0.0, 0.3], [1.0, 1.0, 0.1]]))


# A second robot: the Seeed B601-DM with the model-derived defaults


def test_b601_poses_follow_its_spec(b601_spec: ArmCalibrationSpec) -> None:
    arm = ArmModel(b601_spec)
    pose_set = generate_poses(arm, count=10)
    assert len(pose_set.identification) == 10 and pose_set.arm_id == "b601_dm"
    assert pose_set.rank == 8 and pose_set.condition_number < 30.0
    previous = home_pose(arm).joints_rad
    fixed = [i for i, name in enumerate(arm.joint_names) if name not in arm.measured_joints]
    for pose in pose_set.identification:
        assert arm.pose_ok(pose.joints_rad)
        assert arm.transition_ok(previous, pose.joints_rad)
        for joint, value in pose.as_targets().items():
            low, high = b601_spec.joint_ranges[joint]
            if joint in arm.measured_joints:
                assert low - 1e-12 <= value <= high + 1e-12
        assert all(pose.joints_rad[i] == arm.home[i] for i in fixed)
        tip, others = arm.points(arm.q(pose.joints_rad))
        assert min(tip[2], others[:, 2].min()) > arm.workspace.min_pose_z_m
        previous = pose.joints_rad
