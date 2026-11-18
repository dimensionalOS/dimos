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

from dimos.robot.manipulators.calibration.gravity_poses import (
    FITTED_LINKS,
    GRAVITY_FREE_JOINTS,
    GRAVITY_FREE_LIMIT_NM,
    GRAVITY_PARAMS_PER_BODY,
    JOINT_BOXES_DEG,
    MAX_MIDLINE_CROSSING_M,
    MAX_MODEL_TORQUE_NM,
    MIN_POSE_X_M,
    MIN_POSE_Z_M,
    MIN_TRANSITION_Z_M,
    RIG_HALF_SPACING_M,
    ArmModel,
    PoseSet,
    body_joint_id,
    fitted_links,
    format_pose_set,
    generate_poses,
    home_pose,
    joint_number,
    measured_joints,
)
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)


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


def test_regressor_reproduces_pinocchio_gravity(arm: ArmModel) -> None:
    # Y(q) @ phi on the fitted columns equals the gravity torque of the fitted bodies
    # alone, i.e. the full model minus a copy with those bodies made massless.
    q = arm.full_q(home_pose().joints_rad)
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


def test_poses_stay_inside_the_joint_boxes(pose_set: PoseSet) -> None:
    for pose in pose_set.identification:
        for joint, value in zip(DUAL_OPENYAM_LEFT_ARM_JOINTS, pose.joints_deg, strict=True):
            low, high = JOINT_BOXES_DEG[joint_number(joint)]
            assert low - 1e-9 <= value <= high + 1e-9, (pose.name, joint, value)
        assert pose.joints_deg[0] == 0.0 and pose.joints_deg[5] == 0.0


def test_poses_clear_the_table_and_sit_in_front(arm: ArmModel, pose_set: PoseSet) -> None:
    for pose in pose_set.identification:
        q = arm.full_q(pose.joints_rad)
        tip, others = arm.points(q)
        assert tip[2] > MIN_POSE_Z_M and tip[0] > MIN_POSE_X_M, pose.name
        assert (others[:, 2] > MIN_POSE_Z_M).all(), pose.name
        assert np.abs(arm.gravity(q)).max() <= MAX_MODEL_TORQUE_NM, pose.name


def test_poses_are_reachable_in_sequence_from_home(arm: ArmModel, pose_set: PoseSet) -> None:
    previous = home_pose().joints_rad
    for pose in pose_set.identification:
        assert arm.transition_ok(previous, pose.joints_rad), pose.name
        # The interpolation check is what guards the table: sample it densely too.
        a, b = np.asarray(previous), np.asarray(pose.joints_rad)
        for s in np.linspace(0.0, 1.0, 101):
            tip, others = arm.points(arm.full_q(a + s * (b - a)))
            assert tip[2] > MIN_TRANSITION_Z_M and (others[:, 2] > MIN_TRANSITION_Z_M).all()
        previous = pose.joints_rad


def test_poses_are_distinct_and_vary_every_measured_joint(pose_set: PoseSet) -> None:
    angles = np.asarray([p.joints_deg for p in pose_set.identification])
    assert len({tuple(np.round(row, 3)) for row in angles}) == len(angles)
    for column in (1, 2, 3, 4):
        assert angles[:, column].max() - angles[:, column].min() > 60.0


def test_regressor_is_well_conditioned_on_its_identifiable_directions(pose_set: PoseSet) -> None:
    assert pose_set.rank == 8
    assert len(pose_set.singular_values) == pose_set.rank
    assert math.isfinite(pose_set.condition_number)
    assert pose_set.condition_number < 20.0


def test_gravity_free_poses_null_or_minimise_their_joint_torque(
    arm: ArmModel, pose_set: PoseSet
) -> None:
    assert set(pose_set.gravity_free) == set(GRAVITY_FREE_JOINTS)
    for joint, found in pose_set.gravity_free.items():
        row = DUAL_OPENYAM_LEFT_ARM_JOINTS.index(joint)
        q = arm.full_q(found.pose.joints_rad)
        torque = arm.gravity(q)[row]
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


def test_body_joint_id_requires_a_link_frame_on_its_joint(arm: ArmModel) -> None:
    assert [body_joint_id(arm.model, link) for link in FITTED_LINKS] == [2, 3, 4, 5, 6]
    with pytest.raises(ValueError, match="coincident"):
        body_joint_id(arm.model, "left_grasp_frame")
    with pytest.raises(ValueError, match="no frame"):
        body_joint_id(arm.model, "left_link9")
    assert len(arm.param_columns) == len(FITTED_LINKS) * GRAVITY_PARAMS_PER_BODY


def test_format_lists_every_pose_in_degrees(arm: ArmModel, pose_set: PoseSet) -> None:
    text = format_pose_set(arm, pose_set)
    assert "pose_15" in text and "gravity_free_left_joint2" in text
    assert "condition number" in text and "60.0    60.0" in text


# Right arm


def test_side_helpers_name_the_right_chain() -> None:
    assert fitted_links("right") == (
        "right_link2",
        "right_link3",
        "right_link4",
        "right_link5",
        "right_gripper",
    )
    assert measured_joints("right") == tuple(DUAL_OPENYAM_RIGHT_ARM_JOINTS[1:5])
    assert home_pose("right").as_targets() == dict(
        zip(DUAL_OPENYAM_RIGHT_ARM_JOINTS, home_pose("left").joints_rad, strict=True)
    )
    with pytest.raises(ValueError, match="side must be"):
        ArmModel(side="middle")


def test_right_arm_model_moves_only_the_right_chain(arm: ArmModel, right_arm: ArmModel) -> None:
    assert right_arm.offset == 6 and arm.offset == 0
    assert right_arm.fitted_joint_ids == (8, 9, 10, 11, 12)
    assert right_arm.measured_rows == (7, 8, 9, 10)
    pose = np.radians([0.0, 90.0, 90.0, 0.0, 0.0, 0.0])
    q = right_arm.full_q(pose)
    assert np.allclose(q[:6], arm.home[:6]) and np.allclose(q[6:], pose)
    # Same angles, identical arm, mounted the same way: same fingertip up to the base offset.
    tip_left, _ = arm.points(arm.full_q(pose))
    tip_right, _ = right_arm.points(q)
    assert np.allclose(tip_left - tip_right, [0.0, arm.base_y - right_arm.base_y, 0.0], atol=1e-9)
    assert arm.base_y > 0.0 > right_arm.base_y


def test_right_arm_poses_match_the_left_and_stay_on_their_side(
    right_arm: ArmModel, pose_set: PoseSet, right_pose_set: PoseSet
) -> None:
    assert right_pose_set.side == "right"
    assert [p.joints_deg for p in right_pose_set.identification] == [
        p.joints_deg for p in pose_set.identification
    ]
    assert right_pose_set.condition_number == pytest.approx(pose_set.condition_number)
    for pose in right_pose_set.identification:
        assert pose.side == "right"
        assert set(pose.as_targets()) == set(DUAL_OPENYAM_RIGHT_ARM_JOINTS)
        tip, _ = right_arm.points(right_arm.full_q(pose.joints_rad))
        # y toward the midline from the right base, never past it by more than the margin.
        assert tip[1] - right_arm.base_y <= RIG_HALF_SPACING_M + MAX_MIDLINE_CROSSING_M
        assert right_arm.pose_ok(pose.joints_rad)
    for joint, found in right_pose_set.gravity_free.items():
        assert joint.startswith("right_") and found.pose.side == "right"
        assert right_arm.side_gravity(right_arm.full_q(found.pose.joints_rad))[
            joint_number(joint) - 1
        ] == pytest.approx(found.torque_nm, abs=1e-9)


def test_midline_check_uses_each_arms_own_base(arm: ArmModel, right_arm: ArmModel) -> None:
    limit = RIG_HALF_SPACING_M + MAX_MIDLINE_CROSSING_M
    assert arm.midline_ok(np.array([0.3, arm.base_y - limit + 0.01, 0.3]))
    assert not arm.midline_ok(np.array([0.3, arm.base_y - limit - 0.01, 0.3]))
    assert arm.midline_ok(np.array([0.3, arm.base_y + 0.5, 0.3]))  # away from the midline
    assert right_arm.midline_ok(np.array([0.3, right_arm.base_y + limit - 0.01, 0.3]))
    assert not right_arm.midline_ok(np.array([0.3, right_arm.base_y + limit + 0.01, 0.3]))
    # A yaw toward the midline is caught by pose_ok and by the transition check.
    reaching = np.radians([-90.0, 100.0, 120.0, 0.0, 0.0, 0.0])
    tip, _ = arm.points(arm.full_q(reaching))
    if not arm.midline_ok(tip):
        assert not arm.pose_ok(reaching)
        assert not arm.transition_ok(home_pose().joints_rad, reaching)
    else:
        reaching[0] = -reaching[0]
        assert not arm.pose_ok(reaching)
        assert not arm.transition_ok(home_pose().joints_rad, reaching)
