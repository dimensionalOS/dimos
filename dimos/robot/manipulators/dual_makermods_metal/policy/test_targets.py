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

"""Absolute dual-hand goals and the WebXR relative-to-absolute mapping."""

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.targets import (
    ControllerPose,
    DualHandGoal,
    GoalError,
    HandGoal,
    WebXRGoalMapper,
    quaternion_xyzw_to_matrix,
)

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
QUARTER_Z = quaternion_xyzw_to_matrix((0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)))


def _goal() -> DualHandGoal:
    return DualHandGoal(
        HandGoal((0.4, -0.2, 0.3), IDENTITY),
        HandGoal((0.4, 0.2, 0.3), IDENTITY),
    )


def test_pose24_round_trips_a_proper_rotation():
    # Non-unit quaternions are normalized; a quarter yaw is right-handed.
    assert np.allclose(quaternion_xyzw_to_matrix((0.0, 0.0, 0.0, 2.0)), np.eye(3))
    assert np.allclose(QUARTER_Z @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)

    goal = DualHandGoal(
        HandGoal((0.4, -0.2, 0.3), tuple(map(tuple, QUARTER_Z))),
        HandGoal((0.4, 0.2, 0.3), IDENTITY),
    )
    recovered = DualHandGoal.from_pose24(goal.to_pose24())
    assert np.allclose(recovered.left.matrix, goal.left.matrix)
    assert np.allclose(recovered.right.matrix, goal.right.matrix)
    assert recovered.left.position == goal.left.position


def test_pose24_rejects_invalid_rotations_and_non_finite():
    pose = _goal().to_pose24()
    pose[6:15] = 0.5
    with pytest.raises(GoalError, match="orthonormal"):
        DualHandGoal.from_pose24(pose)
    pose = _goal().to_pose24()
    pose[6:15] = np.diag([1.0, 1.0, -1.0]).reshape(-1)
    with pytest.raises(GoalError, match="determinant"):
        DualHandGoal.from_pose24(pose)
    pose = _goal().to_pose24()
    pose[0] = np.nan
    with pytest.raises(GoalError, match="non-finite"):
        DualHandGoal.from_pose24(pose)


def test_offset_and_hand_goal_validation():
    moved = _goal().offset("left", (0.0, -0.1, 0.02))
    assert np.allclose(moved.left.position, (0.4, -0.3, 0.32))
    assert moved.right == _goal().right
    with pytest.raises(GoalError, match="3-vector"):
        HandGoal((0.0, 0.0), IDENTITY)
    with pytest.raises(GoalError, match="3-vector"):
        HandGoal((0.0, 0.0, np.inf), IDENTITY)


def test_webxr_mapping_requires_engagement_then_tracks_relative_motion():
    mapper = WebXRGoalMapper()
    left = ControllerPose.from_pose((0.0, -0.2, 0.5), (0.0, 0.0, 0.0, 1.0))
    right = ControllerPose.from_pose((0.0, 0.2, 0.5), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(GoalError, match="not engaged"):
        mapper.update(left, right)
    mapper.engage(_goal(), left, right)
    assert np.allclose(mapper.update(left, right).to_pose24(), _goal().to_pose24())
    moved_left = ControllerPose.from_pose((0.1, -0.2, 0.5), (0.0, 0.0, 0.0, 1.0))
    shifted = mapper.update(moved_left, right)
    assert np.allclose(shifted.left.position, (0.5, -0.2, 0.3))
    assert np.allclose(shifted.right.position, (0.4, 0.2, 0.3))
    mapper.release()
    with pytest.raises(GoalError, match="not engaged"):
        mapper.update(left, right)


def test_webxr_rotation_is_relative_to_engagement_not_absolute():
    mapper = WebXRGoalMapper()
    identity = ControllerPose.from_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    turned = ControllerPose(
        (0.0, 0.0, 0.0),
        tuple(
            map(tuple, quaternion_xyzw_to_matrix((0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))))
        ),
    )
    mapper.engage(_goal(), identity, identity)
    rotated = mapper.update(turned, turned)
    assert np.allclose(rotated.left.matrix, QUARTER_Z)
    assert np.allclose(rotated.right.matrix, QUARTER_Z)
