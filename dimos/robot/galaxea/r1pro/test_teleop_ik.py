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

"""R1Pro posture preference and tracking with an explicitly positioned torso."""

import numpy as np
import pytest

from dimos.control.tasks.pose_target_ik import PoseTargetIKTaskConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop import R1PRO_VR_TARGET_FRAMES
from dimos.robot.galaxea.r1pro.config import R1PRO_HOME_POSE
from dimos.robot.galaxea.r1pro.joylo import TORSO, UPPER
from dimos.robot.galaxea.r1pro.teleop_coordinator import shared_teleop_tasks
from dimos.robot.galaxea.r1pro.teleop_ik import R1ProPinkPoseTargetSolver
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix


def test_elbow_posture_improves_without_moving_grippers_or_torso():
    task = next(t for t in shared_teleop_tasks(dict.fromkeys(UPPER, 1.0)) if t.type == "teleop_ik")
    params = {
        key: value for key, value in task.params.items() if key not in ("bindings", "solver_type")
    }
    frames = tuple(R1PRO_VR_TARGET_FRAMES.values())
    solver = R1ProPinkPoseTargetSolver(
        PoseTargetIKTaskConfig(joint_names=tuple(task.joint_names), target_frames=frames, **params)
    )
    positions = {name: R1PRO_HOME_POSE[name.split("/")[-1]] for name in UPPER}
    positions.update(zip(TORSO, (0.4, -0.6, -0.2, 0.15), strict=True))
    positions["r1pro/left_arm_joint3"] += 0.5
    initial = positions.copy()

    def feedback():
        return JointState(name=list(positions), position=list(positions.values()))

    targets = solver.frame_poses(feedback(), frames)
    for _ in range(400):
        command = solver.step(targets, feedback(), 0.01)
        assert not set(command.name) & set(TORSO)
        positions.update(zip(command.name, command.position, strict=True))
    actual = solver.frame_poses(feedback(), frames)
    for frame in frames:
        assert actual[frame].position.to_list() == pytest.approx(
            targets[frame].position.to_list(), abs=0.002
        )
        assert actual[frame].orientation.angle_to(targets[frame].orientation) < 0.02
    joint = "r1pro/left_arm_joint3"
    home = R1PRO_HOME_POSE["left_arm_joint3"]
    assert abs(positions[joint] - home) < abs(initial[joint] - home) * 0.8
    assert [positions[n] for n in TORSO] == [initial[n] for n in TORSO]


def test_lowering_and_tilting_torso_carries_grippers_without_lifting_arms():
    task = next(t for t in shared_teleop_tasks(dict.fromkeys(UPPER, 1.0)) if t.type == "teleop_ik")
    params = {
        key: value for key, value in task.params.items() if key not in ("bindings", "solver_type")
    }
    frames = tuple(R1PRO_VR_TARGET_FRAMES.values())
    solver = R1ProPinkPoseTargetSolver(
        PoseTargetIKTaskConfig(joint_names=tuple(task.joint_names), target_frames=frames, **params)
    )
    positions = {name: R1PRO_HOME_POSE[name.split("/")[-1]] for name in UPPER}
    neutral = solver.frame_poses(
        JointState(name=list(positions), position=list(positions.values())), frames
    )
    joints = {joint.name: joint for joint in params["robot_model"].model.load().joints}
    body_zero = np.eye(4)
    for name in TORSO:
        body_zero = body_zero @ joints[name].parent_from_child(0)
    # The same hand posture while the body lowers and pitches. Its targets are carried with it.
    for index in range(1, 201):
        fraction = index / 200
        positions.update(
            zip(TORSO, (0.4 * fraction, -0.8 * fraction, -0.2 * fraction, 0), strict=True)
        )
        body = np.eye(4)
        for name in TORSO:
            body = body @ joints[name].parent_from_child(positions[name])
        motion = body @ np.linalg.inv(body_zero)
        targets = {}
        for frame in frames:
            pose = matrix_to_pose(motion @ pose_to_matrix(neutral[frame]))
            targets[frame] = PoseStamped(position=pose.position, orientation=pose.orientation)
        command = solver.step(
            targets, JointState(name=list(positions), position=list(positions.values())), 0.01
        )
        positions.update(zip(command.name, command.position, strict=True))
    actual = solver.frame_poses(
        JointState(name=list(positions), position=list(positions.values())), frames
    )
    for frame in frames:
        assert actual[frame].position.to_list() == pytest.approx(
            targets[frame].position.to_list(), abs=0.002
        )
        assert actual[frame].orientation.angle_to(targets[frame].orientation) < 0.02


def test_elbow_soft_stop_overtravel_recovers_with_a_legal_command():
    task = next(t for t in shared_teleop_tasks(dict.fromkeys(UPPER, 1.0)) if t.type == "teleop_ik")
    params = {
        key: value for key, value in task.params.items() if key not in ("bindings", "solver_type")
    }
    frames = tuple(R1PRO_VR_TARGET_FRAMES.values())
    solver = R1ProPinkPoseTargetSolver(
        PoseTargetIKTaskConfig(joint_names=tuple(task.joint_names), target_frames=frames, **params)
    )
    positions = {name: R1PRO_HOME_POSE[name.split("/")[-1]] for name in UPPER}
    elbow = "r1pro/right_arm_joint4"
    lower = params["robot_model"].model.load().get_joint(elbow).lower
    positions[elbow] = lower - 0.012  # Reproduces the observed 0.7-degree soft-stop drift.
    measured = JointState(name=list(positions), position=list(positions.values()))
    targets = solver.frame_poses(measured, frames)
    command = solver.step(targets, measured, 0.01)
    recovered = dict(zip(command.name, command.position, strict=True))[elbow]
    assert lower <= recovered < lower + 0.02
