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

"""Objective tests for single-arm OpenYAM Quest teleoperation."""

import pytest

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver, PoseTargetIKTaskConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.blueprints.teleop import _openyam_quest_task
from dimos.robot.manipulators.openyam.config import OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.openyam.teleop_ik import OpenYamPinkPoseTargetSolver

_SAFE_POSTURE = [0.0, 1.047, 1.047, 0.0, 0.0, 0.0]
_TARGET_FRAME = _openyam_quest_task.params["robot_model"].end_effector_link


def _solver(
    solver_type: type[PinkPoseTargetSolver] = OpenYamPinkPoseTargetSolver,
) -> PinkPoseTargetSolver:
    task = _openyam_quest_task
    config = PoseTargetIKTaskConfig(
        joint_names=tuple(task.joint_names),
        robot_model=task.params["robot_model"],
        target_frames=(_TARGET_FRAME,),
        pink=task.params["pink"],
        max_joint_velocity_rad_s=task.params["max_joint_velocity_rad_s"],
        joint_command_filter_cutoff_hz=task.params["joint_command_filter_cutoff_hz"],
    )
    return solver_type(config)


@pytest.mark.self_hosted
def test_solver_weights_large_joints_above_wrist_joints() -> None:
    solver = _solver()
    state = JointState(name=OPENYAM_ARM_JOINTS, position=_SAFE_POSTURE)
    targets = solver.frame_poses(state, (_TARGET_FRAME,))

    assert solver.step(targets, state, 0.01) is not None

    context = next(iter(solver._control_contexts.values()))
    assert context.tasks is not None
    posture = context.tasks["posture/current"]
    assert posture.cost == pytest.approx([3.0, 3.0, 3.0, 0.01, 0.01, 0.01])


def _orientation_target_motion(
    solver_type: type[PinkPoseTargetSolver],
) -> tuple[float, float]:
    solver = _solver(solver_type)
    initial = JointState(name=OPENYAM_ARM_JOINTS, position=_SAFE_POSTURE)
    pose = solver.frame_poses(initial, (_TARGET_FRAME,))[_TARGET_FRAME]
    target = PoseStamped(
        frame_id=pose.frame_id,
        position=pose.position,
        orientation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)) * pose.orientation,
    )
    state = initial
    for _ in range(10):
        command = solver.step({_TARGET_FRAME: target}, state, 0.01)
        assert command is not None
        state = command
    motion = [
        abs(commanded - start)
        for commanded, start in zip(state.position, initial.position, strict=True)
    ]
    return sum(motion[:3]), sum(motion[3:])


@pytest.mark.self_hosted
def test_weighted_solver_shifts_orientation_motion_toward_wrist() -> None:
    generic_proximal, generic_wrist = _orientation_target_motion(PinkPoseTargetSolver)
    weighted_proximal, weighted_wrist = _orientation_target_motion(OpenYamPinkPoseTargetSolver)

    assert weighted_proximal < generic_proximal * 0.75
    assert weighted_wrist > generic_wrist * 0.9
