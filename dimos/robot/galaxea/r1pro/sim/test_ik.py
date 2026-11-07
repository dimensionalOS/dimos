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

"""Regressions with the pinned R1 Pro asset and real Pink solver."""

from io import BytesIO

import attrs
import numpy as np
from pink.exceptions import NoSolutionFound
import pytest
from yourdfpy import URDF

from dimos.control.tasks.pose_target_ik import PinkJointLimitError, PinkPoseTargetSolver
from dimos.control.tasks.teleop_ik_task.teleop_ik_task import create_task
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.sim.config import MODEL
from dimos.robot.galaxea.r1pro.sim.teleop import arm_tasks

pytestmark = pytest.mark.self_hosted


def test_observed_left_limit_overshoot_can_recover_without_relaxing_command_limits():
    task = create_task(arm_tasks()[0], {})
    config = task._solver._control_config
    # Captured from the stuck office demo; joint 3 is 1.54 mrad past its soft stop.
    measured = JointState(
        name=list(config.joint_names),
        position=[
            -1.51371888,
            1.14137962,
            -2.32282740,
            -1.86805263,
            1.20385503,
            -0.42562564,
            0.61995744,
        ],
    )
    target = task._solver.frame_poses(measured, config.target_frames)
    old = PinkPoseTargetSolver(attrs.evolve(config, feedback_limit_tolerance=0.001))
    with pytest.raises(PinkJointLimitError):
        old.step(target, measured, 0.02)
    initial = measured.position[2]
    for _ in range(150):
        command = task._solver.step(target, measured, 0.02)
        assert command is not None
        assert np.max(np.abs(np.asarray(command.position) - measured.position)) <= 0.010001
        measured = command
    lower = -2.321289415
    assert measured.position[2] > initial
    assert measured.position[2] >= lower + config.command_limit_margin - 1e-8
    bad = JointState(name=list(config.joint_names), position=list(measured.position))
    bad.position[2] = lower - 0.02
    with pytest.raises(PinkJointLimitError):
        task._solver.step(target, bad, 0.02)


@pytest.mark.parametrize("index,side", [(0, "left"), (1, "right")])
def test_arm_ik_uses_shoulder_frame_at_non_neutral_torso(index, side):
    robot = URDF.load(BytesIO(MODEL.load().xml.encode()), load_meshes=False)
    task = create_task(arm_tasks()[index], {})
    config = task._solver._control_config
    values = [0.0, 0.2 if side == "left" else -0.2, 0.0, -0.7, 0.0, 0.2, 0.0]
    robot.update_cfg(
        {
            **dict(zip(config.joint_names, values, strict=True)),
            "r1pro/torso_joint1": 0.4,
            "r1pro/torso_joint2": -0.7,
            "r1pro/torso_joint3": -0.3,
            "r1pro/torso_joint4": 0.5,
        }
    )
    frame = config.target_frames[0]
    expected = np.linalg.inv(
        robot.get_transform(config.robot_model.base_link)
    ) @ robot.get_transform(frame)
    actual = task._solver.frame_poses(
        JointState(name=list(config.joint_names), position=values), [frame]
    )[frame]
    np.testing.assert_allclose(actual.position.to_numpy(), expected[:3, 3], atol=1e-9)


def test_torso_height_path_keeps_shoulders_upright_and_over_same_base_position():
    robot = URDF.load(BytesIO(MODEL.load().xml.encode()), load_meshes=False)
    initial = robot.get_transform("r1pro/torso_link4").copy()
    for fold in np.linspace(0, 0.8, 30):
        values = [fold, -2 * fold, -fold, 0.0]
        robot.update_cfg({f"r1pro/torso_joint{i + 1}": q for i, q in enumerate(values)})
        actual = robot.get_transform("r1pro/torso_link4")
        np.testing.assert_allclose(actual[:3, :3], initial[:3, :3], atol=1e-9)
        np.testing.assert_allclose(actual[:2, 3], initial[:2, 3], atol=1e-9)
        assert initial[2, 3] - actual[2, 3] == pytest.approx(0.7 * (1 - np.cos(fold)))


def test_r1_feasible_step_does_not_freeze_on_proxqp_false_infeasibility():
    solver = create_task(arm_tasks()[0], {})._solver
    config = solver._control_config
    # Reproduced from the stuck demo's measured posture and a deterministic
    # controller-target sweep. The QP is feasible even with zero displacement.
    measured = JointState(
        name=list(config.joint_names),
        position=[
            -0.35129370285950956,
            -0.05516460530984676,
            -0.016090705160493858,
            -0.041510832821439324,
            -0.8124044389479712,
            -0.9473571967605983,
            1.0099650644674445,
        ],
    )
    previous = JointState(
        name=list(config.joint_names),
        position=[
            -0.40249909071863016,
            -0.026276303666692313,
            0.050654641813984554,
            -0.021958604035001675,
            -0.8645582835869707,
            -0.8243250451556747,
            0.9892983978007779,
        ],
    )
    target = PoseStamped(
        position=[-0.4107320555685976, -0.04406483810373035, -0.7854297536896317],
        orientation=[
            -0.28980548520668054,
            -0.5986006491729794,
            -0.7444727908057652,
            0.058739316506777994,
        ],
        frame_id=config.robot_model.base_pose.frame_id,
    )
    args = dict(
        robot_model=config.robot_model,
        frame_targets={config.target_frames[0]: target},
        controlled_joints=config.joint_names,
        command_state=previous,
        measured_state=measured,
        max_command_tracking_error_rad=np.deg2rad(config.max_command_tracking_error_deg),
        feedback_limit_tolerance=config.feedback_limit_tolerance,
        command_limit_margin=config.command_limit_margin,
        dt=0.02,
        max_joint_velocity_rad_s=config.max_joint_velocity_rad_s,
        joint_command_filter_cutoff_hz=config.joint_command_filter_cutoff_hz,
    )
    old = PinkPoseTargetSolver(
        attrs.evolve(config, pink=config.pink.model_copy(update={"solver": "proxqp"}))
    )
    with pytest.raises(NoSolutionFound) as failure:
        old._step_frame_targets(**args)
    assert np.all(failure.value.problem.h >= 0)  # zero step satisfies every bound
    result = solver._step_frame_targets(**args).command
    assert np.isfinite(result.position).all()
    assert np.max(np.abs(np.asarray(result.position) - previous.position)) <= 0.010001
