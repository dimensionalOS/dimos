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

"""Exercise the real Pink QP on a small, local named-joint model."""

import numpy as np
import pink
import pinocchio
import pytest

from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.robot.galaxea.r1pro.posture_ik import (
    MAX_TORSO_PITCH,
    MIN_ELBOW_FLEXION,
    NOMINAL_POSTURE,
    POSTURE_APPROACH_RAD_S,
    POSTURE_TASK,
    R1ProPostureIK,
    R1ProPostureLimit,
    posture_error,
    posture_is_valid,
)


@pytest.fixture
def configuration():
    model = pinocchio.Model()
    # Deliberately reverse model order: the policy must map names, not assume
    # the vendor URDF's traversal order or confuse q and velocity coordinates.
    for name in reversed(UPPER_BODY_JOINTS):
        model.addJoint(
            0, pinocchio.JointModelRY(), pinocchio.SE3.Identity(), coordinator_name(name)
        )
    model.lowerPositionLimit[:] = -3
    model.upperPositionLimit[:] = 3
    model.velocityLimit[:] = 10
    q = np.zeros(model.nq)
    limit = R1ProPostureLimit(model)
    q[limit.qids] = NOMINAL_POSTURE
    return pink.Configuration(model, model.createData(), q)


def test_neutral_posture_pull_is_bounded_and_not_anchored_to_current_pose(configuration):
    solver = R1ProPostureIK(PinkKinematicsConfig(posture_cost=0.2))
    limit = R1ProPostureLimit(configuration.model)
    q = configuration.q.copy()
    q[limit.qids[5]] = 0.8
    configuration.update(q)
    tasks = solver._build_task_stack(configuration, ())
    solver._step_configuration(configuration, tasks, 0.05)

    step = POSTURE_APPROACH_RAD_S * 0.05
    assert configuration.q[limit.qids[5]] == pytest.approx(0.8 - step, abs=1e-6)
    assert tasks[POSTURE_TASK].target_q[limit.qids[5]] == pytest.approx(0.8 - step)
    assert posture_is_valid(configuration.q[limit.qids])


def test_hard_envelope_beats_a_stronger_conflicting_qp_objective(configuration):
    solver = R1ProPostureIK(PinkKinematicsConfig(posture_cost=0.2))
    limit = R1ProPostureLimit(configuration.model)
    tasks = solver._build_task_stack(configuration, ())
    demand = pink.tasks.PostureTask(cost=1000.0)
    target = configuration.q.copy()
    target[limit.qids[0]] = 2.0
    target[limit.qids[5]] = 2.0
    target[limit.qids[7]] = 0.0
    demand.set_target(target)
    tasks = dict(tasks, demand=demand)

    for _ in range(5):
        solver._step_configuration(configuration, tasks, 0.05)
        assert posture_is_valid(configuration.q[limit.qids])

    joints = configuration.q[limit.qids]
    assert joints[0] + joints[1] - joints[2] == pytest.approx(MAX_TORSO_PITCH, abs=1e-5)
    assert joints[5] == pytest.approx(np.pi / 2, abs=1e-5)
    assert joints[7] == pytest.approx(-MIN_ELBOW_FLEXION, abs=1e-5)


def test_neutral_objective_respects_locked_inactive_joints(configuration):
    solver = R1ProPostureIK(
        PinkKinematicsConfig(posture_cost=0.2, solver_kwargs={"eps_abs": 1e-9, "eps_rel": 1e-9})
    )
    limit = R1ProPostureLimit(configuration.model)
    q = configuration.q.copy()
    q[limit.qids[5]] = q[limit.qids[12]] = 0.5
    configuration.update(q)
    tasks = solver._build_task_stack(configuration, ())
    matrix = np.zeros((1, configuration.model.nv))
    joint = configuration.model.joints[
        configuration.model.getJointId(coordinator_name("right_arm_joint2"))
    ]
    matrix[0, joint.idx_v] = 1
    lock = pink.tasks.LinearHolonomicTask(A=matrix, b=np.zeros(1), q_0=q)

    solver._step_configuration(configuration, tasks, 0.05, constraints=(lock,))

    assert configuration.q[limit.qids[12]] == pytest.approx(0.5, abs=1e-8)
    assert configuration.q[limit.qids[5]] < 0.5


def test_saved_failed_grasp_is_rejected_even_without_an_ik_step():
    # Recorded selected pregrasp, seed 2012159698, object_5, left hand.
    failed = np.array(
        [
            0.91239579,
            -1.04018411,
            0.37860947,
            0.31023350,
            0.16318999,
            0.74246748,
            -0.94410461,
            -0.86019374,
            0.88909621,
            -0.62861661,
            -0.47419764,
            -0.19217366,
            -0.83851721,
            0.61526398,
            -1.95128640,
            -2.04494626,
            0.85983700,
            -1.33079792,
        ]
    )
    assert not posture_is_valid(failed)
    assert posture_is_valid(NOMINAL_POSTURE)
    assert posture_error(failed, "left") > posture_error(NOMINAL_POSTURE, "left")
