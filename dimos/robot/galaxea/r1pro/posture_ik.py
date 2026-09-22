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

"""R1Pro planning posture: teleop's neutral QP objective plus a hard envelope.

The neutral pose, joint weights and bounded pull come from
origin/mustafa/task/r1pro-hosted-teleop-demo's teleop_ik.py. The envelope is a
conservative demo policy, not a claim of general human ergonomics.
"""

from collections.abc import Mapping, Sequence

import numpy as np
from numpy.typing import NDArray
import pink
from pink.limits import Limit
import pinocchio

from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name

POSTURE_TASK = "posture/current"
NOMINAL_POSTURE = np.array([0.0] * 4 + [0.0, 0.0, 0.0, -np.pi / 2, 0.0, 0.0, 0.0] * 2)
POSTURE_WEIGHTS = np.array([4.0] * 4 + [2.0, 3.0, 3.0, 3.0, 0.3, 0.3, 0.1] * 2)
POSTURE_APPROACH_RAD_S = 0.15
MAX_TORSO_PITCH = np.deg2rad(20.0)
MAX_SHOULDER_SWIVEL = np.deg2rad(90.0)
MIN_ELBOW_FLEXION = np.deg2rad(10.0)
POSTURE_TOLERANCE = 1e-6
POSTURE_RANK_WEIGHT = 0.5


def posture_inequalities() -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """A q <= b in UPPER_BODY_JOINTS order; wrists retain mechanical limits."""
    rows, bounds = [], []
    pitch = np.zeros(len(UPPER_BODY_JOINTS))
    # The first two torso axes are +Y; joint 3 is -Y, before torso yaw.
    pitch[:3] = [1, 1, -1]
    rows.extend([pitch, -pitch])
    bounds.extend([MAX_TORSO_PITCH, MAX_TORSO_PITCH])
    for side in ("left", "right"):
        for number in (2, 3):
            row = np.zeros(len(UPPER_BODY_JOINTS))
            row[UPPER_BODY_JOINTS.index(f"{side}_arm_joint{number}")] = 1
            rows.extend([row, -row])
            bounds.extend([MAX_SHOULDER_SWIVEL, MAX_SHOULDER_SWIVEL])
        row = np.zeros(len(UPPER_BODY_JOINTS))
        row[UPPER_BODY_JOINTS.index(f"{side}_arm_joint4")] = 1
        rows.append(row)
        bounds.append(-MIN_ELBOW_FLEXION)
    return np.asarray(rows), np.asarray(bounds)


POSTURE_MATRIX, POSTURE_BOUND = posture_inequalities()


def posture_is_valid(positions: NDArray[np.float64]) -> bool:
    """Also validate endpoints/sweeps: IK can return without taking a QP step."""
    joints = positions[: len(UPPER_BODY_JOINTS)]
    return bool(
        joints.shape == NOMINAL_POSTURE.shape
        and np.isfinite(joints).all()
        and np.all(POSTURE_MATRIX @ joints <= POSTURE_BOUND + POSTURE_TOLERANCE)
    )


def posture_error(positions: NDArray[np.float64], arm: str) -> float:
    """Weighted RMS neutral error for the torso and selected arm, in radians."""
    start = 4 if arm == "left" else 11
    columns = np.r_[0:4, start : start + 7]
    weights = POSTURE_WEIGHTS[columns]
    return float(
        np.linalg.norm(weights * (positions[columns] - NOMINAL_POSTURE[columns]))
        / np.linalg.norm(weights)
    )


class R1ProPostureLimit:
    """Exact affine joint inequalities on the QP displacement, not a soft task."""

    def __init__(self, model: pinocchio.Model) -> None:
        self.qids, vids = [], []
        for joint_name in UPPER_BODY_JOINTS:
            name = coordinator_name(joint_name)
            if not model.existJointName(name):
                raise ValueError(f"R1Pro posture model is missing {name}")
            joint = model.joints[model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f"R1Pro posture requires scalar revolute joint {name}")
            self.qids.append(joint.idx_q)
            vids.append(joint.idx_v)
        self.matrix = np.zeros((len(POSTURE_BOUND), model.nv))
        self.matrix[:, vids] = POSTURE_MATRIX

    def compute_qp_inequalities(
        self, configuration: pink.Configuration, dt: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.matrix, POSTURE_BOUND - POSTURE_MATRIX @ configuration.q[self.qids]


class R1ProPostureIK(PinkIK):
    """Prefer the upright, elbows-bent teleop reference during grasp planning."""

    def __init__(self, config: PinkKinematicsConfig) -> None:
        super().__init__(config)
        self._posture_limit: R1ProPostureLimit | None = None

    def _create_tasks(
        self, configuration: pink.Configuration, target_frames: tuple[str, ...]
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)
        task = tasks.get(POSTURE_TASK)
        if not isinstance(task, pink.tasks.PostureTask):
            raise ValueError("R1ProPostureIK requires a positive posture cost")
        if self._posture_limit is None:
            self._posture_limit = R1ProPostureLimit(configuration.model)
        weights = np.zeros(configuration.model.nv)
        for name, weight in zip(UPPER_BODY_JOINTS, POSTURE_WEIGHTS, strict=True):
            joint = configuration.model.joints[
                configuration.model.getJointId(coordinator_name(name))
            ]
            weights[joint.idx_v] = weight
        task.cost = self.config.posture_cost * weights
        return tasks

    def _update_current_posture_target(
        self, tasks: Mapping[str, pink.Task], configuration: pink.Configuration
    ) -> None:
        self._before_solve(tasks, configuration, self.config.dt)

    def _before_solve(
        self, tasks: Mapping[str, pink.Task], configuration: pink.Configuration, dt: float
    ) -> None:
        task = tasks[POSTURE_TASK]
        assert isinstance(task, pink.tasks.PostureTask)
        assert self._posture_limit is not None
        qids = self._posture_limit.qids
        target = configuration.q.copy()
        step = POSTURE_APPROACH_RAD_S * dt / task.gain
        target[qids] += np.clip(NOMINAL_POSTURE - target[qids], -step, step)
        task.set_target(target)

    def _qp_limits(self, configuration: pink.Configuration) -> Sequence[Limit]:
        assert self._posture_limit is not None
        return (
            configuration.model.configuration_limit,
            configuration.model.velocity_limit,
            self._posture_limit,
        )
