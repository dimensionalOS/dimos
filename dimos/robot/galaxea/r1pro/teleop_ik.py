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

"""R1Pro teleop redundancy tuning, separate from arm tracking gains."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.robot.galaxea.r1pro.config import R1PRO_HOME_POSE


class _HandNullspacePosture(pink.tasks.PostureTask):  # type: ignore[misc]  # Pink is untyped.
    """Apply posture preference only in directions that preserve the hand tasks.

    Include locked coordinates in the projector so a torso/base degree of freedom
    cannot be used to compensate for posture motion in an arm-only solve.
    """

    def __init__(
        self,
        cost: NDArray[np.float64],
        frames: tuple[pink.tasks.FrameTask, ...],
        locked: NDArray[np.float64],
    ) -> None:
        super().__init__(cost=cost, gain=0.1)
        self.frames = frames
        self.locked = locked

    def compute_qp_objective(
        self, configuration: pink.Configuration
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        hessian, gradient = super().compute_qp_objective(configuration)
        jacobian = np.vstack(
            [*(frame.compute_jacobian(configuration) for frame in self.frames), self.locked]
        )
        nullspace = np.eye(configuration.model.nv) - np.linalg.pinv(jacobian, rcond=1e-6) @ jacobian
        return nullspace.T @ hessian @ nullspace, nullspace.T @ gradient


class R1ProPinkPoseTargetSolver(PinkPoseTargetSolver):
    """Track the hands with a soft preference for the normal shoulder/elbow posture."""

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)
        if self.config.posture_cost <= 0:
            raise ValueError("R1Pro teleop requires a positive posture cost")
        # Unlike posture/current, this reference never follows the solution. A
        # fixed reference prevents redundant shoulder/elbow angles from drifting
        # on repeated reaches. Wrists remain free to match the operator's rotation.
        target = configuration.q.copy()
        costs = np.zeros(configuration.model.nv, dtype=np.float64)
        # A projected preference can stabilize the redundant posture without
        # sacrificing hand tracking to a competing joint-space objective.
        weights = {1: 0.02, 2: 0.02, 3: 1.0, 4: 2.0}
        for joint_id, model_name in enumerate(configuration.model.names):
            name = model_name.split("/")[-1]
            if "_arm_joint" not in name or name not in R1PRO_HOME_POSE:
                continue
            weight = weights.get(int(name.rsplit("joint", 1)[1]), 0.0)
            joint = configuration.model.joints[joint_id]
            target[joint.idx_q] = R1PRO_HOME_POSE[name]
            costs[joint.idx_v] = self.config.posture_cost * weight
        controlled = {
            configuration.model.joints[joint_id].idx_v
            for joint_id, name in enumerate(configuration.model.names)
            if name in self._control_config.joint_names
        }
        locked = np.eye(configuration.model.nv)[
            [i for i in range(configuration.model.nv) if i not in controlled]
        ]
        preferred = _HandNullspacePosture(
            costs,
            tuple(task for task in tasks.values() if isinstance(task, pink.tasks.FrameTask)),
            locked,
        )
        preferred.set_target(target)
        tasks["posture/preferred"] = preferred
        return tasks
