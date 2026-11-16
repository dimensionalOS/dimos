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
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.robot.galaxea.r1pro.config import R1PRO_HOME_POSE


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
        weights = {1: 0.02, 2: 0.02, 3: 0.1, 4: 0.06}
        for joint_id, model_name in enumerate(configuration.model.names):
            name = model_name.split("/")[-1]
            if "_arm_joint" not in name or name not in R1PRO_HOME_POSE:
                continue
            weight = weights.get(int(name.rsplit("joint", 1)[1]), 0.0)
            joint = configuration.model.joints[joint_id]
            target[joint.idx_q] = R1PRO_HOME_POSE[name]
            costs[joint.idx_v] = self.config.posture_cost * weight
        preferred = pink.tasks.PostureTask(cost=costs)
        preferred.set_target(target)
        tasks["posture/preferred"] = preferred
        return tasks
