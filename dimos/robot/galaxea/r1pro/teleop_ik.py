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


class R1ProPinkPoseTargetSolver(PinkPoseTargetSolver):
    """Prefer arm motion strongly while retaining slow waist redundancy."""

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)
        posture = tasks.get("posture/current")
        if posture is None:
            raise ValueError("R1Pro teleop requires a positive posture cost")
        weights = np.ones(configuration.model.nv, dtype=np.float64)
        for joint_id, name in enumerate(configuration.model.names):
            if name.split("/")[-1] == "torso_joint3":
                joint = configuration.model.joints[joint_id]
                # Only pitch is reluctant: yaw and arms retain their original cost.
                weights[joint.idx_v : joint.idx_v + joint.nv] = 5.0
        posture.cost = self.config.posture_cost * weights
        return tasks
