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

"""OpenYAM-specific Pink objective tuning for Quest teleoperation."""

import numpy as np
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver

# OpenYAM has six joints for a six-DoF frame objective, so modest posture
# ratios have almost no visible effect. These factors yield effective costs of
# 3.0 for the large proximal joints and 0.01 for the wrist at the blueprint's
# 0.01 base posture cost.
_POSTURE_WEIGHTS = np.array([300.0, 300.0, 300.0, 1.0, 1.0, 1.0], dtype=np.float64)


class OpenYamPinkPoseTargetSolver(PinkPoseTargetSolver):
    """Prefer wrist motion while stabilizing the larger proximal joints."""

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)
        posture_task = tasks.get("posture/current")
        if posture_task is None:
            raise ValueError("OpenYamPinkPoseTargetSolver requires a positive posture cost")
        posture_task.cost = self.config.posture_cost * _POSTURE_WEIGHTS
        return tasks
