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

"""R1 Pro whole-upper-body Pink solver for Quest teleoperation."""

from __future__ import annotations

import numpy as np
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver


class R1ProPinkPoseTargetSolver(PinkPoseTargetSolver):
    """Limit the headset target to forward/vertical motion and pitch/yaw."""

    HEAD_FRAME = "head_link"

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)
        head_task = tasks[f"frame/{self.HEAD_FRAME}"]
        head_task.set_position_cost(
            self.config.position_cost * np.array([1.0, 0.0, 1.0], dtype=np.float64)
        )
        head_task.set_orientation_cost(
            self.config.orientation_cost * np.array([0.0, 1.0, 1.0], dtype=np.float64)
        )
        return tasks
