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

"""OpenArm-specific Pink pose-target solver for Quest teleoperation."""

from __future__ import annotations

import numpy as np
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver

_FRAME_POSITION_COST = 1.0
_FRAME_ORIENTATION_COST = 0.2
_MANIPULABILITY_COST = 0.005
_MANIPULABILITY_RATE = 0.05
_POSTURE_WEIGHTS = np.tile(
    np.array([4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1], dtype=np.float64),
    2,
)


class OpenArmPinkPoseTargetSolver(PinkPoseTargetSolver):
    """Shape OpenArm redundancy without changing common solve or safety logic."""

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)

        for frame_name in target_frames:
            frame_task = tasks[f"frame/{frame_name}"]
            frame_task.set_position_cost(_FRAME_POSITION_COST)
            frame_task.set_orientation_cost(_FRAME_ORIENTATION_COST)

        posture_task = tasks.get("posture/current")
        if posture_task is None:
            raise ValueError("OpenArmPinkPoseTargetSolver requires a positive posture cost")
        posture_task.cost = self.config.posture_cost * _POSTURE_WEIGHTS

        for frame_name in target_frames:
            tasks[f"manipulability/{frame_name}"] = pink.tasks.ManipulabilityTask(
                frame_name,
                configuration.model,
                cost=_MANIPULABILITY_COST,
                manipulability_rate=_MANIPULABILITY_RATE,
                mask="position",
            )
        return tasks
