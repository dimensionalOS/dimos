# Copyright 2025-2026 Dimensional Inc.
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

"""G1-specific Pink objective tuning for bimanual Quest teleoperation."""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver

_FRAME_POSITION_COST = 8.0
_FRAME_ORIENTATION_COST = 2.0
_POSTURE_WEIGHTS = np.tile(
    np.array([4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1], dtype=np.float64),
    2,
)
_NOMINAL_POSTURE = np.zeros(14, dtype=np.float64)


class G1PinkPoseTargetSolver(PinkPoseTargetSolver):
    """Shape G1 arm redundancy while retaining common solve and safety logic."""

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
            raise ValueError("G1PinkPoseTargetSolver requires a positive posture cost")
        posture_task.cost = self.config.posture_cost * _POSTURE_WEIGHTS
        return tasks

    def _update_current_posture_target(
        self,
        tasks: Mapping[str, pink.Task],
        configuration: pink.Configuration,
    ) -> None:
        posture_task = tasks.get("posture/current")
        if not isinstance(posture_task, pink.tasks.PostureTask):
            raise ValueError("G1PinkPoseTargetSolver requires a posture task")
        if configuration.model.nq != len(_NOMINAL_POSTURE):
            raise ValueError(
                f"G1 nominal posture has {len(_NOMINAL_POSTURE)} joints, "
                f"model has {configuration.model.nq}"
            )
        posture_task.set_target(_NOMINAL_POSTURE)
