#!/usr/bin/env python3
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

"""R1 Pro dual-arm manipulation (EXPERIMENTAL).

Narrows the coordinator's whole-body trajectory task to the arms; the
planner picks an arm through the model's ``left_arm`` / ``right_arm`` planning
groups. The chassis is welded in the planning model and driven separately by
the velocity task; grippers are not driven yet.

Usage:
    dimos run r1pro-manipulation
"""

from __future__ import annotations

from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_visualization,
)
from dimos.robot.galaxea.r1pro.config import make_r1pro_model_config
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS

# One canonical trajectory task spans both arms and the torso; the planner
# selects an arm through the model's planning groups, not through task names.
_manipulation_tasks = [
    joint_trajectory_task(R1PRO_UPPER_BODY_JOINTS),
    TaskConfig(
        name="vel_chassis",
        type="velocity",
        joint_names=make_twist_base_joints("chassis"),
        priority=10,
    ),
]

r1pro_manipulation = autoconnect(
    r1pro_visualization(),
    r1pro_control(tasks=_manipulation_tasks),
    ManipulationModule.blueprint(
        model=make_r1pro_model_config(),
        planning_timeout=10.0,
        visualization=ViserVisualizationConfig(host="0.0.0.0"),
    ),
).global_config(n_workers=6)

__all__ = ["r1pro_manipulation"]
