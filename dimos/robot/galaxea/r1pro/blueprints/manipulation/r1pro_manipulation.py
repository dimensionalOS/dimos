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

Swaps the whole-body servo task for per-arm trajectory tasks; planned
trajectories execute through ``traj_left_arm`` / ``traj_right_arm``. The
torso holds position; grippers are not driven yet.

Requires the ``r1_pro_description`` package — set ``R1PRO_DESCRIPTION`` to a
local checkout until the asset lands in the LFS store (see ``config.py``).

Usage:
    dimos run r1pro-manipulation
"""

from __future__ import annotations

from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    rerun_config,
)
from dimos.robot.galaxea.r1pro.config import make_r1pro_arm_model_config
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS
from dimos.visualization.vis_module import vis_module

_left_arm_joints = [j for j in R1PRO_UPPER_BODY_JOINTS if "left_arm" in j]
_right_arm_joints = [j for j in R1PRO_UPPER_BODY_JOINTS if "right_arm" in j]

_manipulation_tasks = [
    TaskConfig(
        name="traj_left_arm",
        type="trajectory",
        joint_names=_left_arm_joints,
        priority=10,
    ),
    TaskConfig(
        name="traj_right_arm",
        type="trajectory",
        joint_names=_right_arm_joints,
        priority=10,
    ),
    TaskConfig(
        name="vel_chassis",
        type="velocity",
        joint_names=make_twist_base_joints("chassis"),
        priority=10,
    ),
]

r1pro_manipulation = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    r1pro_control(tasks=_manipulation_tasks),
    ManipulationModule.blueprint(
        robots=[
            make_r1pro_arm_model_config("left"),
            make_r1pro_arm_model_config("right"),
        ],
        planning_timeout=10.0,
    ),
).global_config(n_workers=6)

__all__ = ["r1pro_manipulation"]
