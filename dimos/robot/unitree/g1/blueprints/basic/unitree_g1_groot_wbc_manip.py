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

"""Unitree G1 GR00T WBC + manipulation stack.

Extends ``unitree-g1-groot-wbc`` with a ManipulationModule planning against
the full-body G1 model and a joint-trajectory task claiming the 14 arm
joints at priority 30 -- above the priority-10 servo hold and disjoint from
the priority-50 policy's legs+waist claim -- so planned arm trajectories
override the servo hold while executing and release back to it on
completion.

Usage:
    dimos --simulation mujoco --scene-package office run unitree-g1-groot-wbc-manip
"""

from __future__ import annotations

from typing import Any, cast

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_arms
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    _backend,
    _n_workers,
    _nav_stack,
    _remappings,
    _viewer,
    g1_groot_coordinator,
)
from dimos.robot.unitree.g1.manip_config import make_g1_model_config

_ARM_TRAJECTORY_TASK = TaskConfig(
    name="traj_arms",
    type="trajectory",
    joint_names=list(g1_arms),
    priority=30,
)

unitree_g1_groot_wbc_manip = (
    autoconnect(
        _backend,
        g1_groot_coordinator(extra_tasks=(_ARM_TRAJECTORY_TASK,)),
        _nav_stack,
        ManipulationModule.blueprint(
            robots=[make_g1_model_config()],
            # Reaches are position-driven; a hard 0.01 rad orientation
            # tolerance makes the 7-DOF solve knife-edged from live stances.
            kinematics=PinkKinematicsConfig(
                position_tolerance=0.01, orientation_cost=0.3, orientation_tolerance=0.35
            ),
            visualization=ViserVisualizationConfig(),
        ),
        _viewer(),
    )
    .remappings(cast("Any", _remappings))
    .global_config(robot_model="unitree_g1", n_workers=_n_workers + 1)
)
