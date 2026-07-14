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

"""Basic Galaxea A1Z coordinator and planner blueprints.

With ``--can-port can0`` these drive real hardware through the ``a1z``
adapter; without it they fall back to the mock adapter, which still exercises
the full planning / visualization stack in software.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.a1z.config import a1z_hardware, make_a1z_model_config
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    planner,
    trajectory_task,
)

a1z_planner_only = ManipulationModule.blueprint(
    robots=[make_a1z_model_config(name="arm")],
    planning_timeout=10.0,
)

_a1z_planner_hw = a1z_hardware("arm")

a1z_planner_coordinator = autoconnect(
    planner(robots=[make_a1z_model_config(name="arm")]),
    coordinator(
        hardware=[_a1z_planner_hw],
        tasks=[trajectory_task(_a1z_planner_hw)],
    ),
)

_coordinator_a1z_hw = a1z_hardware("arm")

coordinator_a1z = ControlCoordinator.blueprint(
    hardware=[_coordinator_a1z_hw],
    tasks=[trajectory_task(_coordinator_a1z_hw)],
)
