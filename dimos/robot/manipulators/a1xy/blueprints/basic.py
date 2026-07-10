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

"""Basic Galaxea A1X coordinator and planner blueprints.

The hardware adapter for the A1X transport is not wired yet (pinned during
bring-up), so the coordinator runs on the ``mock`` adapter — enough for the
full planning/visualization stack and for exercising trajectory execution
end-to-end in software.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.a1xy.config import (
    make_a1x_hardware,
    make_a1x_model_config,
)
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    planner,
    trajectory_task,
)

a1x_planner_only = ManipulationModule.blueprint(
    robots=[make_a1x_model_config(name="arm")],
    planning_timeout=10.0,
)

_a1x_hw = make_a1x_hardware("arm")

a1x_planner_coordinator = autoconnect(
    planner(robots=[make_a1x_model_config(name="arm")]),
    coordinator(
        hardware=[_a1x_hw],
        tasks=[trajectory_task(_a1x_hw)],
    ),
)

_coordinator_a1x_hw = make_a1x_hardware("arm")

coordinator_a1x = ControlCoordinator.blueprint(
    hardware=[_coordinator_a1x_hw],
    tasks=[trajectory_task(_coordinator_a1x_hw)],
)
