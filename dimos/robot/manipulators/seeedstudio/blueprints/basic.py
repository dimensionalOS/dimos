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

"""Basic Seeed Studio reBot B601-DM coordinator and planner blueprints.

Mock hardware by default; pass ``--can-port`` with the HDSC bridge's serial
port to drive the real arm.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.seeedstudio.config import (
    SEEEDSTUDIO_TICK_RATE_HZ,
    SEEEDSTUDIO_VELOCITY_SCALE,
    make_seeedstudio_model_config,
    seeedstudio_gripper_task,
    seeedstudio_hardware,
)

_seeedstudio_planner_hw = seeedstudio_hardware("arm")

seeedstudio_planner_coordinator = autoconnect(
    planner(
        model=make_seeedstudio_model_config(),
        visualization={"backend": "viser"},
        trajectory_parametrization={
            "backend": "simple_trapezoid",
            "velocity_scale": SEEEDSTUDIO_VELOCITY_SCALE,
        },
    ),
    coordinator(
        tick_rate=SEEEDSTUDIO_TICK_RATE_HZ,
        hardware=[_seeedstudio_planner_hw],
        tasks=[trajectory_task(_seeedstudio_planner_hw), seeedstudio_gripper_task()],
    ),
)

_coordinator_seeedstudio_hw = seeedstudio_hardware("arm")

coordinator_seeedstudio = ControlCoordinator.blueprint(
    tick_rate=SEEEDSTUDIO_TICK_RATE_HZ,
    hardware=[_coordinator_seeedstudio_hw],
    tasks=[trajectory_task(_coordinator_seeedstudio_hw), seeedstudio_gripper_task()],
)
