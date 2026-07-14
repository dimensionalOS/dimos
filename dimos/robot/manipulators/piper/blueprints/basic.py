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

"""Basic Piper coordinator blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.piper.config import PIPER_SIM_PATH, piper_hardware

_piper_hw = piper_hardware("arm")

coordinator_piper = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_piper_hw],
        tasks=[
            TaskConfig(
                name="traj_piper",
                type="trajectory",
                joint_names=_piper_hw.joints,
                priority=10,
            )
        ],
    ),
    *mujoco_if_sim(PIPER_SIM_PATH, len(_piper_hw.joints)),
)
