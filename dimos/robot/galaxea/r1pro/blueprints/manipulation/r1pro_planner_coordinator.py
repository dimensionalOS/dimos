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

"""R1 Pro planar-base planner with fake joint-state hardware."""

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_PLANNING_JOINTS,
    make_r1pro_planar_model_config,
)
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task

_r1pro_hardware = HardwareComponent(
    hardware_id="r1pro",
    hardware_type=HardwareType.WHOLE_BODY,
    joints=list(R1PRO_PLANNING_JOINTS),
    adapter_type="mock_whole_body",
)

r1pro_planner_coordinator = autoconnect(
    planner(
        model=make_r1pro_planar_model_config(),
        visualization={"backend": "viser"},
        trajectory_parametrization={"backend": "simple_trapezoid"},
    ),
    coordinator(
        hardware=[_r1pro_hardware],
        tasks=[trajectory_task(_r1pro_hardware)],
    ),
)
