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

"""Compatibility exports for manipulation blueprints.

Robot-owned manipulation blueprints now live under ``dimos.robot.manipulators``.
"""

from dimos.robot.manipulators.xarm.blueprints.agentic import (
    xarm7_planner_coordinator_agent as xarm7_planner_coordinator_agent,
    xarm_perception_agent as xarm_perception_agent,
    xarm_perception_sim_agent as xarm_perception_sim_agent,
)
from dimos.robot.manipulators.xarm.blueprints.basic import (
    dual_xarm6_planner as dual_xarm6_planner,
    xarm6_planner_only as xarm6_planner_only,
    xarm7_planner_coordinator as xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.blueprints.perception import xarm_perception as xarm_perception
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    xarm_perception_sim as xarm_perception_sim,
)
