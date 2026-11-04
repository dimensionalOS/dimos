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

from dimos.core.module import ModuleBase
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.blueprints.m20_nav import m20_nav
from dimos.spec.utils import Spec


def test_teleop_cancel_reaches_the_planner() -> None:
    def goal_topic(module: type[ModuleBase]) -> tuple[str | type[ModuleBase] | type[Spec], type]:
        (atom,) = (a for a in m20_nav.active_blueprints if a.module is module)
        (stream,) = (s for s in atom.streams if s.name == "goal")
        return m20_nav.remapping_map.get((atom.name, "goal"), "goal"), stream.type

    assert goal_topic(MLSPlannerNative) == goal_topic(MovementManager)
