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

from typing import Protocol

from dimos.navigation.spec import NavigationInterfaceSpec


class ReplanningAStarPlannerSpec(NavigationInterfaceSpec, Protocol):
    """The navigation RPCs plus the 2D planner's own tuning calls."""

    def set_replanning_enabled(self, enabled: bool) -> None: ...
    def set_safe_goal_clearance(self, clearance: float) -> None: ...
    def reset_safe_goal_clearance(self) -> None: ...
