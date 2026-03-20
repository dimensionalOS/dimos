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

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from dimos.navigation.patrolling.routers.patrol_router import PatrolRouter

PatrolRouterName = Literal["random", "coverage", "frontier"]


def create_patrol_router(name: PatrolRouterName, clearance_radius_m: float) -> PatrolRouter:
    match name:
        case "random":
            # Inline to avoid unnecessary imports.
            from dimos.navigation.patrolling.routers.random_patrol_router import RandomPatrolRouter

            return RandomPatrolRouter(clearance_radius_m)
        case "coverage":
            # Inline to avoid unnecessary imports.
            from dimos.navigation.patrolling.routers.coverage_patrol_router import (
                CoveragePatrolRouter,
            )

            return CoveragePatrolRouter(clearance_radius_m)
        case "frontier":
            # Inline to avoid unnecessary imports.
            from dimos.navigation.patrolling.routers.frontier_patrol_router import (
                FrontierPatrolRouter,
            )

            return FrontierPatrolRouter(clearance_radius_m)
