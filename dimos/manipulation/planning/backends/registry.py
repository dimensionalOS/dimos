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

"""Planning backend registry."""

from __future__ import annotations

from typing import Any

from dimos.manipulation.planning.backends.base import PlanningBackend


def create_planning_backend(
    name: str = "drake",
    *,
    enable_viz: bool = False,
    planner_name: str = "rrt_connect",
    kinematics_name: str = "jacobian",
    options: dict[str, Any] | None = None,
) -> PlanningBackend:
    normalized = name.strip().lower()
    if normalized == "drake":
        from dimos.manipulation.planning.backends.drake.backend import DrakePlanningBackend

        return DrakePlanningBackend(
            enable_viz=enable_viz,
            planner_name=planner_name,
            kinematics_name=kinematics_name,
            options=options or {},
        )
    if normalized == "roboplan":
        from dimos.manipulation.planning.backends.roboplan.backend import RoboPlanPlanningBackend

        return RoboPlanPlanningBackend(
            enable_viz=enable_viz,
            planner_name=planner_name,
            kinematics_name=kinematics_name,
            options=options or {},
        )

    raise ValueError(f"Unknown planning backend: {name}. Available: ['drake', 'roboplan']")
