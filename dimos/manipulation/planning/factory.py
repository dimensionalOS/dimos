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

"""Factory functions for manipulation planning components."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec import (
        KinematicsSpec,
        PlannerSpec,
        WorldSpec,
    )


def create_world(
    backend: str = "drake",
    enable_viz: bool = False,
    **kwargs: Any,
) -> WorldSpec:
    """Create a world instance. backend='drake', enable_viz for Meshcat."""
    if backend == "drake":
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        return DrakeWorld(enable_viz=enable_viz, **kwargs)
    else:
        raise ValueError(f"Unknown backend: {backend}. Available: ['drake']")


def create_kinematics(
    name: str = "jacobian",
    **kwargs: Any,
) -> KinematicsSpec:
    """Create IK solver. name='jacobian'|'drake_optimization'."""
    if name == "jacobian":
        from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK

        return JacobianIK(**kwargs)
    elif name == "drake_optimization":
        from dimos.manipulation.planning.kinematics.drake_optimization_ik import (
            DrakeOptimizationIK,
        )

        return DrakeOptimizationIK(**kwargs)
    else:
        raise ValueError(
            f"Unknown kinematics solver: {name}. Available: ['jacobian', 'drake_optimization']"
        )


def create_planner(
    name: str = "rrt_connect",
    **kwargs: Any,
) -> PlannerSpec:
    """Create motion planner. name='rrt_connect'."""
    if name == "rrt_connect":
        from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner

        return RRTConnectPlanner(**kwargs)
    else:
        raise ValueError(f"Unknown planner: {name}. Available: ['rrt_connect']")


def create_planning_stack(
    robot_config: Any,
    enable_viz: bool = False,
    planner_name: str = "rrt_connect",
    kinematics_name: str = "jacobian",
) -> tuple[WorldSpec, KinematicsSpec, PlannerSpec, str]:
    """Create complete planning stack. Returns (world, kinematics, planner, robot_id)."""
    world = create_world(backend="drake", enable_viz=enable_viz)
    kinematics = create_kinematics(name=kinematics_name)
    planner = create_planner(name=planner_name)

    robot_id = world.add_robot(robot_config)
    world.finalize()

    return world, kinematics, planner, robot_id
