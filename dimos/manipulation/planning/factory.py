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
    from dimos.manipulation.planning.spec.protocols import KinematicsSpec, PlannerSpec, WorldSpec


SUPPORTED_WORLD_BACKENDS = ("drake", "roboplan")
SUPPORTED_PLANNERS = ("rrt_connect", "roboplan")
SUPPORTED_KINEMATICS = ("jacobian", "drake_optimization")


def validate_backend_combination(
    *,
    world_backend: str = "drake",
    planner_name: str = "rrt_connect",
    kinematics_name: str = "jacobian",
) -> None:
    """Validate manipulation backend choices before constructing the stack."""
    if world_backend not in SUPPORTED_WORLD_BACKENDS:
        raise ValueError(
            f"Unknown backend: {world_backend}. Available: {list(SUPPORTED_WORLD_BACKENDS)}"
        )
    if planner_name not in SUPPORTED_PLANNERS:
        raise ValueError(f"Unknown planner: {planner_name}. Available: {list(SUPPORTED_PLANNERS)}")
    if kinematics_name not in SUPPORTED_KINEMATICS:
        raise ValueError(
            f"Unknown kinematics solver: {kinematics_name}. Available: {list(SUPPORTED_KINEMATICS)}"
        )

    if planner_name == "roboplan" and world_backend != "roboplan":
        raise ValueError('planner_name="roboplan" requires world_backend="roboplan"')
    if kinematics_name == "drake_optimization" and world_backend != "drake":
        raise ValueError('kinematics_name="drake_optimization" requires world_backend="drake"')


def create_world(
    backend: str = "drake",
    enable_viz: bool = False,
    **kwargs: Any,
) -> WorldSpec:
    """Create a world instance. backend='drake'|'roboplan', enable_viz for supported backends."""
    if backend == "drake":
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        return DrakeWorld(enable_viz=enable_viz, **kwargs)
    if backend == "roboplan":
        from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

        return RoboPlanWorld(enable_viz=enable_viz, **kwargs)

    raise ValueError(f"Unknown backend: {backend}. Available: {list(SUPPORTED_WORLD_BACKENDS)}")


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
            f"Unknown kinematics solver: {name}. Available: {list(SUPPORTED_KINEMATICS)}"
        )


def create_planner(
    name: str = "rrt_connect",
    world: WorldSpec | None = None,
    world_backend: str | None = None,
    **kwargs: Any,
) -> PlannerSpec:
    """Create motion planner. name='rrt_connect'|'roboplan'.

    RoboPlan-native planning is scene/backend-coupled, so `name='roboplan'`
    returns the RoboPlan world object itself as the planner.
    """
    if name == "rrt_connect":
        from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner

        return RRTConnectPlanner(**kwargs)
    if name == "roboplan":
        if world_backend != "roboplan" or world is None:
            raise ValueError('planner_name="roboplan" requires world_backend="roboplan"')
        if not hasattr(world, "plan_joint_path"):
            raise ValueError("RoboPlan-native planner requires a RoboPlan world planner object")
        return world  # type: ignore[return-value]

    raise ValueError(f"Unknown planner: {name}. Available: {list(SUPPORTED_PLANNERS)}")


def create_planning_stack(
    robot_config: Any,
    enable_viz: bool = False,
    world_backend: str = "drake",
    planner_name: str = "rrt_connect",
    kinematics_name: str = "jacobian",
) -> tuple[WorldSpec, KinematicsSpec, PlannerSpec, str]:
    """Create complete planning stack. Returns (world, kinematics, planner, robot_id)."""
    validate_backend_combination(
        world_backend=world_backend,
        planner_name=planner_name,
        kinematics_name=kinematics_name,
    )
    world = create_world(backend=world_backend, enable_viz=enable_viz)
    kinematics = create_kinematics(name=kinematics_name)
    planner = create_planner(name=planner_name, world=world, world_backend=world_backend)

    robot_id = world.add_robot(robot_config)
    world.finalize()

    return world, kinematics, planner, robot_id
