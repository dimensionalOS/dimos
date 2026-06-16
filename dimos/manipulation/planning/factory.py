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

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos.manipulation.planning.kinematics.config import (
    DrakeOptimizationKinematicsConfig,
    JacobianKinematicsConfig,
    ManipulationKinematicsConfig,
    PinkKinematicsConfig,
    kinematics_config_from_name,
)
from dimos.manipulation.visualization.config import (
    ManipulationVisualizationConfig,
    NoManipulationVisualizationConfig,
)

if TYPE_CHECKING:
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.protocols import (
        KinematicsSpec,
        PlannerSpec,
        WorldSpec,
    )


@dataclass(frozen=True)
class PlanningSpecs:
    """Concrete planning specs created from configuration."""

    world_monitor: WorldMonitor
    kinematics: KinematicsSpec
    planner: PlannerSpec


def create_world(
    backend: str = "drake",
    visualization: ManipulationVisualizationConfig | None = None,
    **kwargs: Any,
) -> WorldSpec:
    """Create a world instance. backend='drake' only for now."""
    visualization = visualization or NoManipulationVisualizationConfig()
    if backend == "drake":
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        return DrakeWorld(enable_viz=visualization.requires_world_visualization, **kwargs)
    else:
        raise ValueError(f"Unknown backend: {backend}. Available: ['drake']")


def create_kinematics(
    name: str = "pink",
    config: ManipulationKinematicsConfig | None = None,
    **kwargs: Any,
) -> KinematicsSpec:
    """Create IK solver from a backend name or typed kinematics config."""
    if config is None:
        config = kinematics_config_from_name(name)

    if isinstance(config, JacobianKinematicsConfig):
        from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK

        return JacobianIK(**kwargs)
    elif isinstance(config, DrakeOptimizationKinematicsConfig):
        from dimos.manipulation.planning.kinematics.drake_optimization_ik import (
            DrakeOptimizationIK,
        )

        return DrakeOptimizationIK(**kwargs)
    elif isinstance(config, PinkKinematicsConfig):
        from dimos.manipulation.planning.kinematics.pink_ik import PinkIK

        return PinkIK(config, **kwargs)
    else:
        raise TypeError(f"Unsupported kinematics config: {type(config).__name__}")


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


def create_planning_specs(
    world: WorldSpec,
    planner_name: str = "rrt_connect",
    kinematics_name: str | None = None,
    kinematics: ManipulationKinematicsConfig | None = None,
) -> PlanningSpecs:
    """Create planning specs around an already-created world."""
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor

    if kinematics_name is not None:
        kinematics = kinematics_config_from_name(kinematics_name)

    return PlanningSpecs(
        world_monitor=WorldMonitor(world=world),
        kinematics=create_kinematics(config=kinematics),
        planner=create_planner(name=planner_name),
    )


def create_planning_stack(
    robot_config: Any,
    visualization: ManipulationVisualizationConfig | None = None,
    planner_name: str = "rrt_connect",
    kinematics_name: str | None = None,
    kinematics: ManipulationKinematicsConfig | None = None,
) -> tuple[WorldSpec, KinematicsSpec, PlannerSpec, str]:
    """Create complete planning stack. Returns (world, kinematics, planner, robot_id)."""
    world = create_world(visualization=visualization)
    planning_specs = create_planning_specs(
        world=world,
        planner_name=planner_name,
        kinematics_name=kinematics_name,
        kinematics=kinematics,
    )

    robot_id = world.add_robot(robot_config)
    world.finalize()

    return world, planning_specs.kinematics, planning_specs.planner, robot_id
