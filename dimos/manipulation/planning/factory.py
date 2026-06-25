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
from typing import TYPE_CHECKING, Any, Literal, TypeAlias, get_args

from dimos.manipulation.planning.kinematics.config import (
    DrakeOptimizationKinematicsConfig,
    JacobianKinematicsConfig,
    ManipulationKinematicsConfig,
    PinkKinematicsConfig,
    kinematics_config_from_name,
)
from dimos.manipulation.planning.spec.protocols import PlannerSpec
from dimos.manipulation.visualization.config import (
    ManipulationVisualizationConfig,
    NoManipulationVisualizationConfig,
)

if TYPE_CHECKING:
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.protocols import (
        KinematicsSpec,
        WorldSpec,
    )


@dataclass(frozen=True)
class PlanningSpecs:
    """Concrete planning specs created from configuration."""

    world_monitor: WorldMonitor
    kinematics: KinematicsSpec
    planner: PlannerSpec


WorldBackend: TypeAlias = Literal["drake", "roboplan"]
PlannerName: TypeAlias = Literal["rrt_connect", "roboplan"]
KinematicsName: TypeAlias = Literal["jacobian", "drake_optimization", "pink"]

SUPPORTED_WORLD_BACKENDS = get_args(WorldBackend)
SUPPORTED_PLANNERS = get_args(PlannerName)
SUPPORTED_KINEMATICS = get_args(KinematicsName)

_ROBOPLAN_PLANNER_REQUIRES_ROBOPLAN_WORLD = (
    'planner_name="roboplan" requires world_backend="roboplan"'
)

DEFAULT_KINEMATICS_NAME: KinematicsName = "pink"


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
        raise ValueError(_ROBOPLAN_PLANNER_REQUIRES_ROBOPLAN_WORLD)
    if kinematics_name == "drake_optimization" and world_backend != "drake":
        raise ValueError('kinematics_name="drake_optimization" requires world_backend="drake"')


def create_world(
    backend: str = "drake",
    visualization: ManipulationVisualizationConfig | None = None,
    **kwargs: Any,
) -> WorldSpec:
    """Create a world instance for the selected planning backend."""
    visualization = visualization or NoManipulationVisualizationConfig()
    enable_viz = visualization.requires_world_visualization

    if backend == "drake":
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        return DrakeWorld(enable_viz=enable_viz, **kwargs)
    if backend == "roboplan":
        from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

        return RoboPlanWorld(enable_viz=enable_viz, **kwargs)

    raise ValueError(f"Unknown backend: {backend}. Available: {list(SUPPORTED_WORLD_BACKENDS)}")


def create_kinematics(
    name: str = DEFAULT_KINEMATICS_NAME,
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
            raise ValueError(_ROBOPLAN_PLANNER_REQUIRES_ROBOPLAN_WORLD)
        if not isinstance(world, PlannerSpec):
            raise ValueError("RoboPlan-native planner requires a RoboPlan world planner object")
        return world

    raise ValueError(f"Unknown planner: {name}. Available: {list(SUPPORTED_PLANNERS)}")


def create_planning_specs(
    world: WorldSpec,
    world_backend: str = "drake",
    planner_name: str = "rrt_connect",
    kinematics_name: str | None = None,
    kinematics: ManipulationKinematicsConfig | None = None,
) -> PlanningSpecs:
    """Create planning specs around an already-created world."""
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor

    if kinematics_name is not None:
        kinematics = kinematics_config_from_name(kinematics_name)
    if kinematics is None:
        kinematics = kinematics_config_from_name(DEFAULT_KINEMATICS_NAME)

    validate_backend_combination(
        world_backend=world_backend,
        planner_name=planner_name,
        kinematics_name=kinematics.backend,
    )

    return PlanningSpecs(
        world_monitor=WorldMonitor(world=world),
        kinematics=create_kinematics(config=kinematics),
        planner=create_planner(name=planner_name, world=world, world_backend=world_backend),
    )


def create_planning_stack(
    robot_config: Any,
    world_backend: str = "drake",
    visualization: ManipulationVisualizationConfig | None = None,
    planner_name: str = "rrt_connect",
    kinematics_name: str | None = None,
    kinematics: ManipulationKinematicsConfig | None = None,
) -> tuple[WorldSpec, KinematicsSpec, PlannerSpec, str]:
    """Create complete planning stack. Returns (world, kinematics, planner, robot_id)."""
    world = create_world(backend=world_backend, visualization=visualization)
    planning_specs = create_planning_specs(
        world=world,
        world_backend=world_backend,
        planner_name=planner_name,
        kinematics_name=kinematics_name,
        kinematics=kinematics,
    )

    robot_id = world.add_robot(robot_config)
    world.finalize()

    return world, planning_specs.kinematics, planning_specs.planner, robot_id
