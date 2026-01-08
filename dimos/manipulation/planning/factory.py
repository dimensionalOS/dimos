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

"""
Factory Functions for Manipulation Planning

Provides factory functions to create instances of WorldSpec, KinematicsSpec,
PlannerSpec, and VizSpec implementations.

Only these factory functions know about concrete implementation types.
All other code should use the Protocol types.

Example:
    from dimos.manipulation.planning.factory import (
        create_world,
        create_kinematics,
        create_planner,
    )

    # Create instances using factory functions
    world = create_world(backend="drake", enable_viz=True)
    kinematics = create_kinematics(backend="drake")
    planner = create_planner(name="rrt_connect", backend="drake")

    # Use them with Protocol types
    robot_id = world.add_robot(config)
    world.finalize()

    result = kinematics.solve(world, robot_id, target_pose)
    plan_result = planner.plan_joint_path(world, robot_id, q_start, q_goal)
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec import (
        KinematicsSpec,
        PlannerSpec,
        VizSpec,
        WorldSpec,
    )


def create_world(
    backend: str = "drake",
    enable_viz: bool = False,
    **kwargs: Any,
) -> WorldSpec:
    """Create a world instance.

    The World owns the physics/collision backend and provides:
    - Robot/obstacle management
    - Collision checking
    - Forward kinematics
    - Context management for thread safety

    Args:
        backend: Backend to use ("drake", future: "mujoco", "vamp")
        enable_viz: Enable Meshcat visualization (Drake only)
        **kwargs: Additional backend-specific arguments
            - time_step: Simulation time step (default 0.0 for kinematics-only)

    Returns:
        WorldSpec implementation

    Example:
        world = create_world(backend="drake", enable_viz=True)
        robot_id = world.add_robot(robot_config)
        world.finalize()

        with world.scratch_context() as ctx:
            world.set_positions(ctx, robot_id, q)
            if world.is_collision_free(ctx, robot_id):
                ee_pose = world.get_ee_pose(ctx, robot_id)
    """
    if backend == "drake":
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        return DrakeWorld(enable_viz=enable_viz, **kwargs)
    else:
        raise ValueError(f"Unknown backend: {backend}. Available: ['drake']")


def create_kinematics(
    backend: str = "drake",
    **kwargs: Any,
) -> KinematicsSpec:
    """Create a kinematics solver instance.

    Kinematics solvers are stateless (except for configuration) and
    use WorldSpec for all FK/collision operations.

    Args:
        backend: Backend to use ("drake")
        **kwargs: Additional backend-specific arguments
            - damping: Damping factor for differential IK (default 0.01)
            - max_iterations: Default max iterations for iterative IK (default 100)
            - singularity_threshold: Manipulability threshold (default 0.001)

    Returns:
        KinematicsSpec implementation

    Example:
        kinematics = create_kinematics(backend="drake", damping=0.01)
        result = kinematics.solve(world, robot_id, target_pose, seed=q_current)
        if result.is_success():
            q_goal = result.joint_positions
    """
    if backend == "drake":
        from dimos.manipulation.planning.kinematics.drake_kinematics import (
            DrakeKinematics,
        )

        return DrakeKinematics(**kwargs)
    else:
        raise ValueError(f"Unknown backend: {backend}. Available: ['drake']")


def create_planner(
    name: str = "rrt_connect",
    backend: str = "drake",
    **kwargs: Any,
) -> PlannerSpec:
    """Create a motion planner instance.

    Planners find collision-free paths from start to goal configurations.
    They use WorldSpec for collision checking and are stateless.

    Args:
        name: Planner algorithm ("rrt_connect", "rrt_star")
        backend: Backend to use ("drake")
        **kwargs: Additional planner-specific arguments
            RRT-Connect:
            - step_size: Extension step size (default 0.1)
            - connect_step_size: Connect step size (default 0.05)
            - goal_tolerance: Goal tolerance (default 0.1)
            - collision_step_size: Collision check step (default 0.02)

            RRT*:
            - step_size: Extension step size (default 0.1)
            - goal_tolerance: Goal tolerance (default 0.1)
            - rewire_radius: Rewiring radius (default 0.5)
            - collision_step_size: Collision check step (default 0.02)

    Returns:
        PlannerSpec implementation

    Example:
        planner = create_planner(name="rrt_connect", step_size=0.1)
        result = planner.plan_joint_path(world, robot_id, q_start, q_goal)
        if result.is_success():
            waypoints = result.path
    """
    if backend == "drake":
        if name == "rrt_connect":
            from dimos.manipulation.planning.planners.drake_planner import DrakePlanner

            return DrakePlanner(**kwargs)
        elif name == "rrt_star":
            from dimos.manipulation.planning.planners.drake_planner import (
                DrakeRRTStarPlanner,
            )

            return DrakeRRTStarPlanner(**kwargs)
        else:
            raise ValueError(
                f"Unknown planner: {name}. Available for Drake: ['rrt_connect', 'rrt_star']"
            )
    else:
        raise ValueError(f"Unknown backend: {backend}. Available: ['drake']")


def create_viz(
    backend: str = "drake",
    world: WorldSpec | None = None,
    **kwargs: Any,
) -> VizSpec:
    """Create a visualization instance.

    Provides methods to update robot/obstacle visualization.
    Can be integrated with WorldSpec or standalone.

    Note: For Drake, visualization is typically integrated into DrakeWorld
    via enable_viz=True. This function is for advanced use cases where
    separate visualization is needed.

    Args:
        backend: Backend to use ("drake")
        world: Optional world to integrate with
        **kwargs: Additional backend-specific arguments

    Returns:
        VizSpec implementation

    Example:
        # Option 1: Integrated visualization (recommended)
        world = create_world(backend="drake", enable_viz=True)
        print(f"Meshcat URL: {world.get_meshcat_url()}")

        # Option 2: Separate visualization (advanced)
        viz = create_viz(backend="drake", world=world)
        viz.set_robot_state(robot_id, q)
        viz.publish()
    """
    if backend == "drake":
        # For Drake, visualization is integrated into DrakeWorld
        # This is a placeholder for potential future standalone viz
        raise NotImplementedError(
            "For Drake, use create_world(enable_viz=True) instead. "
            "Separate visualization is not yet implemented."
        )
    else:
        raise ValueError(f"Unknown backend: {backend}. Available: ['drake']")


# ============= Convenience Functions =============


def create_planning_stack(
    robot_config: Any,
    enable_viz: bool = False,
    planner_name: str = "rrt_connect",
) -> tuple[WorldSpec, KinematicsSpec, PlannerSpec, str]:
    """Convenience function to create a complete planning stack.

    Creates world, kinematics, and planner, adds robot, and finalizes.

    Args:
        robot_config: RobotModelConfig for the robot
        enable_viz: Enable visualization
        planner_name: Planner algorithm to use

    Returns:
        Tuple of (world, kinematics, planner, robot_id)

    Example:
        world, kinematics, planner, robot_id = create_planning_stack(
            robot_config=piper_config,
            enable_viz=True,
        )

        # Now ready to use
        result = kinematics.solve(world, robot_id, target_pose)
    """
    world = create_world(backend="drake", enable_viz=enable_viz)
    kinematics = create_kinematics(backend="drake")
    planner = create_planner(name=planner_name, backend="drake")

    robot_id = world.add_robot(robot_config)
    world.finalize()

    return world, kinematics, planner, robot_id
