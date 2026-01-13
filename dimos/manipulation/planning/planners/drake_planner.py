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
Drake Planner Module

Implements PlannerSpec using RRT-Connect algorithm with WorldSpec for collision checking.

## Key Design

- Uses WorldSpec.scratch_context() for thread-safe collision checking
- Joint-space planning only (use KinematicsSpec.solve() for pose goals first)
- Stateless except for configuration

Example:
    planner = DrakePlanner(step_size=0.1)
    result = planner.plan_joint_path(world, robot_id, q_start, q_goal)
    if result.is_success():
        waypoints = result.path
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.spec import PlanningResult, PlanningStatus, WorldSpec
from dimos.manipulation.planning.utils.path_utils import (
    compute_path_length,
    interpolate_path,
    interpolate_segment,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


@dataclass
class TreeNode:
    """Node in the RRT tree."""

    config: NDArray[np.float64]
    parent: TreeNode | None = None
    children: list[TreeNode] = field(default_factory=list)

    def path_to_root(self) -> list[NDArray[np.float64]]:
        """Get path from this node to root."""
        path = []
        node: TreeNode | None = self
        while node is not None:
            path.append(node.config)
            node = node.parent
        return list(reversed(path))


class DrakePlanner:
    """RRT-Connect planner implementing PlannerSpec.

    Uses bi-directional RRT (RRT-Connect) for joint-space path planning
    with collision checking via WorldSpec.scratch_context().

    ## Algorithm

    1. Initialize start tree and goal tree
    2. Extend start tree toward random sample
    3. Try to connect goal tree to new node
    4. Swap trees and repeat
    5. If connected, extract path

    ## Thread Safety

    All collision checking uses world.scratch_context() for thread safety.
    """

    def __init__(
        self,
        step_size: float = 0.1,
        connect_step_size: float = 0.05,
        goal_tolerance: float = 0.1,
        collision_step_size: float = 0.02,
    ):
        """Create RRT-Connect planner.

        Args:
            step_size: Extension step size in joint space (radians)
            connect_step_size: Step size for connect attempts
            goal_tolerance: Distance to goal to consider success
            collision_step_size: Step size for collision checking along edges
        """
        self._step_size = step_size
        self._connect_step_size = connect_step_size
        self._goal_tolerance = goal_tolerance
        self._collision_step_size = collision_step_size

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float = 10.0,
        max_iterations: int = 5000,
    ) -> PlanningResult:
        """Plan collision-free joint-space path using RRT-Connect.

        Args:
            world: World for collision checking
            robot_id: Which robot
            q_start: Start joint configuration (radians)
            q_goal: Goal joint configuration (radians)
            timeout: Maximum planning time (seconds)
            max_iterations: Maximum iterations

        Returns:
            PlanningResult with path (list of waypoints) or failure info
        """
        start_time = time.time()

        # Validate inputs
        error = self._validate_inputs(world, robot_id, q_start, q_goal)
        if error is not None:
            return error

        # Get joint limits
        lower_limits, upper_limits = world.get_joint_limits(robot_id)

        # Initialize trees
        start_tree = [TreeNode(config=q_start.copy())]
        goal_tree = [TreeNode(config=q_goal.copy())]

        trees_swapped = False  # Track if trees have been swapped an odd number of times

        for iteration in range(max_iterations):
            # Check timeout
            if time.time() - start_time > timeout:
                return _create_failure_result(
                    PlanningStatus.TIMEOUT,
                    f"Timeout after {iteration} iterations",
                    planning_time=time.time() - start_time,
                    iterations=iteration,
                )

            # Sample random configuration
            sample = np.random.uniform(lower_limits, upper_limits)

            # Extend start tree toward sample
            extended_node = self._extend_tree(world, robot_id, start_tree, sample, self._step_size)

            if extended_node is not None:
                # Try to connect goal tree to extended node
                connected_node = self._connect_tree(
                    world, robot_id, goal_tree, extended_node.config, self._connect_step_size
                )

                if connected_node is not None:
                    # Trees connected! Extract path
                    path = self._extract_path(extended_node, connected_node)

                    # If trees were swapped, path is from goal to start - reverse it
                    if trees_swapped:
                        path = list(reversed(path))

                    # Simplify path
                    path = self._simplify_path(world, robot_id, path)

                    return _create_success_result(
                        path=path,
                        planning_time=time.time() - start_time,
                        iterations=iteration + 1,
                    )

            # Swap trees
            start_tree, goal_tree = goal_tree, start_tree
            trees_swapped = not trees_swapped

        return _create_failure_result(
            PlanningStatus.NO_SOLUTION,
            f"No path found after {max_iterations} iterations",
            planning_time=time.time() - start_time,
            iterations=max_iterations,
        )

    def get_name(self) -> str:
        """Get planner name."""
        return "RRTConnect"

    def _validate_inputs(
        self,
        world: WorldSpec,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
    ) -> PlanningResult | None:
        """Validate planning inputs, returns error result or None if valid."""
        # Check world is finalized
        if not world.is_finalized:
            return _create_failure_result(
                PlanningStatus.NO_SOLUTION,
                "World must be finalized before planning",
            )

        # Check robot exists
        if robot_id not in world.get_robot_ids():
            return _create_failure_result(
                PlanningStatus.NO_SOLUTION,
                f"Robot '{robot_id}' not found",
            )

        # Check start validity
        with world.scratch_context() as ctx:
            world.set_positions(ctx, robot_id, q_start)
            if not world.is_collision_free(ctx, robot_id):
                return _create_failure_result(
                    PlanningStatus.COLLISION_AT_START,
                    "Start configuration is in collision",
                )

        # Check goal validity
        with world.scratch_context() as ctx:
            world.set_positions(ctx, robot_id, q_goal)
            if not world.is_collision_free(ctx, robot_id):
                return _create_failure_result(
                    PlanningStatus.COLLISION_AT_GOAL,
                    "Goal configuration is in collision",
                )

        # Check limits
        lower, upper = world.get_joint_limits(robot_id)
        if np.any(q_start < lower) or np.any(q_start > upper):
            return _create_failure_result(
                PlanningStatus.INVALID_START,
                "Start configuration is outside joint limits",
            )

        if np.any(q_goal < lower) or np.any(q_goal > upper):
            return _create_failure_result(
                PlanningStatus.INVALID_GOAL,
                "Goal configuration is outside joint limits",
            )

        return None

    def _extend_tree(
        self,
        world: WorldSpec,
        robot_id: str,
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
    ) -> TreeNode | None:
        """Extend tree toward target, returns new node if successful."""
        # Find nearest node
        nearest = min(tree, key=lambda n: float(np.linalg.norm(n.config - target)))

        # Compute new config
        diff = target - nearest.config
        dist = float(np.linalg.norm(diff))

        if dist <= step_size:
            new_config = target.copy()
        else:
            new_config = nearest.config + step_size * (diff / dist)

        # Check validity of edge
        if self._is_edge_valid(world, robot_id, nearest.config, new_config):
            new_node = TreeNode(config=new_config, parent=nearest)
            nearest.children.append(new_node)
            tree.append(new_node)
            return new_node

        return None

    def _connect_tree(
        self,
        world: WorldSpec,
        robot_id: str,
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
    ) -> TreeNode | None:
        """Try to connect tree to target, returns connected node if successful."""
        # Keep extending toward target
        while True:
            result = self._extend_tree(world, robot_id, tree, target, step_size)

            if result is None:
                return None  # Extension failed

            # Check if reached target
            if float(np.linalg.norm(result.config - target)) < self._goal_tolerance:
                return result

    def _is_edge_valid(
        self,
        world: WorldSpec,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_end: NDArray[np.float64],
    ) -> bool:
        """Check if edge between two configurations is collision-free."""
        # Interpolate and check each point
        segment = interpolate_segment(q_start, q_end, self._collision_step_size)

        with world.scratch_context() as ctx:
            for q in segment:
                world.set_positions(ctx, robot_id, q)
                if not world.is_collision_free(ctx, robot_id):
                    return False

        return True

    def _extract_path(
        self,
        start_node: TreeNode,
        goal_node: TreeNode,
    ) -> list[NDArray[np.float64]]:
        """Extract path from two connected nodes."""
        # Path from start node to its root (reversed to be root->node)
        start_path = start_node.path_to_root()

        # Path from goal node to its root
        goal_path = goal_node.path_to_root()

        # Combine: start_root -> start_node -> goal_node -> goal_root
        # But we need start -> goal, so reverse the goal path
        full_path = start_path + list(reversed(goal_path))

        return full_path

    def _simplify_path(
        self,
        world: WorldSpec,
        robot_id: str,
        path: list[NDArray[np.float64]],
        max_iterations: int = 100,
    ) -> list[NDArray[np.float64]]:
        """Simplify path by random shortcutting."""
        if len(path) <= 2:
            return path

        simplified = list(path)

        for _ in range(max_iterations):
            if len(simplified) <= 2:
                break

            # Pick two random indices (at least 2 apart)
            i = np.random.randint(0, len(simplified) - 2)
            j = np.random.randint(i + 2, len(simplified))

            # Check if direct connection is valid
            if self._is_edge_valid(world, robot_id, simplified[i], simplified[j]):
                # Remove intermediate waypoints
                simplified = simplified[: i + 1] + simplified[j:]

        return simplified


class DrakeRRTStarPlanner:
    """RRT* (Optimal RRT) planner implementing PlannerSpec.

    Like RRT but optimizes path cost through rewiring.
    Produces asymptotically optimal paths.
    """

    def __init__(
        self,
        step_size: float = 0.1,
        goal_tolerance: float = 0.1,
        rewire_radius: float = 0.5,
        collision_step_size: float = 0.02,
    ):
        """Create RRT* planner.

        Args:
            step_size: Extension step size
            goal_tolerance: Distance to goal to consider success
            rewire_radius: Radius for rewiring neighbors
            collision_step_size: Step size for collision checking
        """
        self._step_size = step_size
        self._goal_tolerance = goal_tolerance
        self._rewire_radius = rewire_radius
        self._collision_step_size = collision_step_size

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float = 10.0,
        max_iterations: int = 5000,
    ) -> PlanningResult:
        """Plan optimal collision-free joint-space path using RRT*."""
        start_time = time.time()

        # Get joint limits
        lower_limits, upper_limits = world.get_joint_limits(robot_id)

        # Validate start/goal
        with world.scratch_context() as ctx:
            world.set_positions(ctx, robot_id, q_start)
            if not world.is_collision_free(ctx, robot_id):
                return _create_failure_result(
                    PlanningStatus.COLLISION_AT_START,
                    "Start configuration is in collision",
                )

            world.set_positions(ctx, robot_id, q_goal)
            if not world.is_collision_free(ctx, robot_id):
                return _create_failure_result(
                    PlanningStatus.COLLISION_AT_GOAL,
                    "Goal configuration is in collision",
                )

        # Initialize tree with costs
        root = _RRTStarNode(config=q_start.copy(), cost=0.0)
        nodes = [root]
        goal_node: _RRTStarNode | None = None
        best_cost = float("inf")

        for iteration in range(max_iterations):  # noqa: B007
            if time.time() - start_time > timeout:
                break  # Return best path found so far

            # Sample
            sample = np.random.uniform(lower_limits, upper_limits)

            # Find nearest
            nearest = min(nodes, key=lambda n: float(np.linalg.norm(n.config - sample)))

            # Extend
            diff = sample - nearest.config
            dist = float(np.linalg.norm(diff))
            if dist > self._step_size:
                new_config = nearest.config + self._step_size * (diff / dist)
            else:
                new_config = sample.copy()

            if not self._is_edge_valid(world, robot_id, nearest.config, new_config):
                continue

            # Find neighbors within rewire radius
            neighbors = [
                n
                for n in nodes
                if float(np.linalg.norm(n.config - new_config)) < self._rewire_radius
            ]

            # Find best parent
            best_parent = nearest
            best_new_cost = nearest.cost + float(np.linalg.norm(new_config - nearest.config))

            for neighbor in neighbors:
                new_cost = neighbor.cost + float(np.linalg.norm(new_config - neighbor.config))
                if new_cost < best_new_cost:
                    if self._is_edge_valid(world, robot_id, neighbor.config, new_config):
                        best_parent = neighbor
                        best_new_cost = new_cost

            # Add new node
            new_node = _RRTStarNode(
                config=new_config,
                parent=best_parent,
                cost=best_new_cost,
            )
            best_parent.children.append(new_node)
            nodes.append(new_node)

            # Rewire neighbors through new node
            for neighbor in neighbors:
                if neighbor == best_parent:
                    continue
                potential_cost = new_node.cost + float(
                    np.linalg.norm(neighbor.config - new_node.config)
                )
                if potential_cost < neighbor.cost:
                    if self._is_edge_valid(world, robot_id, new_node.config, neighbor.config):
                        # Rewire
                        if neighbor.parent is not None:
                            neighbor.parent.children.remove(neighbor)
                        neighbor.parent = new_node
                        neighbor.cost = potential_cost
                        new_node.children.append(neighbor)
                        self._update_costs(neighbor)

            # Check goal
            if float(np.linalg.norm(new_node.config - q_goal)) < self._goal_tolerance:
                if new_node.cost < best_cost:
                    goal_node = new_node
                    best_cost = new_node.cost

        if goal_node is not None:
            path = goal_node.path_to_root()
            path.append(q_goal)

            return _create_success_result(
                path=path,
                planning_time=time.time() - start_time,
                iterations=iteration + 1,
            )

        return _create_failure_result(
            PlanningStatus.NO_SOLUTION,
            f"No path found after {max_iterations} iterations",
            planning_time=time.time() - start_time,
            iterations=max_iterations,
        )

    def get_name(self) -> str:
        return "RRTStar"

    def _is_edge_valid(
        self,
        world: WorldSpec,
        robot_id: str,
        q_start: NDArray[np.float64],
        q_end: NDArray[np.float64],
    ) -> bool:
        """Check if edge is collision-free."""
        segment = interpolate_segment(q_start, q_end, self._collision_step_size)

        with world.scratch_context() as ctx:
            for q in segment:
                world.set_positions(ctx, robot_id, q)
                if not world.is_collision_free(ctx, robot_id):
                    return False

        return True

    def _update_costs(self, node: _RRTStarNode) -> None:
        """Recursively update costs after rewiring."""
        for child in node.children:
            child.cost = node.cost + float(np.linalg.norm(child.config - node.config))
            self._update_costs(child)


@dataclass
class _RRTStarNode:
    """Node for RRT* with cost tracking."""

    config: NDArray[np.float64]
    cost: float = 0.0
    parent: _RRTStarNode | None = None
    children: list[_RRTStarNode] = field(default_factory=list)

    def path_to_root(self) -> list[NDArray[np.float64]]:
        """Get path from this node to root."""
        path = []
        node: _RRTStarNode | None = self
        while node is not None:
            path.append(node.config)
            node = node.parent
        return list(reversed(path))


# ============= Result Helpers =============


def _create_success_result(
    path: list[NDArray[np.float64]],
    planning_time: float,
    iterations: int,
) -> PlanningResult:
    """Create a successful planning result."""
    return PlanningResult(
        status=PlanningStatus.SUCCESS,
        path=path,
        planning_time=planning_time,
        path_length=compute_path_length(path),
        iterations=iterations,
        message="Path found",
    )


def _create_failure_result(
    status: PlanningStatus,
    message: str,
    planning_time: float = 0.0,
    iterations: int = 0,
) -> PlanningResult:
    """Create a failed planning result."""
    return PlanningResult(
        status=status,
        path=[],
        planning_time=planning_time,
        iterations=iterations,
        message=message,
    )
