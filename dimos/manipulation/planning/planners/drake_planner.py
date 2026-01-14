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

"""RRT-Connect and RRT* motion planners implementing PlannerSpec."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.spec import PlanningResult, PlanningStatus, WorldSpec
from dimos.manipulation.planning.utils.path_utils import (
    compute_path_length,
    interpolate_segment,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


@dataclass
class TreeNode:
    """Node in RRT tree with optional cost tracking (for RRT*)."""

    config: NDArray[np.float64]
    parent: TreeNode | None = None
    children: list[TreeNode] = field(default_factory=list)
    cost: float = 0.0

    def path_to_root(self) -> list[NDArray[np.float64]]:
        """Get path from this node to root."""
        path = []
        node: TreeNode | None = self
        while node is not None:
            path.append(node.config)
            node = node.parent
        return list(reversed(path))


class DrakePlanner:
    """Bi-directional RRT-Connect planner."""

    def __init__(
        self,
        step_size: float = 0.1,
        connect_step_size: float = 0.05,
        goal_tolerance: float = 0.1,
        collision_step_size: float = 0.02,
    ):
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
        """Plan collision-free path using bi-directional RRT."""
        start_time = time.time()

        error = self._validate_inputs(world, robot_id, q_start, q_goal)
        if error is not None:
            return error

        lower, upper = world.get_joint_limits(robot_id)
        start_tree = [TreeNode(config=q_start.copy())]
        goal_tree = [TreeNode(config=q_goal.copy())]
        trees_swapped = False

        for iteration in range(max_iterations):
            if time.time() - start_time > timeout:
                return _create_failure_result(
                    PlanningStatus.TIMEOUT,
                    f"Timeout after {iteration} iterations",
                    time.time() - start_time,
                    iteration,
                )

            sample = np.random.uniform(lower, upper)
            extended = self._extend_tree(world, robot_id, start_tree, sample, self._step_size)

            if extended is not None:
                connected = self._connect_tree(
                    world, robot_id, goal_tree, extended.config, self._connect_step_size
                )
                if connected is not None:
                    path = self._extract_path(extended, connected)
                    if trees_swapped:
                        path = list(reversed(path))
                    path = self._simplify_path(world, robot_id, path)
                    return _create_success_result(path, time.time() - start_time, iteration + 1)

            start_tree, goal_tree = goal_tree, start_tree
            trees_swapped = not trees_swapped

        return _create_failure_result(
            PlanningStatus.NO_SOLUTION,
            f"No path found after {max_iterations} iterations",
            time.time() - start_time,
            max_iterations,
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
        """Plan optimal path using RRT* with rewiring."""
        start_time = time.time()
        lower, upper = world.get_joint_limits(robot_id)

        # Validate start/goal
        with world.scratch_context() as ctx:
            world.set_positions(ctx, robot_id, q_start)
            if not world.is_collision_free(ctx, robot_id):
                return _create_failure_result(
                    PlanningStatus.COLLISION_AT_START, "Start in collision"
                )
            world.set_positions(ctx, robot_id, q_goal)
            if not world.is_collision_free(ctx, robot_id):
                return _create_failure_result(PlanningStatus.COLLISION_AT_GOAL, "Goal in collision")

        nodes = [TreeNode(config=q_start.copy(), cost=0.0)]
        goal_node: TreeNode | None = None
        best_cost = float("inf")
        iterations_run = 0

        for _ in range(max_iterations):
            iterations_run += 1
            if time.time() - start_time > timeout:
                break

            # Sample and find nearest
            sample = np.random.uniform(lower, upper)
            nearest = min(nodes, key=lambda n: float(np.linalg.norm(n.config - sample)))

            # Extend toward sample
            diff = sample - nearest.config
            dist = float(np.linalg.norm(diff))
            new_config = (
                nearest.config + min(dist, self._step_size) * (diff / dist)
                if dist > 0
                else sample.copy()
            )

            if not self._is_edge_valid(world, robot_id, nearest.config, new_config):
                continue

            # Find best parent among neighbors
            neighbors = [
                n
                for n in nodes
                if float(np.linalg.norm(n.config - new_config)) < self._rewire_radius
            ]
            best_parent, best_cost_to_new = (
                nearest,
                nearest.cost + float(np.linalg.norm(new_config - nearest.config)),
            )
            for n in neighbors:
                cost = n.cost + float(np.linalg.norm(new_config - n.config))
                if cost < best_cost_to_new and self._is_edge_valid(
                    world, robot_id, n.config, new_config
                ):
                    best_parent, best_cost_to_new = n, cost

            # Add node and rewire neighbors
            new_node = TreeNode(config=new_config, parent=best_parent, cost=best_cost_to_new)
            best_parent.children.append(new_node)
            nodes.append(new_node)
            self._rewire_neighbors(world, robot_id, new_node, neighbors)

            # Check goal
            if (
                float(np.linalg.norm(new_node.config - q_goal)) < self._goal_tolerance
                and new_node.cost < best_cost
            ):
                goal_node, best_cost = new_node, new_node.cost

        if goal_node is not None:
            path = [*goal_node.path_to_root(), q_goal]
            return _create_success_result(path, time.time() - start_time, iterations_run)

        return _create_failure_result(
            PlanningStatus.NO_SOLUTION,
            f"No path after {max_iterations} iterations",
            time.time() - start_time,
            max_iterations,
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

    def _rewire_neighbors(
        self,
        world: WorldSpec,
        robot_id: str,
        new_node: TreeNode,
        neighbors: list[TreeNode],
    ) -> None:
        """Rewire neighbors through new node if it provides a shorter path."""
        for neighbor in neighbors:
            if neighbor == new_node.parent:
                continue
            potential_cost = new_node.cost + float(
                np.linalg.norm(neighbor.config - new_node.config)
            )
            if potential_cost < neighbor.cost and self._is_edge_valid(
                world, robot_id, new_node.config, neighbor.config
            ):
                if neighbor.parent is not None:
                    neighbor.parent.children.remove(neighbor)
                neighbor.parent = new_node
                neighbor.cost = potential_cost
                new_node.children.append(neighbor)
                self._update_costs(neighbor)

    def _update_costs(self, node: TreeNode) -> None:
        """Recursively update costs after rewiring."""
        for child in node.children:
            child.cost = node.cost + float(np.linalg.norm(child.config - node.config))
            self._update_costs(child)


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
