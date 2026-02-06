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

"""RRT-Connect and RRT* motion planners implementing PlannerSpec.

These planners are backend-agnostic - they only use WorldSpec methods and can work
with any physics backend (Drake, MuJoCo, PyBullet, etc.).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.spec import (
    JointPath,
    PlanningResult,
    PlanningStatus,
    WorldRobotID,
    WorldSpec,
)
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


@dataclass(eq=False)
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


class RRTConnectPlanner:
    """Bi-directional RRT-Connect planner.

    This planner is backend-agnostic - it only uses WorldSpec methods for
    collision checking and can work with any physics backend.
    """

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
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
        max_iterations: int = 5000,
    ) -> PlanningResult:
        """Plan collision-free path using bi-directional RRT."""
        start_time = time.time()

        # Extract positions as numpy arrays for internal computation
        q_start = np.array(start.position, dtype=np.float64)
        q_goal = np.array(goal.position, dtype=np.float64)
        joint_names = start.name  # Store for converting back to JointState

        error = self._validate_inputs(world, robot_id, start, goal)
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
            extended = self._extend_tree(
                world, robot_id, start_tree, sample, self._step_size, joint_names
            )

            if extended is not None:
                connected = self._connect_tree(
                    world,
                    robot_id,
                    goal_tree,
                    extended.config,
                    self._connect_step_size,
                    joint_names,
                )
                if connected is not None:
                    path = self._extract_path(extended, connected, joint_names)
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
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
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

        # Check start validity using context-free method
        if not world.check_config_collision_free(robot_id, start):
            return _create_failure_result(
                PlanningStatus.COLLISION_AT_START,
                "Start configuration is in collision",
            )

        # Check goal validity using context-free method
        if not world.check_config_collision_free(robot_id, goal):
            return _create_failure_result(
                PlanningStatus.COLLISION_AT_GOAL,
                "Goal configuration is in collision",
            )

        # Check limits (extract arrays for numpy comparison)
        lower, upper = world.get_joint_limits(robot_id)
        q_start = np.array(start.position, dtype=np.float64)
        q_goal = np.array(goal.position, dtype=np.float64)

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
        robot_id: WorldRobotID,
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
        joint_names: list[str],
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

        # Check validity of edge using context-free method
        start_state = JointState(name=joint_names, position=nearest.config.tolist())
        end_state = JointState(name=joint_names, position=new_config.tolist())
        if world.check_edge_collision_free(
            robot_id, start_state, end_state, self._collision_step_size
        ):
            new_node = TreeNode(config=new_config, parent=nearest)
            nearest.children.append(new_node)
            tree.append(new_node)
            return new_node

        return None

    def _connect_tree(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
        joint_names: list[str],
    ) -> TreeNode | None:
        """Try to connect tree to target, returns connected node if successful."""
        # Keep extending toward target
        while True:
            result = self._extend_tree(world, robot_id, tree, target, step_size, joint_names)

            if result is None:
                return None  # Extension failed

            # Check if reached target
            if float(np.linalg.norm(result.config - target)) < self._goal_tolerance:
                return result

    def _extract_path(
        self,
        start_node: TreeNode,
        goal_node: TreeNode,
        joint_names: list[str],
    ) -> JointPath:
        """Extract path from two connected nodes."""
        # Path from start node to its root (reversed to be root->node)
        start_path = start_node.path_to_root()

        # Path from goal node to its root
        goal_path = goal_node.path_to_root()

        # Combine: start_root -> start_node -> goal_node -> goal_root
        # But we need start -> goal, so reverse the goal path
        full_path_arrays = start_path + list(reversed(goal_path))

        # Convert to list of JointState
        return [JointState(name=joint_names, position=q.tolist()) for q in full_path_arrays]

    def _simplify_path(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        path: JointPath,
        max_iterations: int = 100,
    ) -> JointPath:
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

            # Check if direct connection is valid using context-free method
            # path elements are already JointState
            if world.check_edge_collision_free(
                robot_id, simplified[i], simplified[j], self._collision_step_size
            ):
                # Remove intermediate waypoints
                simplified = simplified[: i + 1] + simplified[j:]

        return simplified


# ============= Result Helpers =============


def _create_success_result(
    path: JointPath,
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
