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

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.groups.identifiers import (
    local_joint_name_from_global,
    make_global_joint_names,
)
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import (
    CartesianDelta,
    CartesianPathMode,
    JointPath,
    PlanningGroupID,
    PlanningResult,
    RobotName,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
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
    ) -> None:
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

        if world.check_edge_collision_free(robot_id, start, goal, self._collision_step_size):
            return _create_success_result(
                [start, goal],
                time.time() - start_time,
                0,
            )

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

    def plan_selected_joint_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a collision-free path for an explicit planning-group selection."""
        selected_joint_names = [
            joint_name for group in selection.groups for joint_name in group.joint_names
        ]
        exact_error = _validate_exact_joint_keys(start, selected_joint_names, "start")
        if exact_error is not None:
            return exact_error
        exact_error = _validate_exact_joint_keys(goal, selected_joint_names, "goal")
        if exact_error is not None:
            return exact_error

        try:
            robot_ids_by_name = _robot_ids_by_name(world, selection.robot_names)
        except (KeyError, ValueError) as exc:
            return _create_failure_result(PlanningStatus.INVALID_GOAL, str(exc))

        robot_ids = set(robot_ids_by_name.values())
        if len(robot_ids) != 1:
            return self._plan_multi_robot_selected_joint_path(
                world=world,
                groups=selection.groups,
                robot_ids_by_name=robot_ids_by_name,
                start=start,
                goal=goal,
                timeout=timeout,
            )

        robot_id = next(iter(robot_ids))
        robot_config = world.get_robot_config(robot_id)
        full_global_joint_names = make_global_joint_names(
            robot_config.name, robot_config.joint_names
        )
        if selected_joint_names != full_global_joint_names:
            return _create_failure_result(
                PlanningStatus.UNSUPPORTED,
                "RRTConnectPlanner currently requires the selected groups to cover "
                "the robot controllable joint set exactly",
            )

        local_start = _global_joint_state_to_local(
            start,
            robot_config.name,
            list(robot_config.joint_names),
            selected_joint_names,
        )
        local_goal = _global_joint_state_to_local(
            goal,
            robot_config.name,
            list(robot_config.joint_names),
            selected_joint_names,
        )
        result = self.plan_joint_path(
            world=world,
            robot_id=robot_id,
            start=local_start,
            goal=local_goal,
            timeout=timeout,
        )
        if not result.is_success():
            return result
        return PlanningResult(
            status=result.status,
            path=_local_path_to_global(result.path, robot_config.name, selected_joint_names),
            planning_time=result.planning_time,
            path_length=result.path_length,
            iterations=result.iterations,
            message=result.message,
            timestamps=result.timestamps,
        )

    def plan_cartesian_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        pose_targets: Mapping[PlanningGroupID, PoseStamped],
        *,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        path_mode: CartesianPathMode = "free",
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Return explicit unsupported status for absolute Cartesian requests."""
        _ = (world, selection, start, pose_targets, auxiliary_groups, path_mode, timeout)
        return _create_failure_result(
            PlanningStatus.UNSUPPORTED,
            "Cartesian planning is not supported by this planner",
        )

    def plan_relative_cartesian_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        delta_targets: Mapping[PlanningGroupID, CartesianDelta],
        *,
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        path_mode: CartesianPathMode = "free",
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Return explicit unsupported status for relative Cartesian requests."""
        _ = (world, selection, start, delta_targets, auxiliary_groups, path_mode, timeout)
        return _create_failure_result(
            PlanningStatus.UNSUPPORTED,
            "Cartesian planning is not supported by this planner",
        )

    def _plan_multi_robot_selected_joint_path(
        self,
        world: WorldSpec,
        groups: tuple[PlanningGroup, ...],
        robot_ids_by_name: dict[RobotName, WorldRobotID],
        start: JointState,
        goal: JointState,
        timeout: float,
    ) -> PlanningResult:
        """Plan over one coupled configuration vector for all selected robots."""
        start_time = time.time()

        if not world.is_finalized:
            return _create_failure_result(
                PlanningStatus.NO_SOLUTION,
                "World must be finalized before planning",
            )

        selected_joint_names = [joint for group in groups for joint in group.joint_names]
        q_start = np.array(
            _order_joint_state(start, selected_joint_names).position, dtype=np.float64
        )
        q_goal = np.array(_order_joint_state(goal, selected_joint_names).position, dtype=np.float64)

        try:
            robot_order, robot_joint_names = _validate_full_robot_groups(
                world, groups, robot_ids_by_name
            )
        except KeyError as exc:
            return _create_failure_result(PlanningStatus.NO_SOLUTION, str(exc))
        if not robot_order:
            return _create_failure_result(
                PlanningStatus.INVALID_GOAL, "No planning groups selected"
            )

        unsupported = _validate_selected_groups_cover_full_robots(
            world, robot_order, robot_joint_names
        )
        if unsupported is not None:
            return unsupported

        lower, upper = _combined_joint_limits(world, robot_order)

        if not _coupled_config_collision_free(
            world, robot_order, robot_joint_names, selected_joint_names, q_start
        ):
            return _create_failure_result(
                PlanningStatus.COLLISION_AT_START,
                "Start configuration is in collision",
            )
        if not _coupled_config_collision_free(
            world, robot_order, robot_joint_names, selected_joint_names, q_goal
        ):
            return _create_failure_result(
                PlanningStatus.COLLISION_AT_GOAL,
                "Goal configuration is in collision",
            )

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

        if _coupled_edge_collision_free(
            world,
            robot_order,
            robot_joint_names,
            selected_joint_names,
            q_start,
            q_goal,
            self._collision_step_size,
        ):
            return _create_success_result(
                [start, goal],
                time.time() - start_time,
                0,
            )

        start_tree = [TreeNode(config=q_start.copy())]
        goal_tree = [TreeNode(config=q_goal.copy())]
        trees_swapped = False

        max_iterations = 5000
        for iteration in range(max_iterations):
            if time.time() - start_time > timeout:
                return _create_failure_result(
                    PlanningStatus.TIMEOUT,
                    f"Timeout after {iteration} iterations",
                    time.time() - start_time,
                    iteration,
                )

            sample = np.random.uniform(lower, upper)
            extended = self._extend_coupled_tree(
                world,
                robot_order,
                robot_joint_names,
                start_tree,
                sample,
                self._step_size,
                selected_joint_names,
            )

            if extended is not None:
                connected = self._connect_coupled_tree(
                    world,
                    robot_order,
                    robot_joint_names,
                    goal_tree,
                    extended.config,
                    self._connect_step_size,
                    selected_joint_names,
                )
                if connected is not None:
                    path = self._extract_path(extended, connected, selected_joint_names)
                    if trees_swapped:
                        path = list(reversed(path))
                    path = _simplify_coupled_path(
                        world,
                        robot_order,
                        robot_joint_names,
                        path,
                        self._collision_step_size,
                    )
                    return _create_success_result(path, time.time() - start_time, iteration + 1)

            start_tree, goal_tree = goal_tree, start_tree
            trees_swapped = not trees_swapped

        return _create_failure_result(
            PlanningStatus.NO_SOLUTION,
            f"No path found after {max_iterations} iterations",
            time.time() - start_time,
            max_iterations,
        )

    def _extend_coupled_tree(
        self,
        world: WorldSpec,
        robot_order: list[WorldRobotID],
        robot_joint_names: dict[WorldRobotID, list[str]],
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
        selected_joint_names: list[str],
    ) -> TreeNode | None:
        """Extend a tree in the coupled selected-joint configuration space."""
        nearest = min(tree, key=lambda node: float(np.linalg.norm(node.config - target)))
        diff = target - nearest.config
        dist = float(np.linalg.norm(diff))
        if dist <= step_size:
            new_config = target.copy()
        else:
            new_config = nearest.config + step_size * (diff / dist)

        if _coupled_edge_collision_free(
            world,
            robot_order,
            robot_joint_names,
            selected_joint_names,
            nearest.config,
            new_config,
            self._collision_step_size,
        ):
            new_node = TreeNode(config=new_config, parent=nearest)
            nearest.children.append(new_node)
            tree.append(new_node)
            return new_node
        return None

    def _connect_coupled_tree(
        self,
        world: WorldSpec,
        robot_order: list[WorldRobotID],
        robot_joint_names: dict[WorldRobotID, list[str]],
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
        selected_joint_names: list[str],
    ) -> TreeNode | None:
        """Try to connect a coupled tree to a target configuration."""
        while True:
            result = self._extend_coupled_tree(
                world,
                robot_order,
                robot_joint_names,
                tree,
                target,
                step_size,
                selected_joint_names,
            )
            if result is None:
                return None
            if float(np.linalg.norm(result.config - target)) < self._goal_tolerance:
                return result

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

        # Check limits with small tolerance for driver floating-point drift
        lower, upper = world.get_joint_limits(robot_id)
        q_start = np.array(start.position, dtype=np.float64)
        q_goal = np.array(goal.position, dtype=np.float64)
        limit_eps = 1e-3  # ~0.06 degrees

        if np.any(q_start < lower - limit_eps) or np.any(q_start > upper + limit_eps):
            return _create_failure_result(
                PlanningStatus.INVALID_START,
                "Start configuration is outside joint limits",
            )

        if np.any(q_goal < lower - limit_eps) or np.any(q_goal > upper + limit_eps):
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


# Result Helpers


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


def _validate_full_robot_groups(
    world: WorldSpec,
    groups: tuple[PlanningGroup, ...],
    robot_ids_by_name: dict[RobotName, WorldRobotID],
) -> tuple[list[WorldRobotID], dict[WorldRobotID, list[str]]]:
    robot_order: list[WorldRobotID] = []
    robot_joint_names: dict[WorldRobotID, list[str]] = {}
    known_robot_ids = set(world.get_robot_ids())
    for group in groups:
        robot_id = robot_ids_by_name[group.robot_name]
        if robot_id not in known_robot_ids:
            raise KeyError(f"Robot '{robot_id}' not found")
        if robot_id not in robot_joint_names:
            robot_joint_names[robot_id] = []
            robot_order.append(robot_id)
        robot_joint_names[robot_id].extend(group.joint_names)
    return robot_order, robot_joint_names


def _robot_ids_by_name(
    world: WorldSpec, robot_names: tuple[RobotName, ...]
) -> dict[RobotName, WorldRobotID]:
    robot_ids_by_name: dict[RobotName, WorldRobotID] = {}
    for robot_name in robot_names:
        matches = [
            robot_id
            for robot_id in world.get_robot_ids()
            if world.get_robot_config(robot_id).name == robot_name
        ]
        if not matches:
            raise KeyError(f"Robot '{robot_name}' not found")
        if len(matches) > 1:
            raise ValueError(f"Robot name '{robot_name}' is not unique in planning world")
        robot_ids_by_name[robot_name] = matches[0]
    return robot_ids_by_name


def _validate_selected_groups_cover_full_robots(
    world: WorldSpec,
    robot_order: list[WorldRobotID],
    robot_joint_names: dict[WorldRobotID, list[str]],
) -> PlanningResult | None:
    for robot_id in robot_order:
        robot_config = world.get_robot_config(robot_id)
        full_global_joint_names = make_global_joint_names(
            robot_config.name, robot_config.joint_names
        )
        if robot_joint_names[robot_id] != full_global_joint_names:
            return _create_failure_result(
                PlanningStatus.UNSUPPORTED,
                "RRTConnectPlanner currently requires selected groups to cover "
                "each affected robot's controllable joint set exactly",
            )
    return None


def _combined_joint_limits(
    world: WorldSpec,
    robot_order: list[WorldRobotID],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    lower_parts: list[NDArray[np.float64]] = []
    upper_parts: list[NDArray[np.float64]] = []
    for robot_id in robot_order:
        lower, upper = world.get_joint_limits(robot_id)
        lower_parts.append(lower)
        upper_parts.append(upper)
    return np.concatenate(lower_parts), np.concatenate(upper_parts)


def _robot_joint_state_from_combined(
    combined_joint_names: list[str],
    combined_positions: NDArray[np.float64],
    robot_name: str,
    robot_joint_names: list[str],
) -> JointState:
    position_by_name = dict(zip(combined_joint_names, combined_positions.tolist(), strict=True))
    return JointState(
        name=[local_joint_name_from_global(robot_name, name) for name in robot_joint_names],
        position=[position_by_name[name] for name in robot_joint_names],
    )


def _global_joint_state_to_local(
    joint_state: JointState,
    robot_name: str,
    robot_joint_names: list[str],
    global_joint_names: list[str],
) -> JointState:
    position_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    local_joint_names = [
        local_joint_name_from_global(robot_name, name) for name in global_joint_names
    ]
    if local_joint_names != robot_joint_names:
        raise ValueError("Global selected joints do not match robot joint order")
    return JointState(
        name=robot_joint_names,
        position=[position_by_name[global_name] for global_name in global_joint_names],
    )


def _local_path_to_global(
    path: JointPath,
    robot_name: str,
    global_joint_names: list[str],
) -> JointPath:
    local_joint_names = [
        local_joint_name_from_global(robot_name, name) for name in global_joint_names
    ]
    global_path: JointPath = []
    for waypoint in path:
        position_by_name = dict(zip(waypoint.name, waypoint.position, strict=True))
        global_path.append(
            JointState(
                name=global_joint_names,
                position=[position_by_name[local_name] for local_name in local_joint_names],
            )
        )
    return global_path


def _coupled_config_collision_free(
    world: WorldSpec,
    robot_order: list[WorldRobotID],
    robot_joint_names: dict[WorldRobotID, list[str]],
    selected_joint_names: list[str],
    q: NDArray[np.float64],
) -> bool:
    with world.scratch_context() as ctx:
        for robot_id in robot_order:
            world.set_joint_state(
                ctx,
                robot_id,
                _robot_joint_state_from_combined(
                    selected_joint_names,
                    q,
                    world.get_robot_config(robot_id).name,
                    robot_joint_names[robot_id],
                ),
            )
        return all(world.is_collision_free(ctx, robot_id) for robot_id in robot_order)


def _coupled_edge_collision_free(
    world: WorldSpec,
    robot_order: list[WorldRobotID],
    robot_joint_names: dict[WorldRobotID, list[str]],
    selected_joint_names: list[str],
    q_start: NDArray[np.float64],
    q_end: NDArray[np.float64],
    step_size: float,
) -> bool:
    dist = float(np.linalg.norm(q_end - q_start))
    if dist < 1e-8:
        return _coupled_config_collision_free(
            world,
            robot_order,
            robot_joint_names,
            selected_joint_names,
            q_start,
        )

    n_steps = max(2, int(np.ceil(dist / step_size)) + 1)
    with world.scratch_context() as ctx:
        for i in range(n_steps):
            t = i / (n_steps - 1)
            q = q_start + t * (q_end - q_start)
            for robot_id in robot_order:
                world.set_joint_state(
                    ctx,
                    robot_id,
                    _robot_joint_state_from_combined(
                        selected_joint_names,
                        q,
                        world.get_robot_config(robot_id).name,
                        robot_joint_names[robot_id],
                    ),
                )
            if not all(world.is_collision_free(ctx, robot_id) for robot_id in robot_order):
                return False
    return True


def _simplify_coupled_path(
    world: WorldSpec,
    robot_order: list[WorldRobotID],
    robot_joint_names: dict[WorldRobotID, list[str]],
    path: JointPath,
    collision_step_size: float,
    max_iterations: int = 100,
) -> JointPath:
    if len(path) <= 2:
        return path

    simplified = list(path)
    selected_joint_names = list(path[0].name)
    for _ in range(max_iterations):
        if len(simplified) <= 2:
            break
        i = np.random.randint(0, len(simplified) - 2)
        j = np.random.randint(i + 2, len(simplified))
        q_start = np.array(simplified[i].position, dtype=np.float64)
        q_end = np.array(simplified[j].position, dtype=np.float64)
        if _coupled_edge_collision_free(
            world,
            robot_order,
            robot_joint_names,
            selected_joint_names,
            q_start,
            q_end,
            collision_step_size,
        ):
            simplified = simplified[: i + 1] + simplified[j:]
    return simplified


def _validate_exact_joint_keys(
    joint_state: JointState, selected_joint_names: list[str], state_name: str
) -> PlanningResult | None:
    actual_names = list(joint_state.name)
    expected_names = selected_joint_names
    if set(actual_names) != set(expected_names):
        missing = [name for name in expected_names if name not in actual_names]
        extra = [name for name in actual_names if name not in expected_names]
        details: list[str] = []
        if missing:
            details.append(f"missing={missing}")
        if extra:
            details.append(f"extra={extra}")
        return _create_failure_result(
            PlanningStatus.INVALID_START if state_name == "start" else PlanningStatus.INVALID_GOAL,
            f"{state_name} joint names must exactly match selected joints ({', '.join(details)})",
        )
    if len(joint_state.position) != len(joint_state.name):
        return _create_failure_result(
            PlanningStatus.INVALID_START if state_name == "start" else PlanningStatus.INVALID_GOAL,
            f"{state_name} joint name and position lengths must match",
        )
    return None


def _order_joint_state(joint_state: JointState, joint_names: list[str]) -> JointState:
    position_by_name = dict(zip(joint_state.name, joint_state.position, strict=False))
    return JointState(
        name=joint_names,
        position=[position_by_name[name] for name in joint_names],
    )
