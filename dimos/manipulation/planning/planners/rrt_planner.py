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

from dimos.manipulation.planning.groups.identifiers import (
    is_global_joint_name,
    local_joint_name_from_global,
    make_global_joint_name,
)
from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import (
    JointPath,
    LocalModelJointName,
    PlanningResult,
    RobotName,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.path_utils import compute_path_length
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


@dataclass(frozen=True)
class _SelectedRobotState:
    robot_id: WorldRobotID
    robot_name: RobotName
    local_joint_names: list[LocalModelJointName]
    base_positions_by_local_name: dict[LocalModelJointName, float]
    lower_limits_by_local_name: dict[LocalModelJointName, float]
    upper_limits_by_local_name: dict[LocalModelJointName, float]


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
            return _create_success_result([start, goal], time.time() - start_time, 0)

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
        max_iterations: int = 5000,
    ) -> PlanningResult:
        """Plan over an explicit planning-group selection.

        The search space is the selected global-joint order. Collision checks project
        candidates into full per-robot states, holding unselected joints at the world
        current state.
        """
        start_time = time.time()
        if not world.is_finalized:
            return _create_failure_result(
                PlanningStatus.NO_SOLUTION,
                "World must be finalized before planning",
            )

        if not selection.groups:
            return _create_failure_result(
                PlanningStatus.INVALID_GOAL, "No planning groups selected"
            )

        selected_joint_names = list(selection.joint_names)
        try:
            normalized_start = _normalize_selection_target(selection, start, "start")
        except ValueError as exc:
            return _create_failure_result(PlanningStatus.INVALID_START, str(exc))
        try:
            normalized_goal = _normalize_selection_target(selection, goal, "goal")
        except ValueError as exc:
            return _create_failure_result(PlanningStatus.INVALID_GOAL, str(exc))
        try:
            robot_states = _build_selected_robot_states(world, selection)
            q_start = np.asarray(normalized_start.position, dtype=np.float64)
            q_goal = np.asarray(normalized_goal.position, dtype=np.float64)
            lower, upper = _selected_joint_limits(robot_states, selected_joint_names)
        except ValueError as exc:
            return _create_failure_result(PlanningStatus.NO_SOLUTION, str(exc))

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

        if not _selected_config_collision_free(world, robot_states, selected_joint_names, q_start):
            return _create_failure_result(
                PlanningStatus.COLLISION_AT_START,
                "Start configuration is in collision",
            )
        if not _selected_config_collision_free(world, robot_states, selected_joint_names, q_goal):
            return _create_failure_result(
                PlanningStatus.COLLISION_AT_GOAL,
                "Goal configuration is in collision",
            )

        if _selected_edge_collision_free(
            world,
            robot_states,
            selected_joint_names,
            q_start,
            q_goal,
            self._collision_step_size,
        ):
            return _create_success_result(
                [normalized_start, normalized_goal], time.time() - start_time, 0
            )

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
            extended = self._extend_selected_tree(
                world,
                robot_states,
                selected_joint_names,
                start_tree,
                sample,
                self._step_size,
            )
            if extended is not None:
                connected = self._connect_selected_tree(
                    world,
                    robot_states,
                    selected_joint_names,
                    goal_tree,
                    extended.config,
                    self._connect_step_size,
                )
                if connected is not None:
                    path = self._extract_path(extended, connected, selected_joint_names)
                    if trees_swapped:
                        path = list(reversed(path))
                    path = _simplify_selected_path(
                        world,
                        robot_states,
                        selected_joint_names,
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

    def _extend_selected_tree(
        self,
        world: WorldSpec,
        robot_states: list[_SelectedRobotState],
        selected_joint_names: list[str],
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
    ) -> TreeNode | None:
        """Extend a tree in selected-joint space."""
        nearest = min(tree, key=lambda node: float(np.linalg.norm(node.config - target)))
        diff = target - nearest.config
        dist = float(np.linalg.norm(diff))
        if dist <= step_size:
            new_config = target.copy()
        else:
            new_config = nearest.config + step_size * (diff / dist)

        if _selected_edge_collision_free(
            world,
            robot_states,
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

    def _connect_selected_tree(
        self,
        world: WorldSpec,
        robot_states: list[_SelectedRobotState],
        selected_joint_names: list[str],
        tree: list[TreeNode],
        target: NDArray[np.float64],
        step_size: float,
    ) -> TreeNode | None:
        """Try to connect a selected-joint tree to a target."""
        while True:
            result = self._extend_selected_tree(
                world,
                robot_states,
                selected_joint_names,
                tree,
                target,
                step_size,
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
        start_state = JointState({"name": joint_names, "position": nearest.config.tolist()})
        end_state = JointState({"name": joint_names, "position": new_config.tolist()})
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
        return [JointState({"name": joint_names, "position": q.tolist()}) for q in full_path_arrays]

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


def _normalize_selection_target(
    selection: PlanningGroupSelection,
    target: JointState,
    label: str,
) -> JointState:
    """Normalize a selected-joint target to global selection order."""
    selected_global_names = list(selection.joint_names)
    if not target.name:
        if len(target.position) != len(selected_global_names):
            raise ValueError(
                f"{label} target has {len(target.position)} positions, "
                f"expected {len(selected_global_names)}"
            )
        return JointState({"name": selected_global_names, "position": list(target.position)})

    if len(target.name) != len(target.position):
        raise ValueError(
            f"{label} target has {len(target.name)} names but {len(target.position)} positions"
        )

    names = list(target.name)
    global_flags = [is_global_joint_name(name) for name in names]
    if any(global_flags) and not all(global_flags):
        raise ValueError(f"{label} target mixes global and local joint names: {names}")

    if all(global_flags):
        expected_names = selected_global_names
    else:
        if len(selection.groups) != 1:
            raise ValueError(
                f"{label} target uses local joint names for a multi-group selection; "
                "use global joint names"
            )
        expected_names = list(selection.groups[0].local_joint_names)

    positions_by_name = dict(zip(names, target.position, strict=True))
    missing = [name for name in expected_names if name not in positions_by_name]
    if missing:
        raise ValueError(f"{label} target is missing joints: {missing}")
    extra = sorted(set(names) - set(expected_names))
    if extra:
        raise ValueError(f"{label} target has extra joints: {extra}")

    ordered_positions = [float(positions_by_name[name]) for name in expected_names]
    return JointState({"name": selected_global_names, "position": ordered_positions})


def _build_selected_robot_states(
    world: WorldSpec,
    selection: PlanningGroupSelection,
) -> list[_SelectedRobotState]:
    robot_ids_by_name = _robot_ids_by_name(world, selection.robot_names)
    selected_robot_states: list[_SelectedRobotState] = []
    with world.scratch_context() as ctx:
        for robot_name in selection.robot_names:
            robot_id = robot_ids_by_name[robot_name]
            config = world.get_robot_config(robot_id)
            local_joint_names = list(config.joint_names)
            current_state = world.get_joint_state(ctx, robot_id)
            base_positions_by_local_name = _positions_by_local_name(
                current_state,
                robot_name,
                local_joint_names,
            )
            lower, upper = world.get_joint_limits(robot_id)
            if len(lower) != len(local_joint_names) or len(upper) != len(local_joint_names):
                raise ValueError(
                    f"Robot '{robot_name}' joint limits do not match configured joints"
                )
            selected_robot_states.append(
                _SelectedRobotState(
                    robot_id=robot_id,
                    robot_name=robot_name,
                    local_joint_names=local_joint_names,
                    base_positions_by_local_name=base_positions_by_local_name,
                    lower_limits_by_local_name=dict(
                        zip(local_joint_names, lower.tolist(), strict=True)
                    ),
                    upper_limits_by_local_name=dict(
                        zip(local_joint_names, upper.tolist(), strict=True)
                    ),
                )
            )
    return selected_robot_states


def _positions_by_local_name(
    joint_state: JointState,
    robot_name: RobotName,
    local_joint_names: list[LocalModelJointName],
) -> dict[LocalModelJointName, float]:
    if not joint_state.name:
        if len(joint_state.position) != len(local_joint_names):
            raise ValueError(
                f"Current state for robot '{robot_name}' has {len(joint_state.position)} positions, "
                f"expected {len(local_joint_names)}"
            )
        return dict(zip(local_joint_names, map(float, joint_state.position), strict=True))

    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    positions_by_local_name: dict[LocalModelJointName, float] = {}
    for local_name in local_joint_names:
        global_name = make_global_joint_name(robot_name, local_name)
        if local_name in positions_by_name:
            positions_by_local_name[local_name] = float(positions_by_name[local_name])
        elif global_name in positions_by_name:
            positions_by_local_name[local_name] = float(positions_by_name[global_name])
        else:
            raise ValueError(
                f"Current state for robot '{robot_name}' is missing joint '{local_name}'"
            )
    return positions_by_local_name


def _selected_joint_limits(
    robot_states: list[_SelectedRobotState],
    selected_joint_names: list[str],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    states_by_robot_name = {state.robot_name: state for state in robot_states}
    lower: list[float] = []
    upper: list[float] = []
    for global_name in selected_joint_names:
        robot_name, local_name = _split_selected_global_joint_name(
            global_name, states_by_robot_name
        )
        state = states_by_robot_name[robot_name]
        lower.append(state.lower_limits_by_local_name[local_name])
        upper.append(state.upper_limits_by_local_name[local_name])
    return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)


def _selected_config_collision_free(
    world: WorldSpec,
    robot_states: list[_SelectedRobotState],
    selected_joint_names: list[str],
    selected_positions: NDArray[np.float64],
) -> bool:
    with world.scratch_context() as ctx:
        projected_states = _project_selected_config(
            robot_states, selected_joint_names, selected_positions
        )
        for robot_state in robot_states:
            world.set_joint_state(ctx, robot_state.robot_id, projected_states[robot_state.robot_id])
        return all(
            world.is_collision_free(ctx, robot_state.robot_id) for robot_state in robot_states
        )


def _selected_edge_collision_free(
    world: WorldSpec,
    robot_states: list[_SelectedRobotState],
    selected_joint_names: list[str],
    start: NDArray[np.float64],
    end: NDArray[np.float64],
    step_size: float,
) -> bool:
    distance = float(np.linalg.norm(end - start))
    steps = max(1, int(np.ceil(distance / step_size)))
    for step in range(steps + 1):
        ratio = step / steps
        candidate = start + ratio * (end - start)
        if not _selected_config_collision_free(
            world, robot_states, selected_joint_names, candidate
        ):
            return False
    return True


def _project_selected_config(
    robot_states: list[_SelectedRobotState],
    selected_joint_names: list[str],
    selected_positions: NDArray[np.float64],
) -> dict[WorldRobotID, JointState]:
    selected_positions_by_global_name = dict(
        zip(selected_joint_names, selected_positions.tolist(), strict=True)
    )
    projected_states: dict[WorldRobotID, JointState] = {}
    for robot_state in robot_states:
        positions: list[float] = []
        for local_name in robot_state.local_joint_names:
            global_name = make_global_joint_name(robot_state.robot_name, local_name)
            position = selected_positions_by_global_name.get(
                global_name,
                robot_state.base_positions_by_local_name[local_name],
            )
            positions.append(float(position))
        projected_states[robot_state.robot_id] = JointState(
            {"name": list(robot_state.local_joint_names), "position": positions}
        )
    return projected_states


def _simplify_selected_path(
    world: WorldSpec,
    robot_states: list[_SelectedRobotState],
    selected_joint_names: list[str],
    path: JointPath,
    collision_step_size: float,
    max_iterations: int = 100,
) -> JointPath:
    if len(path) <= 2:
        return path

    simplified = list(path)
    for _ in range(max_iterations):
        if len(simplified) <= 2:
            break
        i = np.random.randint(0, len(simplified) - 2)
        j = np.random.randint(i + 2, len(simplified))
        start = np.asarray(simplified[i].position, dtype=np.float64)
        end = np.asarray(simplified[j].position, dtype=np.float64)
        if _selected_edge_collision_free(
            world,
            robot_states,
            selected_joint_names,
            start,
            end,
            collision_step_size,
        ):
            simplified = simplified[: i + 1] + simplified[j:]
    return simplified


def _robot_ids_by_name(
    world: WorldSpec,
    robot_names: tuple[RobotName, ...],
) -> dict[RobotName, WorldRobotID]:
    robot_ids_by_name: dict[RobotName, WorldRobotID] = {}
    for robot_name in robot_names:
        matches = [
            robot_id
            for robot_id in world.get_robot_ids()
            if world.get_robot_config(robot_id).name == robot_name
        ]
        if not matches:
            raise ValueError(f"Robot '{robot_name}' not found")
        if len(matches) > 1:
            raise ValueError(f"Robot name '{robot_name}' is not unique in planning world")
        robot_ids_by_name[robot_name] = matches[0]
    return robot_ids_by_name


def _split_selected_global_joint_name(
    global_name: str,
    states_by_robot_name: dict[RobotName, _SelectedRobotState],
) -> tuple[RobotName, LocalModelJointName]:
    for robot_name, state in states_by_robot_name.items():
        try:
            local_name = local_joint_name_from_global(robot_name, global_name)
        except ValueError:
            continue
        if local_name not in state.local_joint_names:
            raise ValueError(
                f"Selected joint '{global_name}' is not configured for robot '{robot_name}'"
            )
        return robot_name, local_name
    raise ValueError(f"Selected joint '{global_name}' does not belong to a selected robot")


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
