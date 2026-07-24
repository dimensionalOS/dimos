# Copyright 2026 Dimensional Inc.
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

"""Selected planning-group joint-space projection helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.groups.identifiers import (
    is_global_joint_name,
    local_joint_name_from_global,
    make_global_joint_name,
)
from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.models import (
    JointPath,
    LocalModelJointName,
    RobotName,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass(frozen=True)
class SelectedRobotProjection:
    """Runtime state needed to project selected-group samples into one robot.

    This is not static model metadata like ``RobotModelConfig``. It captures the
    current planning context: the world robot ID, base/current positions used for
    non-selected joints, and joint-limit lookup tables for the selected-space
    planner.
    """

    robot_id: WorldRobotID
    robot_name: RobotName
    local_joint_names: list[LocalModelJointName]
    base_positions_by_local_name: dict[LocalModelJointName, float]
    lower_limits_by_local_name: dict[LocalModelJointName, float]
    upper_limits_by_local_name: dict[LocalModelJointName, float]


class SelectedJointSpace:
    """Projection adapter between selected global joints and full robot states."""

    def __init__(
        self,
        robot_projections: list[SelectedRobotProjection],
        selected_joint_names: list[str],
    ) -> None:
        self.robot_projections = robot_projections
        self.selected_joint_names = selected_joint_names

    @classmethod
    def from_world(
        cls,
        world: WorldSpec,
        selection: PlanningGroupSelection,
    ) -> SelectedJointSpace:
        return cls(
            robot_projections=_build_robot_projections(world, selection),
            selected_joint_names=list(selection.joint_names),
        )

    def joint_limits(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        projections_by_robot_name = {
            projection.robot_name: projection for projection in self.robot_projections
        }
        lower: list[float] = []
        upper: list[float] = []
        for global_name in self.selected_joint_names:
            robot_name, local_name = _split_selected_global_joint_name(
                global_name, projections_by_robot_name
            )
            projection = projections_by_robot_name[robot_name]
            lower.append(projection.lower_limits_by_local_name[local_name])
            upper.append(projection.upper_limits_by_local_name[local_name])
        return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)

    def config_collision_free(
        self,
        world: WorldSpec,
        selected_positions: NDArray[np.float64],
    ) -> bool:
        with world.scratch_context() as ctx:
            projected_states = self.project_config(selected_positions)
            for projection in self.robot_projections:
                world.set_joint_state(
                    ctx, projection.robot_id, projected_states[projection.robot_id]
                )
            return all(
                world.is_collision_free(ctx, projection.robot_id)
                for projection in self.robot_projections
            )

    def edge_collision_free(
        self,
        world: WorldSpec,
        start: NDArray[np.float64],
        end: NDArray[np.float64],
        step_size: float,
    ) -> bool:
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(np.ceil(distance / step_size)))
        for step in range(steps + 1):
            ratio = step / steps
            candidate = start + ratio * (end - start)
            if not self.config_collision_free(world, candidate):
                return False
        return True

    def project_config(
        self,
        selected_positions: NDArray[np.float64],
    ) -> dict[WorldRobotID, JointState]:
        selected_positions_by_global_name = dict(
            zip(self.selected_joint_names, selected_positions.tolist(), strict=True)
        )
        projected_states: dict[WorldRobotID, JointState] = {}
        for projection in self.robot_projections:
            positions: list[float] = []
            for local_name in projection.local_joint_names:
                global_name = make_global_joint_name(projection.robot_name, local_name)
                position = selected_positions_by_global_name.get(
                    global_name,
                    projection.base_positions_by_local_name[local_name],
                )
                positions.append(float(position))
            projected_states[projection.robot_id] = JointState(
                {"name": list(projection.local_joint_names), "position": positions}
            )
        return projected_states

    def simplify_path(
        self,
        world: WorldSpec,
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
            if self.edge_collision_free(world, start, end, collision_step_size):
                simplified = simplified[: i + 1] + simplified[j:]
        return simplified


def normalize_selection_target(
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


def _build_robot_projections(
    world: WorldSpec,
    selection: PlanningGroupSelection,
) -> list[SelectedRobotProjection]:
    robot_ids_by_name = _robot_ids_by_name(world, selection.robot_names)
    robot_projections: list[SelectedRobotProjection] = []
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
            robot_projections.append(
                SelectedRobotProjection(
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
    return robot_projections


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
    projections_by_robot_name: dict[RobotName, SelectedRobotProjection],
) -> tuple[RobotName, LocalModelJointName]:
    for robot_name, projection in projections_by_robot_name.items():
        try:
            local_name = local_joint_name_from_global(robot_name, global_name)
        except ValueError:
            continue
        if local_name not in projection.local_joint_names:
            raise ValueError(
                f"Selected joint '{global_name}' is not configured for robot '{robot_name}'"
            )
        return robot_name, local_name
    raise ValueError(f"Selected joint '{global_name}' does not belong to a selected robot")
