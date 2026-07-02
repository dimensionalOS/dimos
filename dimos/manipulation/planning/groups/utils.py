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

"""Shared helpers for planning-group selectors and joint-state projection."""

from collections.abc import Mapping, Sequence

from dimos.manipulation.planning.groups.identifiers import (
    assert_global_joint_names,
    assert_local_joint_names,
    is_global_joint_name,
    make_global_joint_names,
    make_planning_group_id,
    parse_planning_group_id,
)
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import (
    GlobalJointName,
    LocalModelJointName,
    PlanningGroupID,
    RobotName,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def planning_group_id_from_selector(selector: PlanningGroupID | PlanningGroup) -> PlanningGroupID:
    """Return the planning-group ID represented by a selector."""
    if isinstance(selector, PlanningGroup):
        return selector.id
    return selector


def matching_global_joint_name(
    positions_by_name: Mapping[str, float], local_joint_name: LocalModelJointName
) -> GlobalJointName | None:
    """Find the unique global joint name ending with a local joint name."""
    suffix = f"/{local_joint_name}"
    matches = [name for name in positions_by_name if name.endswith(suffix)]
    if len(matches) == 1:
        return matches[0]
    logger.warning(
        f"Expected exactly one global joint ending with local joint name "
        f"'{local_joint_name}', found {len(matches)} matches: {matches}"
    )
    return None


def planning_group_from_configs(
    group_id: PlanningGroupID, configs: Sequence[RobotModelConfig]
) -> PlanningGroup:
    """Resolve a public planning-group ID from stored robot configs."""
    robot_name, group_name = parse_planning_group_id(group_id)
    config = _config_for_robot_name(robot_name, configs)
    definition = _definition_for_group_name(group_name, config.planning_groups, group_id)
    return planning_group_from_definition(config, definition)


def planning_group_from_definition(
    config: RobotModelConfig, definition: PlanningGroupDefinition
) -> PlanningGroup:
    """Build the public planning-group model for one config definition."""
    return PlanningGroup(
        id=make_planning_group_id(config.name, definition.name),
        robot_name=config.name,
        group_name=definition.name,
        joint_names=tuple(make_global_joint_names(config.name, definition.joint_names)),
        local_joint_names=definition.joint_names,
        base_link=definition.base_link,
        tip_link=definition.tip_link,
        source=definition.source,
    )


def validate_planning_group_config(config: RobotModelConfig) -> None:
    """Validate config-derived planning groups without constructing a registry."""
    seen_group_ids: set[PlanningGroupID] = set()
    seen_group_names: set[str] = set()
    for definition in config.planning_groups:
        group_id = make_planning_group_id(config.name, definition.name)
        if definition.name in seen_group_names or group_id in seen_group_ids:
            raise ValueError(f"Planning group '{group_id}' is already registered")
        planning_group_from_definition(config, definition)
        seen_group_names.add(definition.name)
        seen_group_ids.add(group_id)


def primary_pose_group_id_for_config(config: RobotModelConfig) -> PlanningGroupID | None:
    """Return the unique pose-targetable group ID for robot-scoped wrappers."""
    pose_groups = [group for group in config.planning_groups if group.has_pose_target]
    if not pose_groups:
        return None
    if len(pose_groups) > 1:
        raise ValueError(
            f"Robot '{config.name}' has {len(pose_groups)} pose-targetable planning groups; "
            "use an explicit planning group ID"
        )
    return make_planning_group_id(config.name, pose_groups[0].name)


def full_robot_joint_state_from_input(
    config: RobotModelConfig, joint_state: JointState
) -> JointState | None:
    """Return a full robot-local state if input contains all robot joints."""
    if not joint_state.name:
        if len(joint_state.position) != len(config.joint_names):
            return None
        return JointState(
            {"name": list(config.joint_names), "position": list(joint_state.position)}
        )
    if len(joint_state.name) != len(joint_state.position):
        raise ValueError("JointState name and position lengths must match")
    resolved_positions: dict[str, float] = {}
    global_prefix = f"{config.name}/"
    for name, position in zip(joint_state.name, joint_state.position, strict=True):
        if name in config.joint_names:
            resolved_name = name
        elif name in config.joint_name_mapping:
            resolved_name = config.joint_name_mapping[name]
        elif name.startswith(global_prefix):
            resolved_name = name[len(global_prefix) :]
        else:
            resolved_name = config.get_urdf_joint_name(name)
        if resolved_name not in config.joint_names:
            return None
        if resolved_name in resolved_positions:
            raise ValueError(f"JointState resolves duplicate joint '{resolved_name}'")
        resolved_positions[resolved_name] = float(position)
    if set(resolved_positions) != set(config.joint_names):
        return None
    return JointState(
        {
            "name": list(config.joint_names),
            "position": [resolved_positions[name] for name in config.joint_names],
        }
    )


def joint_state_for_group_query(
    config: RobotModelConfig,
    group: PlanningGroup,
    current_state: JointState,
    joint_state: JointState | None,
) -> JointState:
    """Normalize full-robot or group-scoped input to full robot-local state."""
    if joint_state is None:
        return current_state
    full_state = full_robot_joint_state_from_input(config, joint_state)
    if full_state is not None:
        return full_state
    group_state = filter_joint_state_to_selected_joints(
        joint_state, group.joint_names, group.local_joint_names
    )
    positions_by_name = dict(zip(current_state.name, current_state.position, strict=True))
    for local_name, position in zip(group.local_joint_names, group_state.position, strict=True):
        if local_name not in positions_by_name:
            raise ValueError(f"Current state is missing group joint '{local_name}'")
        positions_by_name[local_name] = float(position)
    return JointState(
        {
            "name": list(current_state.name),
            "position": [positions_by_name[name] for name in current_state.name],
        }
    )


def _config_for_robot_name(
    robot_name: RobotName, configs: Sequence[RobotModelConfig]
) -> RobotModelConfig:
    for config in configs:
        if config.name == robot_name:
            return config
    raise KeyError(f"No robot registered for planning group robot '{robot_name}'")


def _definition_for_group_name(
    group_name: str,
    definitions: Sequence[PlanningGroupDefinition],
    group_id: PlanningGroupID,
) -> PlanningGroupDefinition:
    for definition in definitions:
        if definition.name == group_name:
            return definition
    raise KeyError(f"Unknown planning group ID: {group_id}")


def filter_joint_state_to_selected_joints(
    joint_state: JointState,
    global_joint_names: Sequence[GlobalJointName],
    local_joint_names: Sequence[LocalModelJointName] = (),
) -> JointState:
    """Project a joint state to selected global joints.

    Values are looked up by global name first. When ``local_joint_names`` is
    provided, each corresponding local name is used as a fallback.
    """
    if local_joint_names and len(global_joint_names) != len(local_joint_names):
        raise ValueError("Global and local selected joint lists must have the same length")

    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    selected_positions: list[float] = []
    missing: list[str] = []
    for index, global_name in enumerate(global_joint_names):
        if global_name in positions_by_name:
            selected_positions.append(float(positions_by_name[global_name]))
            continue
        if local_joint_names:
            local_name = local_joint_names[index]
            if local_name in positions_by_name:
                selected_positions.append(float(positions_by_name[local_name]))
                continue
        missing.append(global_name)

    if missing:
        raise ValueError(f"Joint state is missing selected joints: {missing}")

    return JointState({"name": list(global_joint_names), "position": selected_positions})


def joint_target_to_global_names(
    group: PlanningGroup,
    target: JointState,
) -> JointState:
    """Convert a group joint target to global joint names in group order.

    Named targets may use either the public global planning names or the
    robot-local model names used by legacy robot-scoped callers, but the two
    namespaces must not be mixed in one target.
    """
    if not target.name:
        if len(target.position) != len(group.joint_names):
            raise ValueError(
                f"Target for '{group.id}' has {len(target.position)} positions, "
                f"expected {len(group.joint_names)}"
            )
        return JointState({"name": list(group.joint_names), "position": list(target.position)})

    if len(target.name) != len(target.position):
        raise ValueError(
            f"Target for '{group.id}' has {len(target.name)} names but "
            f"{len(target.position)} positions"
        )

    target_names = list(target.name)
    global_flags = [is_global_joint_name(name) for name in target_names]
    if any(global_flags) and not all(global_flags):
        raise ValueError(
            f"Target for '{group.id}' mixes global and local joint names: {target_names}"
        )

    if all(global_flags):
        assert_global_joint_names(target_names)
        expected_names = group.joint_names
    else:
        assert_local_joint_names(target_names)
        expected_names = group.local_joint_names

    positions_by_name = dict(zip(target_names, target.position, strict=True))
    global_positions: list[float] = []
    missing: list[str] = []
    for expected_name in expected_names:
        if expected_name in positions_by_name:
            global_positions.append(positions_by_name[expected_name])
        else:
            missing.append(expected_name)
    if missing:
        raise ValueError(f"Target for '{group.id}' is missing joints: {missing}")

    extra = set(target_names) - set(expected_names)
    if extra:
        raise ValueError(f"Target for '{group.id}' has extra joints: {sorted(extra)}")
    return JointState({"name": list(group.joint_names), "position": global_positions})
