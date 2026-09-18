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

"""Shared helpers for planning-group-scoped kinematics backends."""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.utils import filter_joint_state_to_selected_joints
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass(frozen=True)
class SinglePoseTargetRequest:
    group: PlanningGroup
    target_pose: PoseStamped
    joint_names: list[str]
    seed_positions: NDArray[np.float64]
    group_indices: list[int]


def unique_pose_target_frame(world: WorldSpec) -> str | None:
    frames = [
        group.tip_link
        for group in world.get_prepared_model().config.planning_groups
        if group.tip_link is not None
    ]
    unique = list(dict.fromkeys(frames))
    return unique[0] if len(unique) == 1 else None


def seed_positions_with_world_fallback(
    world: WorldSpec, joint_names: list[str], seed: JointState | None
) -> NDArray[np.float64]:
    if seed is not None and not seed.name:
        return positions_by_name(seed, joint_names)
    if seed is not None and set(seed.name) == set(joint_names):
        return positions_by_name(seed, joint_names)

    with world.scratch_context() as ctx:
        current = world.get_joint_state(ctx)
    fallback = positions_by_name(current, joint_names)
    if seed is None:
        return fallback
    known = set(joint_names)
    for name, position in zip(seed.name, seed.position, strict=True):
        if name not in known:
            raise ValueError(f"Unrecognized seed joint '{name}'")
        fallback[joint_names.index(name)] = position
    return fallback


def resolve_single_pose_target_request(
    world: WorldSpec,
    pose_targets: Mapping[PlanningGroup, PoseStamped],
    auxiliary_groups: Sequence[PlanningGroup],
    seed: JointState | None,
    backend_name: str,
) -> tuple[SinglePoseTargetRequest | None, IKResult | None]:
    if not pose_targets:
        return None, _failure(IKStatus.NO_SOLUTION, "At least one pose target is required")
    if len(pose_targets) != 1 or auxiliary_groups:
        return None, _failure(
            IKStatus.UNSUPPORTED,
            f"{backend_name} supports exactly one pose target and no auxiliary planning groups",
        )
    group = next(iter(pose_targets))
    if not group.has_pose_target:
        return None, _failure(IKStatus.UNSUPPORTED, f"Planning group '{group.id}' has no tip")
    try:
        joint_names = list(world.get_prepared_model().joint_space.names)
        seed_positions = seed_positions_with_world_fallback(world, joint_names, seed)
        indices = [joint_names.index(name) for name in group.joint_names]
    except ValueError as exc:
        return None, _failure(IKStatus.NO_SOLUTION, str(exc))
    return SinglePoseTargetRequest(
        group=group,
        target_pose=pose_targets[group],
        joint_names=joint_names,
        seed_positions=seed_positions,
        group_indices=indices,
    ), None


def positions_by_name(state: JointState, joint_names: list[str]) -> NDArray[np.float64]:
    if not state.name:
        if len(state.position) != len(joint_names):
            raise ValueError("JointState position count does not match model joints")
        return np.asarray(state.position, dtype=np.float64)
    positions = dict(zip(state.name, state.position, strict=True))
    missing = [name for name in joint_names if name not in positions]
    if missing:
        raise ValueError(f"JointState missing joints: {missing}")
    return np.asarray([positions[name] for name in joint_names], dtype=np.float64)


def filter_result_to_group(result: IKResult, group: PlanningGroup) -> IKResult:
    return filter_result_to_selection(result, PlanningGroupSelection.from_groups((group,)))


def filter_result_to_selection(result: IKResult, selection: PlanningGroupSelection) -> IKResult:
    if result.joint_state is None:
        return result
    return IKResult(
        status=result.status,
        joint_state=filter_joint_state_to_selected_joints(
            result.joint_state, selection.joint_names
        ),
        position_error=result.position_error,
        orientation_error=result.orientation_error,
        iterations=result.iterations,
        message=result.message,
    )


def _failure(status: IKStatus, message: str) -> IKResult:
    return IKResult(status=status, joint_state=None, message=message)
