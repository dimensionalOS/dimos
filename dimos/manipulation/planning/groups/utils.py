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

"""Planning-group selection and canonical joint-state projection."""

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.sensor_msgs.JointState import JointState


def planning_group_id_from_selector(selector: PlanningGroupID | PlanningGroup) -> PlanningGroupID:
    """Return the planning-group ID represented by a selector."""
    return selector.id if isinstance(selector, PlanningGroup) else selector


def filter_joint_state_to_selected_joints(
    joint_state: JointState, joint_names: Sequence[str]
) -> JointState:
    """Project a canonical joint state to selected joints in the requested order."""
    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    missing = [name for name in joint_names if name not in positions_by_name]
    if missing:
        raise ValueError(f"Joint state is missing selected joints: {missing}")
    return JointState(
        name=list(joint_names),
        position=[float(positions_by_name[name]) for name in joint_names],
    )


def normalize_joint_target(group: PlanningGroup, target: JointState) -> JointState:
    """Normalize one group target to canonical group joint order."""
    if not target.name:
        if len(target.position) != len(group.joint_names):
            raise ValueError(
                f"Target for '{group.id}' has {len(target.position)} positions, "
                f"expected {len(group.joint_names)}"
            )
        return JointState(name=list(group.joint_names), position=list(target.position))
    if len(target.name) != len(target.position):
        raise ValueError(
            f"Target for '{group.id}' has {len(target.name)} names but "
            f"{len(target.position)} positions"
        )
    positions = dict(zip(target.name, target.position, strict=True))
    missing = set(group.joint_names) - positions.keys()
    extra = positions.keys() - set(group.joint_names)
    if missing:
        raise ValueError(f"Target for '{group.id}' is missing joints: {sorted(missing)}")
    if extra:
        raise ValueError(f"Target for '{group.id}' has extra joints: {sorted(extra)}")
    return JointState(
        name=list(group.joint_names),
        position=[float(positions[name]) for name in group.joint_names],
    )


def joint_state_to_ordered_positions(
    joint_state: JointState, *, joint_names: Sequence[str]
) -> NDArray[np.float64]:
    """Normalize an unnamed or canonically named state to model joint order."""
    if not joint_state.name:
        if len(joint_state.position) != len(joint_names):
            raise ValueError(
                f"Joint state has {len(joint_state.position)} positions, "
                f"expected {len(joint_names)}"
            )
        return np.asarray(joint_state.position, dtype=np.float64)
    if len(joint_state.name) != len(joint_state.position):
        raise ValueError("Joint state names and positions must have the same length")
    positions = dict(zip(joint_state.name, joint_state.position, strict=True))
    missing = [name for name in joint_names if name not in positions]
    extra = set(positions) - set(joint_names)
    if missing:
        raise ValueError(f"Joint state is missing joints: {missing}")
    if extra:
        raise ValueError(f"Joint state has unknown joints: {sorted(extra)}")
    return np.asarray([positions[name] for name in joint_names], dtype=np.float64)
