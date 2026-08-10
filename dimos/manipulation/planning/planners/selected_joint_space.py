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

# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0

"""Projection between selected planning groups and one full-model joint vector."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.models import JointPath
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass(frozen=True)
class SelectedJointSpace:
    """One full-model baseline plus projections for selected canonical joints."""

    model_joint_names: tuple[str, ...]
    selected_joint_names: tuple[str, ...]
    base_positions: NDArray[np.float64]
    lower_limits: NDArray[np.float64]
    upper_limits: NDArray[np.float64]

    @classmethod
    def from_world(cls, world: WorldSpec, selection: PlanningGroupSelection) -> SelectedJointSpace:
        config = world.get_model_config()
        with world.scratch_context() as ctx:
            current = world.get_joint_state(ctx)
        base = _ordered_positions(current, config.joint_names, "Current state")
        lower, upper = world.get_joint_limits()
        indices = [config.joint_names.index(name) for name in selection.joint_names]
        return cls(
            model_joint_names=tuple(config.joint_names),
            selected_joint_names=selection.joint_names,
            base_positions=base,
            lower_limits=np.asarray(lower, dtype=np.float64)[indices],
            upper_limits=np.asarray(upper, dtype=np.float64)[indices],
        )

    def joint_limits(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.lower_limits.copy(), self.upper_limits.copy()

    def project_config(self, selected_positions: NDArray[np.float64]) -> JointState:
        if len(selected_positions) != len(self.selected_joint_names):
            raise ValueError("Selected position count does not match selected joints")
        positions = self.base_positions.copy()
        indices = {name: i for i, name in enumerate(self.model_joint_names)}
        for name, value in zip(self.selected_joint_names, selected_positions, strict=True):
            positions[indices[name]] = value
        return JointState(name=list(self.model_joint_names), position=positions.tolist())

    def config_collision_free(
        self, world: WorldSpec, selected_positions: NDArray[np.float64]
    ) -> bool:
        return world.check_config_collision_free(self.project_config(selected_positions))

    def edge_collision_free(
        self,
        world: WorldSpec,
        start: NDArray[np.float64],
        end: NDArray[np.float64],
        step_size: float,
    ) -> bool:
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(np.ceil(distance / step_size)))
        return all(
            self.config_collision_free(world, start + (step / steps) * (end - start))
            for step in range(steps + 1)
        )

    def simplify_path(
        self,
        world: WorldSpec,
        path: JointPath,
        collision_step_size: float,
        max_iterations: int = 100,
    ) -> JointPath:
        simplified = list(path)
        for _ in range(max_iterations):
            if len(simplified) <= 2:
                break
            i = np.random.randint(0, len(simplified) - 2)
            j = np.random.randint(i + 2, len(simplified))
            if self.edge_collision_free(
                world,
                np.asarray(simplified[i].position),
                np.asarray(simplified[j].position),
                collision_step_size,
            ):
                simplified = simplified[: i + 1] + simplified[j:]
        return simplified


def normalize_selection_target(
    selection: PlanningGroupSelection, target: JointState, label: str
) -> JointState:
    """Normalize a target to canonical selection order."""
    names = list(selection.joint_names)
    if not target.name:
        if len(target.position) != len(names):
            raise ValueError(
                f"{label} target has {len(target.position)} positions, expected {len(names)}"
            )
        return JointState(name=names, position=list(target.position))
    positions = dict(zip(target.name, target.position, strict=True))
    missing = [name for name in names if name not in positions]
    extra = sorted(set(positions) - set(names))
    if missing:
        raise ValueError(f"{label} target is missing joints: {missing}")
    if extra:
        raise ValueError(f"{label} target has extra joints: {extra}")
    return JointState(name=names, position=[float(positions[name]) for name in names])


def _ordered_positions(state: JointState, names: list[str], label: str) -> NDArray[np.float64]:
    if not state.name:
        if len(state.position) != len(names):
            raise ValueError(f"{label} position count does not match model joints")
        return np.asarray(state.position, dtype=np.float64)
    positions = dict(zip(state.name, state.position, strict=True))
    missing = [name for name in names if name not in positions]
    if missing:
        raise ValueError(f"{label} is missing joints: {missing}")
    return np.asarray([positions[name] for name in names], dtype=np.float64)
