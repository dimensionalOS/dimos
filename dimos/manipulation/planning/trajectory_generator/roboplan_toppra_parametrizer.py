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

"""RoboPlan TOPP-RA trajectory parametrization adapter."""

from dataclasses import dataclass
import math
import sys
from typing import Any

import numpy as np
import roboplan.core as roboplan_core
import roboplan.toppra as roboplan_toppra

from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.trajectory_generator.config import (
    RoboPlanTOPPRAParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    BaseTrajectoryParametrizer,
    TrajectoryParametrizationError,
)
from dimos.manipulation.planning.world.roboplan_model import RoboPlanGroup, RoboPlanModel
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@dataclass(frozen=True)
class _GroupParametrizer:
    group: RoboPlanGroup
    native: Any


class RoboPlanTOPPRAParametrizer(BaseTrajectoryParametrizer):
    """Convert selected-joint paths with a finalized RoboPlan scene."""

    def __init__(
        self,
        config: RoboPlanTOPPRAParametrizationConfig,
    ) -> None:
        self._config = config
        self._groups: dict[frozenset[str], _GroupParametrizer] = {}

    def _parametrize_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        path: tuple[JointState, ...],
        speed_scale: float,
    ) -> JointTrajectory:
        if not isinstance(world, RoboPlanWorld):
            raise TrajectoryParametrizationError("RoboPlan TOPP-RA requires RoboPlanWorld")
        try:
            with world.parametrization_model() as model:
                resolved = self._resolve_group(model, selection)
                native_path = self._native_path(resolved.group, selection, path)
                native_trajectory = resolved.native.generate(
                    native_path, self._options(speed_scale)
                )
                return self._canonical_result(
                    resolved,
                    selection,
                    native_trajectory,
                )
        except TrajectoryParametrizationError:
            raise
        except (IndexError, KeyError, RuntimeError, TypeError, ValueError) as exc:
            raise TrajectoryParametrizationError(
                f"RoboPlan TOPP-RA parametrization failed: {exc}"
            ) from exc

    def _resolve_group(
        self,
        model: RoboPlanModel,
        selection: PlanningGroupSelection,
    ) -> _GroupParametrizer:
        key = frozenset(selection.group_ids)
        cached = self._groups.get(key)
        if cached is not None:
            return cached
        group = model.groups.get(key)
        if group is None:
            raise TrajectoryParametrizationError(
                f"RoboPlan has no generated group for {list(selection.group_ids)}"
            )
        expected = set(selection.joint_names)
        if expected != set(group.public_names):
            raise TrajectoryParametrizationError(
                f"RoboPlan group '{group.name}' does not match selected joints"
            )
        self._validate_limits(
            model.scene.getVelocityLimitVectors(group.name),
            group,
            "velocity",
        )
        self._validate_limits(
            model.scene.getAccelerationLimitVectors(group.name),
            group,
            "acceleration",
        )
        resolved = _GroupParametrizer(
            group=group,
            native=roboplan_toppra.PathParameterizerTOPPRA(model.scene, group.name),
        )
        self._groups[key] = resolved
        return resolved

    @staticmethod
    def _validate_limits(
        bounds: tuple[Any, Any],
        group: RoboPlanGroup,
        label: str,
    ) -> None:
        lower = np.asarray(bounds[0], dtype=np.float64)
        upper = np.asarray(bounds[1], dtype=np.float64)
        if lower.shape != upper.shape or len(lower) != len(group.native_names):
            raise TrajectoryParametrizationError(
                f"RoboPlan {label} limits do not match group '{group.name}'"
            )
        for public_name, low, high in zip(group.public_names, lower, upper, strict=True):
            magnitude = min(abs(float(low)), abs(float(high)))
            if not math.isfinite(magnitude) or magnitude <= 0.0 or magnitude >= sys.float_info.max:
                raise TrajectoryParametrizationError(
                    f"RoboPlan group '{group.name}' has no usable URDF {label} "
                    f"limit for joint '{public_name}'"
                )

    @staticmethod
    def _native_path(
        group: RoboPlanGroup,
        selection: PlanningGroupSelection,
        path_states: tuple[JointState, ...],
    ) -> Any:
        public_index = {name: index for index, name in enumerate(selection.joint_names)}
        path = roboplan_core.JointPath()
        path.joint_names = list(group.native_names)
        path.positions = [
            np.asarray(
                [state.position[public_index[public_name]] for public_name in group.public_names],
                dtype=np.float64,
            )
            for state in path_states
        ]
        return path

    def _options(self, speed_scale: float) -> Any:
        return roboplan_toppra.TOPPRAOptions(
            dt=self._config.output_period,
            mode=roboplan_toppra.SplineFittingMode.LinearBlend,
            velocity_scale=self._config.velocity_scale * speed_scale,
            acceleration_scale=self._config.acceleration_scale * speed_scale,
            max_blend_deviation=0.0,
        )

    @staticmethod
    def _canonical_result(
        resolved: _GroupParametrizer,
        selection: PlanningGroupSelection,
        native_trajectory: Any,
    ) -> JointTrajectory:
        native_names = tuple(native_trajectory.joint_names)
        if set(native_names) != set(resolved.group.native_names):
            raise TrajectoryParametrizationError("RoboPlan TOPP-RA returned unexpected joint names")
        native_index = {name: index for index, name in enumerate(native_names)}
        native_by_public = dict(
            zip(
                resolved.group.public_names,
                resolved.group.native_names,
                strict=True,
            )
        )
        output_indices = [native_index[native_by_public[name]] for name in selection.joint_names]
        times = [float(value) for value in native_trajectory.times]
        positions = list(native_trajectory.positions)
        velocities = list(native_trajectory.velocities)
        if not (len(times) == len(positions) == len(velocities)):
            raise TrajectoryParametrizationError(
                "RoboPlan TOPP-RA returned inconsistent trajectory fields"
            )
        points = [
            TrajectoryPoint(
                time_from_start=time,
                positions=[float(position[index]) for index in output_indices],
                velocities=[float(velocity[index]) for index in output_indices],
            )
            for time, position, velocity in zip(times, positions, velocities, strict=True)
        ]
        return JointTrajectory(
            joint_names=list(selection.joint_names),
            points=points,
        )
