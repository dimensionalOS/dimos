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

from dimos.manipulation.planning.trajectory_generator.config import (
    RoboPlanTOPPRAParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    ParametrizedTrajectory,
    TrajectoryParametrizationError,
    TrajectoryParametrizationRequest,
)
from dimos.manipulation.planning.world.roboplan_model import RoboPlanGroup, RoboPlanModel
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@dataclass(frozen=True)
class _GroupParametrizer:
    group: RoboPlanGroup
    native: Any
    velocity_limits: tuple[float, ...]
    acceleration_limits: tuple[float, ...]


class RoboPlanTOPPRAParametrizer:
    """Convert selected-joint paths with a finalized RoboPlan scene."""

    def __init__(
        self,
        world: RoboPlanWorld,
        config: RoboPlanTOPPRAParametrizationConfig,
    ) -> None:
        self._world = world
        self._config = config
        self._groups: dict[frozenset[str], _GroupParametrizer] = {}

    @property
    def uses_request_limits(self) -> bool:
        """RoboPlan uses only limits from its authoritative scene."""
        return False

    def parametrize(self, request: TrajectoryParametrizationRequest) -> ParametrizedTrajectory:
        try:
            with self._world.parametrization_model() as model:
                resolved = self._resolve_group(model, request)
                native_path = self._native_path(resolved.group, request)
                native_trajectory = resolved.native.generate(
                    native_path, self._options(request.speed_scale)
                )
                return self._canonical_result(resolved, request, native_trajectory)
        except TrajectoryParametrizationError:
            raise
        except (IndexError, KeyError, RuntimeError, TypeError, ValueError) as exc:
            raise TrajectoryParametrizationError(
                f"RoboPlan TOPP-RA parametrization failed: {exc}"
            ) from exc

    def _resolve_group(
        self,
        model: RoboPlanModel,
        request: TrajectoryParametrizationRequest,
    ) -> _GroupParametrizer:
        key = frozenset(request.group_ids)
        cached = self._groups.get(key)
        if cached is not None:
            return cached
        group = model.groups.get(key)
        if group is None:
            raise TrajectoryParametrizationError(
                f"RoboPlan has no generated group for {list(request.group_ids)}"
            )
        expected = set(request.joint_names)
        if expected != set(group.public_names):
            raise TrajectoryParametrizationError(
                f"RoboPlan group '{group.name}' does not match selected joints"
            )
        velocity_limits = self._limits(
            model.scene.getVelocityLimitVectors(group.name),
            group,
            "velocity",
        )
        acceleration_limits = self._limits(
            model.scene.getAccelerationLimitVectors(group.name),
            group,
            "acceleration",
        )
        resolved = _GroupParametrizer(
            group=group,
            native=roboplan_toppra.PathParameterizerTOPPRA(model.scene, group.name),
            velocity_limits=tuple(value * self._config.velocity_scale for value in velocity_limits),
            acceleration_limits=tuple(
                value * self._config.acceleration_scale for value in acceleration_limits
            ),
        )
        self._groups[key] = resolved
        return resolved

    @staticmethod
    def _limits(
        bounds: tuple[Any, Any],
        group: RoboPlanGroup,
        label: str,
    ) -> tuple[float, ...]:
        lower = np.asarray(bounds[0], dtype=np.float64)
        upper = np.asarray(bounds[1], dtype=np.float64)
        if lower.shape != upper.shape or len(lower) != len(group.native_names):
            raise TrajectoryParametrizationError(
                f"RoboPlan {label} limits do not match group '{group.name}'"
            )
        by_public: dict[str, float] = {}
        for public_name, low, high in zip(group.public_names, lower, upper, strict=True):
            magnitude = min(abs(float(low)), abs(float(high)))
            if not math.isfinite(magnitude) or magnitude <= 0.0 or magnitude >= sys.float_info.max:
                raise TrajectoryParametrizationError(
                    f"RoboPlan group '{group.name}' has no usable URDF {label} "
                    f"limit for joint '{public_name}'"
                )
            by_public[public_name] = magnitude
        return tuple(by_public[name] for name in group.public_names)

    @staticmethod
    def _native_path(
        group: RoboPlanGroup,
        request: TrajectoryParametrizationRequest,
    ) -> Any:
        public_index = {name: index for index, name in enumerate(request.joint_names)}
        path = roboplan_core.JointPath()
        path.joint_names = list(group.native_names)
        path.positions = [
            np.asarray(
                [state.position[public_index[public_name]] for public_name in group.public_names],
                dtype=np.float64,
            )
            for state in request.path
        ]
        return path

    def _options(self, speed_scale: float) -> Any:
        return roboplan_toppra.TOPPRAOptions(
            dt=self._config.output_period,
            mode={
                "hermite": roboplan_toppra.SplineFittingMode.Hermite,
                "cubic": roboplan_toppra.SplineFittingMode.Cubic,
                "adaptive": roboplan_toppra.SplineFittingMode.Adaptive,
                "linear_blend": roboplan_toppra.SplineFittingMode.LinearBlend,
            }[self._config.fitting_mode],
            velocity_scale=self._config.velocity_scale * speed_scale,
            acceleration_scale=self._config.acceleration_scale * speed_scale,
            max_adaptive_iterations=self._config.max_adaptive_iterations,
            max_adaptive_step_size=self._config.max_adaptive_step_size,
            max_blend_deviation=self._config.max_blend_deviation,
        )

    @staticmethod
    def _canonical_result(
        resolved: _GroupParametrizer,
        request: TrajectoryParametrizationRequest,
        native_trajectory: Any,
    ) -> ParametrizedTrajectory:
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
        output_indices = [native_index[native_by_public[name]] for name in request.joint_names]
        times = [float(value) for value in native_trajectory.times]
        positions = list(native_trajectory.positions)
        velocities = list(native_trajectory.velocities)
        accelerations = list(native_trajectory.accelerations)
        if not (len(times) == len(positions) == len(velocities) == len(accelerations)):
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
        canonical_accelerations = tuple(
            tuple(float(acceleration[index]) for index in output_indices)
            for acceleration in accelerations
        )
        velocity_by_public = dict(
            zip(
                resolved.group.public_names,
                resolved.velocity_limits,
                strict=True,
            )
        )
        acceleration_by_public = dict(
            zip(
                resolved.group.public_names,
                resolved.acceleration_limits,
                strict=True,
            )
        )
        return ParametrizedTrajectory(
            trajectory=JointTrajectory(
                joint_names=list(request.joint_names),
                points=points,
            ),
            velocity_limits=tuple(
                velocity_by_public[name] * request.speed_scale for name in request.joint_names
            ),
            acceleration_limits=tuple(
                acceleration_by_public[name] * request.speed_scale for name in request.joint_names
            ),
            accelerations=canonical_accelerations,
        )
