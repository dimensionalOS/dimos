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

"""Trajectory parametrizer using segmented trapezoids."""

import math

from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    BaseTrajectoryParametrizer,
    TrajectoryParametrizationError,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


class SimpleTrapezoidParametrizer(BaseTrajectoryParametrizer):
    """Wrap the existing trajectory generator behind the adapter protocol."""

    def __init__(self, config: SimpleTrapezoidParametrizationConfig) -> None:
        self._config = config

    def _parametrize_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        path: tuple[JointState, ...],
        speed_scale: float,
    ) -> JointTrajectory:
        request_velocity_limits, request_acceleration_limits = self._selected_limits(
            world,
            selection,
        )
        velocity_limits = tuple(
            value * self._config.velocity_scale * speed_scale for value in request_velocity_limits
        )
        acceleration_limits = tuple(
            value * self._config.acceleration_scale * speed_scale
            for value in request_acceleration_limits
        )
        try:
            generator = JointTrajectoryGenerator(
                num_joints=len(selection.joint_names),
                max_velocity=list(velocity_limits),
                max_acceleration=list(acceleration_limits),
                points_per_segment=self._config.points_per_segment,
            )
            generated = generator.generate([list(state.position) for state in path])
        except (IndexError, RuntimeError, TypeError, ValueError) as exc:
            raise TrajectoryParametrizationError(
                f"Simple trapezoid parametrization failed: {exc}"
            ) from exc
        return JointTrajectory(
            joint_names=list(selection.joint_names),
            points=generated.points,
            timestamp=generated.timestamp,
        )

    @staticmethod
    def _selected_limits(
        world: WorldSpec,
        selection: PlanningGroupSelection,
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        joint_space = world.get_prepared_model().joint_space
        velocities: list[float] = []
        accelerations: list[float] = []
        for joint_name in selection.joint_names:
            if joint_name not in joint_space.names:
                raise TrajectoryParametrizationError(f"Unknown model joint '{joint_name}'")
            coordinate = joint_space.coordinate(joint_name)
            velocity = coordinate.max_velocity
            acceleration = coordinate.max_acceleration
            if not math.isfinite(velocity) or velocity <= 0.0:
                raise TrajectoryParametrizationError(f"Invalid velocity limit for '{joint_name}'")
            if not math.isfinite(acceleration) or acceleration <= 0.0:
                raise TrajectoryParametrizationError(
                    f"Invalid acceleration limit for '{joint_name}'"
                )
            velocities.append(velocity)
            accelerations.append(acceleration)
        return tuple(velocities), tuple(accelerations)
