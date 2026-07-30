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

"""Compatibility trajectory parametrizer using segmented trapezoids."""

from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    ParametrizedTrajectory,
    TrajectoryParametrizationError,
    TrajectoryParametrizationRequest,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


class SimpleTrapezoidParametrizer:
    """Wrap the existing trajectory generator behind the adapter protocol."""

    def __init__(self, config: SimpleTrapezoidParametrizationConfig) -> None:
        self._config = config

    @property
    def uses_request_limits(self) -> bool:
        """The compatibility backend uses limits resolved from DimOS config."""
        return True

    def parametrize(self, request: TrajectoryParametrizationRequest) -> ParametrizedTrajectory:
        if request.velocity_limits is None or request.acceleration_limits is None:
            raise TrajectoryParametrizationError(
                "Simple trapezoid parametrization requires DimOS motion limits"
            )
        velocity_limits = tuple(
            value * self._config.velocity_scale * request.speed_scale
            for value in request.velocity_limits
        )
        acceleration_limits = tuple(
            value * self._config.acceleration_scale * request.speed_scale
            for value in request.acceleration_limits
        )
        try:
            generator = JointTrajectoryGenerator(
                num_joints=len(request.joint_names),
                max_velocity=list(velocity_limits),
                max_acceleration=list(acceleration_limits),
                points_per_segment=self._config.points_per_segment,
            )
            generated = generator.generate([list(state.position) for state in request.path])
        except (IndexError, RuntimeError, TypeError, ValueError) as exc:
            raise TrajectoryParametrizationError(
                f"Simple trapezoid parametrization failed: {exc}"
            ) from exc
        trajectory = JointTrajectory(
            joint_names=list(request.joint_names),
            points=generated.points,
            timestamp=generated.timestamp,
        )
        return ParametrizedTrajectory(
            trajectory=trajectory,
            velocity_limits=velocity_limits,
            acceleration_limits=acceleration_limits,
        )
