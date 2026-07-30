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

"""Shared implementation for the trajectory-parametrizer planning Spec."""

from abc import ABC, abstractmethod
from collections.abc import Sequence
from dataclasses import dataclass
import math

from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningResult
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

_TRAJECTORY_POSITION_TOLERANCE = 1e-6
_TRAJECTORY_LIMIT_RELATIVE_TOLERANCE = 1e-2
_TRAJECTORY_LIMIT_ABSOLUTE_TOLERANCE = 1e-8


class TrajectoryParametrizationError(ValueError):
    """Planning output could not be converted into a valid generated plan."""


@dataclass(frozen=True)
class ParametrizedTrajectory:
    """Backend output plus the limits and accelerations used to validate it."""

    trajectory: JointTrajectory
    velocity_limits: tuple[float, ...]
    acceleration_limits: tuple[float, ...]
    accelerations: tuple[tuple[float, ...], ...] | None = None


class BaseTrajectoryParametrizer(ABC):
    """Own common PlanningResult-to-GeneratedPlan materialization."""

    def materialize_plan(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        result: PlanningResult,
        speed_scale: float = 1.0,
    ) -> GeneratedPlan:
        """Preserve timed output or parametrize an untimed path, then validate it."""
        self._validate_speed_scale(speed_scale)
        if not result.is_success():
            raise TrajectoryParametrizationError(
                f"Cannot materialize unsuccessful planning result: {result.status.name}"
            )

        path = [JointState(state) for state in result.path]
        waypoints = self._validate_selected_path(path, selection.joint_names)
        if result.timestamps is None:
            parametrized = self._parametrize_path(world, selection, tuple(path), speed_scale)
            trajectory = parametrized.trajectory
            self._validate_generated_trajectory(
                trajectory,
                selection.joint_names,
                waypoints,
                velocity_limits=parametrized.velocity_limits,
                acceleration_limits=parametrized.acceleration_limits,
                accelerations=parametrized.accelerations,
            )
        else:
            trajectory = self._timed_trajectory(selection, path, result.timestamps)
            self._validate_generated_trajectory(
                trajectory,
                selection.joint_names,
                waypoints,
            )

        return GeneratedPlan(
            group_ids=selection.group_ids,
            trajectory=trajectory,
            path=path,
            status=result.status,
            planning_time=result.planning_time,
            path_length=result.path_length,
            iterations=result.iterations,
            message=result.message,
        )

    @abstractmethod
    def _parametrize_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        path: tuple[JointState, ...],
        speed_scale: float,
    ) -> ParametrizedTrajectory:
        """Convert one validated untimed path using the selected backend."""

    @staticmethod
    def _validate_speed_scale(speed_scale: float) -> None:
        if not math.isfinite(speed_scale) or speed_scale <= 0.0 or speed_scale > 1.0:
            raise TrajectoryParametrizationError("speed_scale must be finite, > 0, and <= 1")

    @staticmethod
    def _assert_finite_sequence(values: Sequence[float], label: str) -> None:
        for value in values:
            if not math.isfinite(value):
                raise TrajectoryParametrizationError(f"{label} contains non-finite value")

    @classmethod
    def _validate_selected_path(
        cls,
        path: Sequence[JointState],
        expected_names: Sequence[str],
    ) -> list[list[float]]:
        if len(path) < 2:
            raise TrajectoryParametrizationError("Planner returned fewer than two waypoints")
        expected = list(expected_names)
        waypoints: list[list[float]] = []
        for waypoint_index, state in enumerate(path):
            if list(state.name) != expected:
                raise TrajectoryParametrizationError(
                    f"Waypoint {waypoint_index} joint names do not match selected order"
                )
            positions = list(state.position)
            if len(positions) != len(expected):
                raise TrajectoryParametrizationError(
                    f"Waypoint {waypoint_index} position dimension mismatch"
                )
            cls._assert_finite_sequence(
                positions,
                f"Waypoint {waypoint_index} positions",
            )
            waypoints.append(positions)
        return waypoints

    @classmethod
    def _timed_trajectory(
        cls,
        selection: PlanningGroupSelection,
        path: Sequence[JointState],
        timestamps: Sequence[float],
    ) -> JointTrajectory:
        if len(timestamps) != len(path):
            raise TrajectoryParametrizationError("Planner must return one timestamp per waypoint")
        points: list[TrajectoryPoint] = []
        for waypoint_index, (state, timestamp) in enumerate(zip(path, timestamps, strict=True)):
            velocities = list(state.velocity)
            if len(velocities) != len(selection.joint_names):
                raise TrajectoryParametrizationError(
                    f"Waypoint {waypoint_index} velocity dimension mismatch"
                )
            points.append(
                TrajectoryPoint(
                    time_from_start=float(timestamp),
                    positions=list(state.position),
                    velocities=velocities,
                )
            )
        return JointTrajectory(
            joint_names=list(selection.joint_names),
            points=points,
        )

    @classmethod
    def _validate_generated_trajectory(
        cls,
        trajectory: JointTrajectory,
        expected_names: Sequence[str],
        waypoints: Sequence[Sequence[float]],
        *,
        velocity_limits: Sequence[float] | None = None,
        acceleration_limits: Sequence[float] | None = None,
        accelerations: Sequence[Sequence[float]] | None = None,
    ) -> None:
        expected = list(expected_names)
        if list(trajectory.joint_names) != expected:
            raise TrajectoryParametrizationError(
                "Generated trajectory joint names do not match selected order"
            )
        if not trajectory.points:
            raise TrajectoryParametrizationError("Generated trajectory has no points")
        previous_time: float | None = None
        for point_index, point in enumerate(trajectory.points):
            if len(point.positions) != len(expected) or len(point.velocities) != len(expected):
                raise TrajectoryParametrizationError(
                    f"Generated point {point_index} dimension mismatch"
                )
            cls._assert_finite_sequence(
                point.positions,
                f"Generated point {point_index} positions",
            )
            cls._assert_finite_sequence(
                point.velocities,
                f"Generated point {point_index} velocities",
            )
            if not math.isfinite(point.time_from_start):
                raise TrajectoryParametrizationError(
                    f"Generated point {point_index} time is non-finite"
                )
            if point_index == 0 and point.time_from_start != 0.0:
                raise TrajectoryParametrizationError("Generated trajectory must start at time 0")
            if previous_time is not None and point.time_from_start <= previous_time:
                raise TrajectoryParametrizationError(
                    "Generated trajectory times must be strictly increasing"
                )
            previous_time = point.time_from_start
        non_noop = any(list(waypoint) != list(waypoints[0]) for waypoint in waypoints[1:])
        if non_noop and trajectory.duration <= 0.0:
            raise TrajectoryParametrizationError("Generated trajectory duration must be positive")
        if not cls._positions_close(trajectory.points[0].positions, waypoints[0]):
            raise TrajectoryParametrizationError(
                "Generated trajectory does not preserve the path start"
            )
        if not cls._positions_close(trajectory.points[-1].positions, waypoints[-1]):
            raise TrajectoryParametrizationError(
                "Generated trajectory does not preserve the path goal"
            )
        if velocity_limits is not None:
            cls._validate_motion_limits(
                trajectory,
                velocity_limits,
                acceleration_limits,
                accelerations,
            )

    @staticmethod
    def _positions_close(first: Sequence[float], second: Sequence[float]) -> bool:
        return len(first) == len(second) and all(
            math.isclose(
                left,
                right,
                rel_tol=0.0,
                abs_tol=_TRAJECTORY_POSITION_TOLERANCE,
            )
            for left, right in zip(first, second, strict=True)
        )

    @classmethod
    def _validate_motion_limits(
        cls,
        trajectory: JointTrajectory,
        velocity_limits: Sequence[float],
        acceleration_limits: Sequence[float] | None,
        accelerations: Sequence[Sequence[float]] | None,
    ) -> None:
        expected_dimension = len(trajectory.joint_names)
        if len(velocity_limits) != expected_dimension:
            raise TrajectoryParametrizationError("Velocity limits do not match selected joints")
        cls._assert_valid_motion_limits(velocity_limits, "velocity")
        for point_index, point in enumerate(trajectory.points):
            cls._assert_within_limits(
                point.velocities,
                velocity_limits,
                f"Generated point {point_index} velocity",
            )
        if acceleration_limits is None:
            return
        if len(acceleration_limits) != expected_dimension:
            raise TrajectoryParametrizationError("Acceleration limits do not match selected joints")
        cls._assert_valid_motion_limits(acceleration_limits, "acceleration")
        if accelerations is not None:
            if len(accelerations) != len(trajectory.points):
                raise TrajectoryParametrizationError(
                    "Acceleration samples do not match trajectory points"
                )
            for point_index, values in enumerate(accelerations):
                if len(values) != expected_dimension:
                    raise TrajectoryParametrizationError(
                        f"Generated point {point_index} acceleration dimension mismatch"
                    )
                cls._assert_finite_sequence(
                    values,
                    f"Generated point {point_index} accelerations",
                )
                cls._assert_within_limits(
                    values,
                    acceleration_limits,
                    f"Generated point {point_index} acceleration",
                )
            return
        for point_index in range(1, len(trajectory.points)):
            previous = trajectory.points[point_index - 1]
            current = trajectory.points[point_index]
            dt = current.time_from_start - previous.time_from_start
            derived = [
                (current_velocity - previous_velocity) / dt
                for previous_velocity, current_velocity in zip(
                    previous.velocities,
                    current.velocities,
                    strict=True,
                )
            ]
            cls._assert_within_limits(
                derived,
                acceleration_limits,
                f"Generated interval {point_index - 1}:{point_index} acceleration",
            )

    @staticmethod
    def _assert_valid_motion_limits(values: Sequence[float], label: str) -> None:
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise TrajectoryParametrizationError(f"Invalid {label} limits")

    @staticmethod
    def _assert_within_limits(
        values: Sequence[float],
        limits: Sequence[float],
        label: str,
    ) -> None:
        for joint_index, (value, limit) in enumerate(zip(values, limits, strict=True)):
            tolerance = max(
                _TRAJECTORY_LIMIT_ABSOLUTE_TOLERANCE,
                limit * _TRAJECTORY_LIMIT_RELATIVE_TOLERANCE,
            )
            if abs(value) > limit + tolerance:
                raise TrajectoryParametrizationError(
                    f"{label} exceeds joint {joint_index} limit: {value} vs {limit}"
                )
