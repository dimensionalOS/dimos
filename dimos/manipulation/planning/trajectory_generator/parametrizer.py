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
import copy
import math

from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningResult
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.time import duration_from_seconds, header_now, to_nanoseconds

_TRAJECTORY_POSITION_TOLERANCE = 1e-6


class TrajectoryParametrizationError(ValueError):
    """Planning output could not be converted into a valid generated plan."""


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

        path = [copy.deepcopy(state) for state in result.path]
        self._validate_selected_path(path, selection.joint_names)
        if result.timestamps is None:
            trajectory = self._parametrize_path(world, selection, tuple(path), speed_scale)
        else:
            trajectory = self._timed_trajectory(selection, path, result.timestamps)
        self._validate_trajectory(
            trajectory,
            selection.joint_names,
            expected_start=list(path[0].position),
            expected_goal=list(path[-1].position),
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
    ) -> JointTrajectory:
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
    ) -> None:
        if len(path) < 2:
            raise TrajectoryParametrizationError("Planner returned fewer than two waypoints")
        expected = list(expected_names)
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

    @classmethod
    def _timed_trajectory(
        cls,
        selection: PlanningGroupSelection,
        path: Sequence[JointState],
        timestamps: Sequence[float],
    ) -> JointTrajectory:
        if len(timestamps) != len(path):
            raise TrajectoryParametrizationError("Planner must return one timestamp per waypoint")
        points: list[JointTrajectoryPoint] = []
        for waypoint_index, (state, timestamp) in enumerate(zip(path, timestamps, strict=True)):
            velocities = list(state.velocity)
            if len(velocities) != len(selection.joint_names):
                raise TrajectoryParametrizationError(
                    f"Waypoint {waypoint_index} velocity dimension mismatch"
                )
            points.append(
                JointTrajectoryPoint(
                    time_from_start=duration_from_seconds(float(timestamp)),
                    positions=list(state.position),
                    velocities=velocities,
                )
            )
        return JointTrajectory(
            header=header_now(),
            joint_names=list(selection.joint_names),
            points=points,
        )

    @classmethod
    def _validate_trajectory(
        cls,
        trajectory: JointTrajectory,
        expected_names: Sequence[str],
        *,
        expected_start: Sequence[float],
        expected_goal: Sequence[float],
    ) -> None:
        expected = list(expected_names)
        if list(trajectory.joint_names) != expected:
            raise TrajectoryParametrizationError(
                "Generated trajectory joint names do not match selected order"
            )
        if not trajectory.points:
            raise TrajectoryParametrizationError("Generated trajectory has no points")
        previous_time: int | None = None
        for point_index, point in enumerate(trajectory.points):
            if len(point.positions) != len(expected) or len(point.velocities) != len(expected):
                raise TrajectoryParametrizationError(
                    f"Generated point {point_index} dimension mismatch"
                )
            cls._assert_finite_sequence(
                list(point.positions),
                f"Generated point {point_index} positions",
            )
            cls._assert_finite_sequence(
                list(point.velocities),
                f"Generated point {point_index} velocities",
            )
            try:
                point_time = to_nanoseconds(point.time_from_start)
            except ValueError as exc:
                raise TrajectoryParametrizationError(
                    f"Generated point {point_index} duration is invalid"
                ) from exc
            if point_index == 0 and point_time != 0.0:
                raise TrajectoryParametrizationError("Generated trajectory must start at time 0")
            if previous_time is not None and point_time <= previous_time:
                raise TrajectoryParametrizationError(
                    "Generated trajectory times must be strictly increasing"
                )
            previous_time = point_time
        if not cls._positions_close(list(trajectory.points[0].positions), expected_start):
            raise TrajectoryParametrizationError(
                "Generated trajectory does not preserve the path start"
            )
        if not cls._positions_close(list(trajectory.points[-1].positions), expected_goal):
            raise TrajectoryParametrizationError(
                "Generated trajectory does not preserve the path goal"
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
