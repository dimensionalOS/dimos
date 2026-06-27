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

"""Trajectory parametrization backends for manipulation planning."""

from __future__ import annotations

from dataclasses import dataclass

from dimos.manipulation.planning.spec.config import TrajectoryParametrizationConfig
from dimos.manipulation.planning.spec.enums import ParametrizationStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, GeneratedTrajectory
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


def _failure(
    plan: GeneratedPlan,
    status: ParametrizationStatus,
    message: str,
    *,
    speed_scale: float = 1.0,
) -> GeneratedTrajectory:
    return GeneratedTrajectory(
        status=status,
        message=message,
        speed_scale=speed_scale,
        source_group_ids=plan.group_ids,
        source_plan_status=plan.status,
        source_plan_message=plan.message,
    )


def _plan_joint_names(plan: GeneratedPlan) -> list[str]:
    if not plan.path:
        return []
    return list(plan.path[0].name)


def _plan_waypoints(plan: GeneratedPlan) -> list[list[float]]:
    joint_names = _plan_joint_names(plan)
    waypoints: list[list[float]] = []
    for waypoint in plan.path:
        if list(waypoint.name) != joint_names:
            raise ValueError("All generated-plan waypoints must share identical joint ordering")
        if len(waypoint.position) != len(joint_names):
            raise ValueError("Generated-plan waypoint name and position lengths must match")
        waypoints.append([float(position) for position in waypoint.position])
    return waypoints


def _limits(
    configured: list[float] | None,
    scale: float,
    num_joints: int,
    default_limit: float,
) -> list[float]:
    raw_limits = configured if configured is not None else [default_limit] * num_joints
    if len(raw_limits) != num_joints:
        raise ValueError(f"Expected {num_joints} limits, got {len(raw_limits)}")
    limits = [float(limit) * scale for limit in raw_limits]
    if any(limit <= 0.0 for limit in limits):
        raise ValueError("Trajectory parametrization limits must be positive")
    return limits


def _trajectory_from_joint_trajectory(
    plan: GeneratedPlan,
    joint_names: list[str],
    trajectory: JointTrajectory,
    message: str,
    *,
    speed_scale: float,
) -> GeneratedTrajectory:
    return GeneratedTrajectory(
        joint_names=joint_names,
        points=list(trajectory.points),
        duration=trajectory.duration,
        speed_scale=speed_scale,
        status=ParametrizationStatus.SUCCESS,
        message=message,
        source_group_ids=plan.group_ids,
        source_plan_status=plan.status,
        source_plan_message=plan.message,
    )


@dataclass(frozen=True)
class SimpleTrapezoidParametrizer:
    """Compatibility parametrizer wrapping the existing trapezoidal generator."""

    config: TrajectoryParametrizationConfig

    def get_name(self) -> str:
        """Get trajectory parametrizer backend name."""
        return "simple_trapezoid"

    def parametrize(
        self,
        plan: GeneratedPlan,
        *,
        speed_scale: float = 1.0,
    ) -> GeneratedTrajectory:
        """Parametrize a geometric plan with the simple trapezoidal backend."""
        if not plan.is_success():
            return _failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                "GeneratedPlan is not successful",
                speed_scale=speed_scale,
            )
        if speed_scale <= 0.0:
            return _failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                "speed_scale must be positive",
                speed_scale=speed_scale,
            )
        if len(plan.path) < 2:
            return _failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                "GeneratedPlan must contain at least two waypoints",
                speed_scale=speed_scale,
            )
        try:
            joint_names = _plan_joint_names(plan)
            waypoints = _plan_waypoints(plan)
            generator = JointTrajectoryGenerator(
                num_joints=len(joint_names),
                max_velocity=_limits(
                    self.config.velocity_limits,
                    self.config.velocity_scale * speed_scale,
                    len(joint_names),
                    1.0,
                ),
                max_acceleration=_limits(
                    self.config.acceleration_limits,
                    self.config.acceleration_scale * speed_scale,
                    len(joint_names),
                    2.0,
                ),
                points_per_segment=self.config.simple_points_per_segment,
                minimum_segment_duration=self.config.minimum_segment_duration or 0.01,
            )
            trajectory = generator.generate(waypoints)
            trajectory.joint_names = list(joint_names)
            trajectory.num_joints = len(joint_names)
        except ValueError as exc:
            return _failure(
                plan,
                ParametrizationStatus.INVALID_PLAN,
                str(exc),
                speed_scale=speed_scale,
            )
        return _trajectory_from_joint_trajectory(
            plan,
            joint_names,
            trajectory,
            "Trajectory parametrized with simple_trapezoid",
            speed_scale=speed_scale,
        )


def create_trajectory_parametrizer(
    config: TrajectoryParametrizationConfig | None = None,
) -> SimpleTrapezoidParametrizer:
    """Create a trajectory parametrizer backend from config."""
    resolved_config = config or TrajectoryParametrizationConfig()
    if resolved_config.backend == "simple_trapezoid":
        return SimpleTrapezoidParametrizer(resolved_config)
    if resolved_config.backend == "roboplan":
        raise ValueError(
            'trajectory_parametrization.backend="roboplan" is provided by RoboPlanWorld; '
            'select world_backend="roboplan" or use backend="simple_trapezoid"'
        )
    raise ValueError(f"Unknown trajectory parametrization backend: {resolved_config.backend}")
