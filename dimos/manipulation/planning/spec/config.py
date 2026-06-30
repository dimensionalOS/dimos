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

"""Robot configuration for manipulation planning."""

from __future__ import annotations

from pathlib import Path
from typing import Literal, TypeAlias

from pydantic import Field

from dimos.core.module import ModuleConfig
from dimos.manipulation.planning.groups.discovery import FALLBACK_PLANNING_GROUP_NAME
from dimos.manipulation.planning.groups.identifiers import (
    assert_local_joint_names,
    assert_valid_robot_name,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

TrajectoryParametrizationBackend: TypeAlias = Literal["simple_trapezoid", "roboplan"]
RoboPlanSplineFittingMode: TypeAlias = Literal["hermite", "cubic", "adaptive", "linear_blend"]


class TrajectoryParametrizationConfig(ModuleConfig):
    """Configuration for manipulation trajectory parametrization.

    The default `simple_trapezoid` backend preserves the historical baseline
    timing behavior. RoboPlan trajectory parametrization is explicit opt-in and
    runs through the RoboPlan world so the scene, planning group name, and native
    joint mapping stay backend-owned.
    """

    backend: TrajectoryParametrizationBackend = "simple_trapezoid"
    velocity_scale: float = Field(default=1.0, gt=0.0)
    acceleration_scale: float = Field(default=1.0, gt=0.0)
    velocity_limits: list[float] | None = None
    acceleration_limits: list[float] | None = None
    minimum_segment_duration: float | None = Field(default=None, gt=0.0)
    simple_points_per_segment: int = Field(default=50, ge=1)
    roboplan_dt: float = Field(default=0.01, gt=0.0)
    roboplan_spline_mode: RoboPlanSplineFittingMode = "hermite"
    roboplan_max_adaptive_iterations: int = Field(default=10, ge=1)
    roboplan_max_adaptive_step_size: float = Field(default=0.05, gt=0.0)
    roboplan_max_blend_deviation: float = Field(default=0.01, gt=0.0)
    roboplan_smoothing_enabled: bool = True
    roboplan_smoothing_min_waypoints: int = Field(default=8, ge=3)
    roboplan_smoothing_max_attempts: int = Field(default=4, ge=1)
    roboplan_smoothing_max_joint_deviation: float = Field(default=0.02, ge=0.0)
    roboplan_smoothing_edge_step_size: float = Field(default=0.02, gt=0.0)
    roboplan_smoothing_min_keep_fraction: float = Field(default=0.1, gt=0.0, le=1.0)


class RobotModelConfig(ModuleConfig):
    """Configuration for adding a robot to the world.

    Attributes:
        name: Human-readable robot name
        model_path: Path to robot model file (.urdf, .xacro, or .xml/MJCF)
        srdf_path: Optional path to SRDF file containing planning group definitions
        base_pose: Compatibility placement transform used by current Drake
            world loading/welding. This is the canonical world placement for
            robot instances; model-authored world/base attach joints are
            stripped when strip_model_world_joint is true.
        joint_names: Ordered list of controllable joints in the local model
            namespace. This is not a planning group.
        end_effector_link: Compatibility robot-scoped end-effector link used by
            legacy helpers. New pose-targeted planning should use planning
            group target frames instead.
        base_link: Compatibility robot-scoped base link used by current Drake
            weld/placement behavior. Planning groups own chain base links.
            TODO: should remove
        package_paths: Dict mapping package names to filesystem Paths
        joint_limits_lower: Lower joint limits (radians)
        joint_limits_upper: Upper joint limits (radians)
        velocity_limits: Joint velocity limits (rad/s)
        auto_convert_meshes: Auto-convert DAE/STL meshes to OBJ for Drake
        xacro_args: Arguments to pass to xacro processor (for .xacro files)
        collision_exclusion_pairs: List of (link1, link2) pairs to exclude from collision.
            Useful for parallel linkage mechanisms like grippers where non-adjacent
            links may legitimately overlap (e.g., mimic joints).
        max_velocity: Maximum joint velocity for trajectory generation (rad/s)
        max_acceleration: Maximum joint acceleration for trajectory generation (rad/s^2)
        coordinator_task_name: Task name for executing trajectories via coordinator RPC.
            If set, trajectories can be executed via execute_trajectory() RPC.
    """

    name: str
    model_path: Path
    srdf_path: Path | None = None
    base_pose: PoseStamped = Field(default_factory=PoseStamped)
    strip_model_world_joint: bool = False
    joint_names: list[str]
    end_effector_link: str | None = None
    base_link: str = "base_link"
    planning_groups: list[PlanningGroupDefinition] = Field(default_factory=list)
    package_paths: dict[str, Path] = Field(default_factory=dict)
    joint_limits_lower: list[float] | None = None
    joint_limits_upper: list[float] | None = None
    velocity_limits: list[float] | None = None
    auto_convert_meshes: bool = False
    xacro_args: dict[str, str] = Field(default_factory=dict)
    collision_exclusion_pairs: list[tuple[str, str]] = Field(default_factory=list)
    # Motion constraints for trajectory generation
    max_velocity: float = 1.0
    max_acceleration: float = 2.0
    # Coordinator integration
    coordinator_task_name: str | None = None
    gripper_hardware_id: str | None = None
    # TF publishing for extra links (e.g., camera mount)
    tf_extra_links: list[str] = Field(default_factory=list)
    # Home/observe joint configuration for go_home skill
    home_joints: list[float] | None = None
    # Pre-grasp offset distance in meters (along approach direction)
    pre_grasp_offset: float = 0.10

    def model_post_init(self, __context: object) -> None:
        """Validate delimiter-based naming constraints."""
        assert_valid_robot_name(self.name)
        assert_local_joint_names(self.joint_names)
        if not self.planning_groups:
            self.planning_groups = [
                PlanningGroupDefinition(
                    name=FALLBACK_PLANNING_GROUP_NAME,
                    joint_names=tuple(self.joint_names),
                    base_link=self.base_link,
                    tip_link=self.end_effector_link,
                    source="fallback",
                )
            ]
