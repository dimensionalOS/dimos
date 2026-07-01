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

from pydantic import Field

from dimos.core.module import ModuleConfig
from dimos.manipulation.planning.groups.identifiers import (
    assert_local_joint_names,
    assert_valid_robot_name,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class RobotModelConfig(ModuleConfig):
    """Configuration for adding a robot to the world.

    Attributes:
        name: Human-readable robot name
        model_path: Path to robot model file (.urdf, .xacro, or .xml/MJCF)
        srdf_path: Optional path to SRDF file containing planning group definitions
        base_pose: Placement transform. This is the canonical world placement for
            robot instances.
        joint_names: Ordered list of controllable joints in the local model
            namespace. This is not a planning group.
        base_link: Robot-scoped link that base_pose places in the world and
            current backends use for weld/placement.
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
        joint_name_mapping: Maps coordinator joint names to local model joint names.
            This is retained for current coordinator/monitor integrations while planning
            APIs move toward globally scoped joint names.
        coordinator_task_name: Task name for executing trajectories via coordinator RPC.
            If set, trajectories can be executed via execute_trajectory() RPC.
    """

    name: str
    model_path: Path
    srdf_path: Path | None = None
    base_pose: PoseStamped = Field(default_factory=PoseStamped)
    joint_names: list[str]
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
    joint_name_mapping: dict[str, str] = Field(default_factory=dict)
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

    @property
    def end_effector_link(self) -> str:
        """Compatibility pose target frame derived from planning groups.

        Current world, IK, and visualization layers still ask robot configs for
        one end-effector link. The planning-group model stores that frame as a
        group ``tip_link``; this shim keeps those layers working until they are
        migrated to explicit planning-group IDs.
        """
        pose_tip_links = [
            group.tip_link for group in self.planning_groups if group.tip_link is not None
        ]
        if not pose_tip_links:
            raise ValueError(
                f"RobotModelConfig '{self.name}' has no pose-target planning group; "
                "define PlanningGroupDefinition.tip_link"
            )
        unique_tip_links = list(dict.fromkeys(pose_tip_links))
        if len(unique_tip_links) > 1:
            raise ValueError(
                f"RobotModelConfig '{self.name}' has multiple pose-target planning groups; "
                "use an explicit planning group ID"
            )
        return unique_tip_links[0]

    def get_urdf_joint_name(self, coordinator_name: str) -> str:
        """Translate coordinator joint name to local model joint name."""
        return self.joint_name_mapping.get(coordinator_name, coordinator_name)

    def get_coordinator_joint_name(self, urdf_name: str) -> str:
        """Translate local model joint name to coordinator joint name."""
        for coord_name, model_name in self.joint_name_mapping.items():
            if model_name == urdf_name:
                return coord_name
        return urdf_name

    def get_coordinator_joint_names(self) -> list[str]:
        """Get joint names in coordinator namespace."""
        if not self.joint_name_mapping:
            return self.joint_names
        return [self.get_coordinator_joint_name(joint_name) for joint_name in self.joint_names]
