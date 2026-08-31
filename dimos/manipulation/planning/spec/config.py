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

import math
from pathlib import Path

from pydantic import Field

from dimos.core.module import ModuleConfig
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.planning.groups.identifiers import assert_valid_joint_names
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.assets.model import RobotModel


class RobotModelConfig(ModuleConfig):
    """Configuration for the logical robot model loaded into the world.

    Attributes:
        model: Portable robot model loaded by backend adapters
        srdf_path: Optional path to SRDF file containing planning group definitions
        base_pose: Placement transform for the model's base link in the world.
        joint_names: Ordered list of controllable joints in the canonical model
            namespace. This is not a planning group.
        base_link: Robot-scoped link that base_pose places in the world and
            current backends use for weld/placement.
        joint_limits_lower: Lower limits in each joint's native coordinate unit
        joint_limits_upper: Upper limits in each joint's native coordinate unit
        velocity_limits: Velocity limits in each joint's native coordinate unit per second
        auto_convert_meshes: Auto-convert DAE/STL meshes to OBJ for Drake
        collision_exclusion_pairs: List of (link1, link2) pairs to exclude from collision.
            Useful for parallel linkage mechanisms like grippers where non-adjacent
            links may legitimately overlap (e.g., mimic joints).
        max_velocity: Maximum native-coordinate velocity for trajectory generation
        max_acceleration: Maximum native-coordinate acceleration for trajectory generation
    """

    model: RobotModel
    srdf_path: Path | None = None
    base_pose: PoseStamped = Field(default_factory=PoseStamped)
    joint_names: list[str]
    base_link: str = "base_link"
    planning_groups: list[PlanningGroupDefinition] = Field(default_factory=list)
    joint_limits_lower: list[float] | None = None
    joint_limits_upper: list[float] | None = None
    velocity_limits: list[float] | None = None
    auto_convert_meshes: bool = False
    collision_exclusion_pairs: list[tuple[str, str]] = Field(default_factory=list)
    # Motion constraints for trajectory generation
    max_velocity: float = 1.0
    max_acceleration: float = 2.0
    gripper_hardware_id: str | None = None
    # TF publishing for extra links (e.g., camera mount)
    tf_extra_links: list[str] = Field(default_factory=list)
    # Home/observe joint configuration for go_home skill
    home_joints: list[float] | None = None
    # Pre-grasp offset distance in meters (along approach direction)
    pre_grasp_offset: float = 0.10
    # Gripper feedback thresholds for pick/place.
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)

    def model_post_init(self, __context: object) -> None:
        """Validate canonical joint-name constraints."""
        assert_valid_joint_names(self.joint_names)
        if any(not name for name in self.joint_names):
            raise ValueError("RobotModelConfig.joint_names must contain non-empty names")
        if len(self.joint_names) != len(set(self.joint_names)):
            raise ValueError("RobotModelConfig contains duplicate canonical joint names")

    def canonical_velocity_limits(self) -> list[float]:
        """Return positive velocity limits in canonical joint order."""
        if self.velocity_limits is None:
            limits = [float(self.max_velocity)] * len(self.joint_names)
        else:
            limits = [float(value) for value in self.velocity_limits]
            if len(limits) != len(self.joint_names):
                raise ValueError("velocity_limits must match joint_names")

        planar_base = self.model.planar_base
        if planar_base is not None:
            by_name = dict(zip(planar_base.joint_names, planar_base.velocity_limits, strict=True))
            limits = [
                float(by_name.get(name, limit))
                for name, limit in zip(self.joint_names, limits, strict=True)
            ]
        if any(not math.isfinite(value) or value <= 0.0 for value in limits):
            raise ValueError("Velocity limits must be positive and finite")
        return limits

    def canonical_acceleration_limits(self) -> list[float]:
        """Return positive acceleration limits in canonical joint order."""
        limits = [float(self.max_acceleration)] * len(self.joint_names)
        planar_base = self.model.planar_base
        if planar_base is not None:
            by_name = dict(
                zip(
                    planar_base.joint_names,
                    planar_base.acceleration_limits,
                    strict=True,
                )
            )
            limits = [
                float(by_name.get(name, limit))
                for name, limit in zip(self.joint_names, limits, strict=True)
            ]
        if any(not math.isfinite(value) or value <= 0.0 for value in limits):
            raise ValueError("Acceleration limits must be positive and finite")
        return limits
