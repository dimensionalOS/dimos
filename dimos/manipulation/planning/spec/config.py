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
        auto_convert_meshes: Auto-convert DAE/STL meshes to OBJ for Drake
        collision_exclusion_pairs: List of (link1, link2) pairs to exclude from collision.
            Useful for parallel linkage mechanisms like grippers where non-adjacent
            links may legitimately overlap (e.g., mimic joints).
    """

    model: RobotModel
    srdf_path: Path | None = None
    base_pose: PoseStamped = Field(default_factory=PoseStamped)
    joint_names: list[str]
    base_link: str = "base_link"
    planning_groups: list[PlanningGroupDefinition] = Field(default_factory=list)
    auto_convert_meshes: bool = False
    collision_exclusion_pairs: list[tuple[str, str]] = Field(default_factory=list)
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
