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

from collections.abc import Sequence
from pathlib import Path
from typing import Any

from pydantic import Field, field_validator

from dimos.core.module import ModuleConfig
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.planning.groups.identifiers import assert_valid_joint_names
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.spec.gripper import GripperConfig
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
        gripper: Where the gripper may touch a grasped object and how it closes,
            for checking grasp candidates; None skips that check.
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
    gripper_hardware_ids: dict[str, str] = Field(default_factory=dict)
    # TF publishing for extra links (e.g., camera mount)
    tf_extra_links: list[str] = Field(default_factory=list)
    # Home/observe joint configuration for go_home skill
    home_joints: list[float] | None = None
    # Pre-grasp offset distance in meters (along approach direction)
    pre_grasp_offset: float = 0.10
    # Gripper feedback thresholds for pick/place.
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)
    gripper: GripperConfig | None = None

    @field_validator("base_pose", mode="before")
    @classmethod
    def _parse_base_pose(cls, value: Any) -> Any:
        """Accept a pose object or its JSON configuration across the launcher boundary."""
        return PoseStamped(**value) if isinstance(value, dict) else value

    def model_post_init(self, __context: object) -> None:
        """Validate canonical joint-name constraints."""
        if self.gripper_hardware_id is not None and self.gripper_hardware_ids:
            raise ValueError("Use either gripper_hardware_id or gripper_hardware_ids, not both")
        if any(not group or not hardware for group, hardware in self.gripper_hardware_ids.items()):
            raise ValueError("Gripper group and hardware ids must be non-empty")
        if len(set(self.gripper_hardware_ids.values())) != len(self.gripper_hardware_ids):
            raise ValueError("Each planning group requires a distinct gripper hardware id")
        if self.planning_groups:
            self.resolve_gripper_hardware_ids(self.planning_groups)
        assert_valid_joint_names(self.joint_names)
        if any(not name for name in self.joint_names):
            raise ValueError("RobotModelConfig.joint_names must contain non-empty names")
        if len(self.joint_names) != len(set(self.joint_names)):
            raise ValueError("RobotModelConfig contains duplicate canonical joint names")

    def resolve_gripper_hardware_ids(
        self, groups: Sequence[PlanningGroup | PlanningGroupDefinition]
    ) -> dict[str, str]:
        """Resolve explicit group bindings or an unambiguous legacy scalar binding."""
        pose_groups = {
            str(group.name if isinstance(group, PlanningGroupDefinition) else group.id)
            for group in groups
            if group.has_pose_target
        }
        if self.gripper_hardware_id is not None:
            if len(pose_groups) != 1:
                raise ValueError(
                    "gripper_hardware_id requires exactly one pose-capable planning group"
                )
            return {next(iter(pose_groups)): self.gripper_hardware_id}
        invalid = set(self.gripper_hardware_ids) - pose_groups
        if invalid:
            raise ValueError(f"Gripper bindings require pose-capable groups: {sorted(invalid)}")
        return dict(self.gripper_hardware_ids)
