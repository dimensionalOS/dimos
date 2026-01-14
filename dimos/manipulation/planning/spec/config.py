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

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


@dataclass
class RobotModelConfig:
    """Configuration for adding a robot to the world.

    Attributes:
        name: Human-readable robot name
        urdf_path: Path to URDF file (can be .urdf or .xacro)
        base_pose: 4x4 homogeneous transform for robot base
        joint_names: Ordered list of controlled joint names (in URDF namespace)
        end_effector_link: Name of the end-effector link for FK/IK
        base_link: Name of the base link (default: "base_link")
        package_paths: Dict mapping package names to filesystem paths
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
        joint_name_mapping: Maps orchestrator joint names to URDF joint names.
            Example: {"left_joint1": "joint1"} means orchestrator's "left_joint1"
            corresponds to URDF's "joint1". If empty, names are assumed to match.
        orchestrator_task_name: Task name for executing trajectories via orchestrator RPC.
            If set, trajectories can be executed via execute_trajectory() RPC.
    """

    name: str
    urdf_path: str
    base_pose: NDArray[np.float64]
    joint_names: list[str]
    end_effector_link: str
    base_link: str = "base_link"
    package_paths: dict[str, str] = field(default_factory=dict)
    joint_limits_lower: NDArray[np.float64] | None = None
    joint_limits_upper: NDArray[np.float64] | None = None
    velocity_limits: NDArray[np.float64] | None = None
    auto_convert_meshes: bool = False
    xacro_args: dict[str, str] = field(default_factory=dict)
    collision_exclusion_pairs: list[tuple[str, str]] = field(default_factory=list)
    # Motion constraints for trajectory generation
    max_velocity: float = 1.0
    max_acceleration: float = 2.0
    # Orchestrator integration
    joint_name_mapping: dict[str, str] = field(default_factory=dict)
    orchestrator_task_name: str | None = None

    def get_urdf_joint_name(self, orchestrator_name: str) -> str:
        """Translate orchestrator joint name to URDF joint name."""
        return self.joint_name_mapping.get(orchestrator_name, orchestrator_name)

    def get_orchestrator_joint_name(self, urdf_name: str) -> str:
        """Translate URDF joint name to orchestrator joint name."""
        for orch_name, u_name in self.joint_name_mapping.items():
            if u_name == urdf_name:
                return orch_name
        return urdf_name

    def get_orchestrator_joint_names(self) -> list[str]:
        """Get joint names in orchestrator namespace."""
        if not self.joint_name_mapping:
            return self.joint_names
        return [self.get_orchestrator_joint_name(j) for j in self.joint_names]
