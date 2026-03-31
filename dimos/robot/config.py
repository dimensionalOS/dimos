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

"""Unified robot configuration.

Single source of truth for a robot arm. The URDF/MJCF model file is the
ground truth — joint names, DOF, limits, and link hierarchy are parsed
automatically. Generates RobotModelConfig, HardwareComponent, and TaskConfig.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field, PrivateAttr

from dimos.robot.model_parser import ModelDescription, parse_model


class GripperConfig(BaseModel):
    """Gripper configuration."""

    type: str
    joints: list[str] = Field(default_factory=list)
    collision_exclusions: list[tuple[str, str]] = Field(default_factory=list)
    open_position: float = 1.0
    close_position: float = 0.0


from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class RobotConfig(BaseModel):
    """Unified robot configuration — URDF/MJCF is the ground truth.

    Model parsing is lazy to avoid LFS downloads at import time.
    """

    # Required fields
    name: str
    model_path: Path
    end_effector_link: str

    # Hardware connection
    adapter_type: str = "mock"
    address: str | None = None
    adapter_kwargs: dict[str, Any] = Field(default_factory=dict)
    auto_enable: bool = True

    # Optional overrides (derived from model if not set)
    joint_names: list[str] | None = None
    base_link: str | None = None
    home_joints: list[float] | None = None

    # Multi-robot / coordinator
    joint_prefix: str | None = None  # defaults to "{name}_"
    base_pose: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    # Planning
    max_velocity: float = 1.0
    max_acceleration: float = 2.0
    pre_grasp_offset: float = 0.10

    # Gripper
    gripper: GripperConfig | None = None

    # Model loading
    package_paths: dict[str, Path] = Field(default_factory=dict)
    xacro_args: dict[str, str] = Field(default_factory=dict)
    auto_convert_meshes: bool = True

    # TF publishing
    tf_extra_links: list[str] = Field(default_factory=list)

    # Task defaults
    task_type: str = "trajectory"
    task_priority: int = 10

    # Collision exclusion pairs (gripper-specific, cannot be parsed from model)
    collision_exclusion_pairs: list[tuple[str, str]] = Field(default_factory=list)

    _parsed: ModelDescription | None = PrivateAttr(default=None)

    def _ensure_parsed(self) -> ModelDescription:
        """Parse model lazily on first access."""
        if self._parsed is None:
            self._parsed = parse_model(self.model_path, self.package_paths, self.xacro_args)
            if self.joint_prefix is None:
                self.joint_prefix = f"{self.name}_"
            if self.joint_names is None:
                self.joint_names = self._parsed.actuated_joint_names
            if self.base_link is None:
                self.base_link = self._parsed.root_link
            if self.home_joints is None:
                self.home_joints = self._compute_default_home()
        return self._parsed

    def _compute_default_home(self) -> list[float]:
        assert self._parsed is not None
        home = []
        for joint_name in self.resolved_joint_names:
            joint = self._parsed.get_joint(joint_name)
            if (
                joint is not None
                and joint.lower_limit is not None
                and joint.upper_limit is not None
            ):
                home.append((joint.lower_limit + joint.upper_limit) / 2.0)
            else:
                home.append(0.0)
        return home

    # -- Derived properties ---------------------------------------------------

    @property
    def model_description(self) -> ModelDescription:
        return self._ensure_parsed()

    @property
    def resolved_joint_names(self) -> list[str]:
        self._ensure_parsed()
        assert self.joint_names is not None
        return self.joint_names

    @property
    def resolved_base_link(self) -> str:
        self._ensure_parsed()
        assert self.base_link is not None
        return self.base_link

    @property
    def dof(self) -> int:
        return len(self.resolved_joint_names)

    @property
    def coordinator_joint_names(self) -> list[str]:
        if not self.joint_prefix:
            return list(self.resolved_joint_names)
        return [f"{self.joint_prefix}{j}" for j in self.resolved_joint_names]

    @property
    def joint_name_mapping(self) -> dict[str, str]:
        if not self.joint_prefix:
            return {}
        return {f"{self.joint_prefix}{j}": j for j in self.resolved_joint_names}

    @property
    def coordinator_task_name(self) -> str:
        return f"traj_{self.name}"

    # -- Converter methods ----------------------------------------------------

    def to_robot_model_config(self) -> RobotModelConfig:
        """Generate RobotModelConfig for ManipulationModule."""
        bp = self.base_pose
        base_pose = PoseStamped(
            position=Vector3(x=bp[0], y=bp[1], z=bp[2]),
            orientation=Quaternion(bp[3], bp[4], bp[5], bp[6]),
        )

        exclusions = list(self.collision_exclusion_pairs)
        if self.gripper:
            exclusions.extend(self.gripper.collision_exclusions)

        return RobotModelConfig(
            name=self.name,
            urdf_path=self.model_path,
            base_pose=base_pose,
            joint_names=self.resolved_joint_names,
            end_effector_link=self.end_effector_link,
            base_link=self.resolved_base_link,
            package_paths=self.package_paths,
            xacro_args=self.xacro_args,
            collision_exclusion_pairs=exclusions,
            auto_convert_meshes=self.auto_convert_meshes,
            max_velocity=self.max_velocity,
            max_acceleration=self.max_acceleration,
            joint_name_mapping=self.joint_name_mapping,
            coordinator_task_name=self.coordinator_task_name,
            gripper_hardware_id=self.name if self.gripper else None,
            tf_extra_links=self.tf_extra_links,
            home_joints=self.home_joints,
            pre_grasp_offset=self.pre_grasp_offset,
        )

    def to_hardware_component(self) -> HardwareComponent:
        """Generate HardwareComponent for ControlCoordinator."""
        gripper_joints: list[str] = []
        if self.gripper and self.gripper.joints:
            gripper_joints = [f"{self.joint_prefix}{j}" for j in self.gripper.joints]

        adapter_kwargs = dict(self.adapter_kwargs)
        if self.home_joints is not None:
            adapter_kwargs.setdefault("initial_positions", self.home_joints)

        return HardwareComponent(
            hardware_id=self.name,
            hardware_type=HardwareType.MANIPULATOR,
            joints=self.coordinator_joint_names,
            adapter_type=self.adapter_type,
            address=self.address,
            auto_enable=self.auto_enable,
            gripper_joints=gripper_joints,
            adapter_kwargs=adapter_kwargs,
        )

    def to_task_config(self) -> TaskConfig:
        """Generate TaskConfig for ControlCoordinator."""
        return TaskConfig(
            name=self.coordinator_task_name,
            type=self.task_type,
            joint_names=self.coordinator_joint_names,
            priority=self.task_priority,
        )


__all__ = [
    "GripperConfig",
    "RobotConfig",
]
