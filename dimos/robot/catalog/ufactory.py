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

"""UFactory xArm robot configurations."""

from __future__ import annotations

from typing import Any

from dimos.core.global_config import global_config
from dimos.robot.config import GripperConfig, RobotConfig
from dimos.utils.data import LfsPath

# Pre-built URDFs for Pinocchio FK (xacro not supported by Pinocchio)
XARM6_FK_MODEL = LfsPath("xarm_description/urdf/xarm6/xarm6.urdf")
XARM7_FK_MODEL = LfsPath("xarm_description/urdf/xarm7/xarm7.urdf")

# Simulation model paths (MJCF)
XARM7_SIM_PATH = LfsPath("xarm7/scene.xml")
XARM6_SIM_PATH = LfsPath("xarm6/scene.xml")

# XArm gripper collision exclusions (parallel linkage mechanism)
# The gripper uses mimic joints where non-adjacent links can overlap legitimately
XARM_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("right_inner_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "left_outer_knuckle"),
    ("right_inner_knuckle", "right_finger"),
    ("left_inner_knuckle", "left_finger"),
    ("left_finger", "right_finger"),
    ("left_outer_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "right_inner_knuckle"),
    ("left_outer_knuckle", "right_finger"),
    ("right_outer_knuckle", "left_finger"),
    ("xarm_gripper_base_link", "left_inner_knuckle"),
    ("xarm_gripper_base_link", "right_inner_knuckle"),
    ("xarm_gripper_base_link", "left_finger"),
    ("xarm_gripper_base_link", "right_finger"),
    ("link6", "xarm_gripper_base_link"),
    ("link6", "left_outer_knuckle"),
    ("link6", "right_outer_knuckle"),
]


def xarm7(
    name: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    add_gripper: bool = False,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    pitch: float = 0.0,
    tf_extra_links: list[str] | None = None,
    **overrides: Any,
) -> RobotConfig:
    """Create an xArm7 robot configuration."""
    xacro_args: dict[str, str] = {
        "dof": "7",
        "limited": "true",
        "attach_xyz": f"{x_offset} {y_offset} {z_offset}",
        "attach_rpy": f"0 {pitch} 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    defaults: dict[str, Any] = {
        "name": name,
        "model_path": LfsPath("xarm_description") / "urdf/xarm_device.urdf.xacro",
        "end_effector_link": "link_tcp" if add_gripper else "link7",
        "adapter_type": adapter_type,
        "address": address,
        "joint_names": [f"joint{i}" for i in range(1, 8)],
        "base_link": "link_base",
        "home_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "base_pose": [x_offset, y_offset, z_offset, 0, 0, 0, 1],
        "package_paths": {"xarm_description": LfsPath("xarm_description")},
        "xacro_args": xacro_args,
        "auto_convert_meshes": True,
        "collision_exclusion_pairs": XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else [],
        "tf_extra_links": tf_extra_links or [],
        "gripper": GripperConfig(
            type="xarm",
            joints=["gripper"],
            collision_exclusions=XARM_GRIPPER_COLLISION_EXCLUSIONS,
            open_position=0.85,
            close_position=0.0,
        )
        if add_gripper
        else None,
    }
    if global_config.simulation and adapter_type == "mock":
        defaults.update(adapter_type="sim_mujoco", address=str(XARM7_SIM_PATH))
        defaults.setdefault("adapter_kwargs", {})["headless"] = False

    defaults.update(overrides)
    return RobotConfig(**defaults)


def xarm6(
    name: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    add_gripper: bool = True,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    pitch: float = 0.0,
    tf_extra_links: list[str] | None = None,
    **overrides: Any,
) -> RobotConfig:
    """Create an xArm6 robot configuration."""
    xacro_args: dict[str, str] = {
        "dof": "6",
        "limited": "true",
        "attach_xyz": f"{x_offset} {y_offset} {z_offset}",
        "attach_rpy": f"0 {pitch} 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    defaults: dict[str, Any] = {
        "name": name,
        "model_path": LfsPath("xarm_description") / "urdf/xarm_device.urdf.xacro",
        "end_effector_link": "link_tcp" if add_gripper else "link6",
        "adapter_type": adapter_type,
        "address": address,
        "joint_names": [f"joint{i}" for i in range(1, 7)],
        "base_link": "link_base",
        "home_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "base_pose": [x_offset, y_offset, z_offset, 0, 0, 0, 1],
        "package_paths": {"xarm_description": LfsPath("xarm_description")},
        "xacro_args": xacro_args,
        "auto_convert_meshes": True,
        "collision_exclusion_pairs": XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else [],
        "tf_extra_links": tf_extra_links or [],
        "gripper": GripperConfig(
            type="xarm",
            joints=["gripper"],
            collision_exclusions=XARM_GRIPPER_COLLISION_EXCLUSIONS,
            open_position=0.85,
            close_position=0.0,
        )
        if add_gripper
        else None,
    }
    if global_config.simulation and adapter_type == "mock":
        defaults.update(adapter_type="sim_mujoco", address=str(XARM6_SIM_PATH))
        defaults.setdefault("adapter_kwargs", {})["headless"] = False

    defaults.update(overrides)
    return RobotConfig(**defaults)


__all__ = [
    "XARM6_FK_MODEL",
    "XARM7_FK_MODEL",
    "XARM_GRIPPER_COLLISION_EXCLUSIONS",
    "xarm6",
    "xarm7",
]
