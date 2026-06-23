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

"""xArm planning model configuration helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.core.global_config import global_config
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

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

XARM_MODEL_PATH = LfsPath("xarm_description") / "urdf/xarm_device.urdf.xacro"
XARM_PACKAGE_PATHS: dict[str, Path] = {"xarm_description": LfsPath("xarm_description")}
XARM6_FK_MODEL = LfsPath("xarm_description/urdf/xarm6/xarm6.urdf")
XARM7_FK_MODEL = LfsPath("xarm_description/urdf/xarm7/xarm7.urdf")
XARM6_SIM_PATH = LfsPath("xarm6/scene.xml")
XARM7_SIM_PATH = LfsPath("xarm7/scene.xml")


def _adapter_kwargs(home_joints: list[float] | None = None) -> dict[str, object]:
    if home_joints is None:
        return {}
    return {"initial_positions": home_joints}


def make_xarm_hardware(
    hw_id: str,
    dof: int,
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    gripper: bool = False,
    auto_enable: bool = True,
    adapter_kwargs: dict[str, object] | None = None,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    kwargs = _adapter_kwargs(home_joints)
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, dof),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/gripper"] if gripper else [],
        adapter_kwargs=kwargs,
    )


def xarm7_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = False,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    if global_config.simulation:
        return make_xarm_hardware(
            hw_id,
            7,
            adapter_type="sim_mujoco",
            address=str(XARM7_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
        )
    address = global_config.xarm7_ip
    if mock_without_address and not address:
        return make_xarm_hardware(hw_id, 7, gripper=gripper, home_joints=home_joints)
    return make_xarm_hardware(
        hw_id,
        7,
        adapter_type="xarm",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
    )


def xarm6_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = False,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    if global_config.simulation:
        return make_xarm_hardware(
            hw_id,
            6,
            adapter_type="sim_mujoco",
            address=str(XARM6_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
        )
    address = global_config.xarm6_ip
    if mock_without_address and not address:
        return make_xarm_hardware(hw_id, 6, gripper=gripper, home_joints=home_joints)
    return make_xarm_hardware(
        hw_id,
        6,
        adapter_type="xarm",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
    )


def make_xarm_model_config(
    name: str,
    dof: int,
    *,
    add_gripper: bool = True,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    pitch: float = 0.0,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
    tf_extra_links: list[str] | None = None,
    home_joints: list[float] | None = None,
    pre_grasp_offset: float = 0.10,
) -> RobotModelConfig:
    xacro_args = {
        "dof": str(dof),
        "limited": "true",
        "attach_xyz": f"{x_offset} {y_offset} {z_offset}",
        "attach_rpy": f"0 {pitch} 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    return RobotModelConfig(
        name=name,
        model_path=XARM_MODEL_PATH,
        base_pose=base_pose(x_offset, y_offset, z_offset),
        joint_names=joint_names(dof),
        end_effector_link="link_tcp" if add_gripper else f"link{dof}",
        base_link="link_base",
        package_paths=XARM_PACKAGE_PATHS,
        xacro_args=xacro_args,
        auto_convert_meshes=True,
        collision_exclusion_pairs=(XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else []),
        joint_name_mapping=coordinator_joint_mapping(
            name,
            dof,
            joint_prefix=joint_prefix,
        ),
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name if add_gripper else None,
        tf_extra_links=tf_extra_links or [],
        home_joints=home_joints or [0.0] * dof,
        pre_grasp_offset=pre_grasp_offset,
    )


def make_xarm6_model_config(
    name: str = "arm",
    **kwargs: Any,
) -> RobotModelConfig:
    return make_xarm_model_config(name, 6, **kwargs)


def make_xarm7_model_config(
    name: str = "arm",
    **kwargs: Any,
) -> RobotModelConfig:
    return make_xarm_model_config(name, 7, **kwargs)
