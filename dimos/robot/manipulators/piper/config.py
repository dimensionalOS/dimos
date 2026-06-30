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

"""Piper planning model configuration helpers."""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.core.global_config import global_config
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import (
    base_pose,
    joint_names,
)
from dimos.utils.data import LfsPath

PIPER_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("gripper_base", "link7"),
    ("gripper_base", "link8"),
    ("link7", "link8"),
    ("link6", "gripper_base"),
]

PIPER_MODEL_PATH = LfsPath("piper_description") / "urdf/piper_description.xacro"
PIPER_PACKAGE_PATHS: dict[str, Path] = {
    "piper_description": LfsPath("piper_description"),
    "piper_gazebo": LfsPath("piper_description"),
}
PIPER_FK_MODEL = LfsPath("piper_description/mujoco_model/piper_no_gripper_description.xml")
PIPER_SIM_PATH = LfsPath("piper/scene.xml")


def _adapter_kwargs(home_joints: list[float] | None = None) -> dict[str, object]:
    if home_joints is None:
        return {}
    return {"initial_positions": home_joints}


def make_piper_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    gripper: bool = True,
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
        joints=make_joints(hw_id, 6),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/gripper"] if gripper else [],
        adapter_kwargs=kwargs,
    )


def piper_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = True,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    if global_config.simulation:
        return make_piper_hardware(
            hw_id,
            adapter_type="sim_mujoco",
            address=str(PIPER_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
        )
    address = global_config.can_port or "can0"
    if mock_without_address and not global_config.can_port:
        return make_piper_hardware(hw_id, gripper=gripper, home_joints=home_joints)
    return make_piper_hardware(
        hw_id,
        adapter_type="piper",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
    )


def make_piper_model_config(
    name: str = "arm",
    *,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    _ = joint_prefix
    dof = 6
    return RobotModelConfig(
        name=name,
        model_path=PIPER_MODEL_PATH,
        base_pose=base_pose(),
        joint_names=joint_names(dof),
        end_effector_link="gripper_base",
        base_link="base_link",
        package_paths=PIPER_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=PIPER_GRIPPER_COLLISION_EXCLUSIONS,
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name,
        home_joints=home_joints or [0.0] * dof,
    )
