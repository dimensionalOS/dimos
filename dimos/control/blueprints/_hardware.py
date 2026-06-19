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

"""Hardware component helpers for coordinator blueprints."""

from __future__ import annotations

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_joints,
    make_twist_base_joints,
)
from dimos.core.global_config import global_config
from dimos.utils.data import LfsPath

PIPER_FK_MODEL = LfsPath("piper_description/mujoco_model/piper_no_gripper_description.xml")
XARM6_FK_MODEL = LfsPath("xarm_description/urdf/xarm6/xarm6.urdf")
XARM7_FK_MODEL = LfsPath("xarm_description/urdf/xarm7/xarm7.urdf")
A750_FK_MODEL = LfsPath("a750_description/urdf/a750_rev1_no_gripper.urdf")

XARM7_SIM_PATH = LfsPath("xarm7/scene.xml")
XARM6_SIM_PATH = LfsPath("xarm6/scene.xml")
PIPER_SIM_PATH = LfsPath("piper/scene.xml")


def _adapter_kwargs(home_joints: list[float] | None = None) -> dict[str, object]:
    if home_joints is None:
        return {}
    return {"initial_positions": home_joints}


def manipulator(
    hw_id: str,
    dof: int,
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    gripper: bool = False,
    gripper_suffix: str = "gripper",
    auto_enable: bool = True,
    adapter_kwargs: dict[str, object] | None = None,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Create a manipulator hardware component with DimOS joint names."""
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
        gripper_joints=[f"{hw_id}/{gripper_suffix}"] if gripper else [],
        adapter_kwargs=kwargs,
    )


def mock_arm(
    hw_id: str = "arm",
    dof: int = 7,
    *,
    gripper: bool = False,
    gripper_suffix: str = "gripper",
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Mock manipulator with no real hardware."""
    return manipulator(
        hw_id,
        dof,
        gripper=gripper,
        gripper_suffix=gripper_suffix,
        home_joints=home_joints,
    )


def xarm7(
    hw_id: str = "arm",
    *,
    gripper: bool = False,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """xArm7 hardware, MuJoCo when --simulation is set, or mock if requested."""
    if global_config.simulation:
        return manipulator(
            hw_id,
            7,
            adapter_type="sim_mujoco",
            address=str(XARM7_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
        )
    address = global_config.xarm7_ip
    if mock_without_address and not address:
        return mock_arm(hw_id, 7, gripper=gripper, home_joints=home_joints)
    return manipulator(
        hw_id,
        7,
        adapter_type="xarm",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
    )


def xarm6(
    hw_id: str = "arm",
    *,
    gripper: bool = False,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """xArm6 hardware, MuJoCo when --simulation is set, or mock if requested."""
    if global_config.simulation:
        return manipulator(
            hw_id,
            6,
            adapter_type="sim_mujoco",
            address=str(XARM6_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
        )
    address = global_config.xarm6_ip
    if mock_without_address and not address:
        return mock_arm(hw_id, 6, gripper=gripper, home_joints=home_joints)
    return manipulator(
        hw_id,
        6,
        adapter_type="xarm",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
    )


def piper(
    hw_id: str = "arm",
    *,
    gripper: bool = True,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Piper hardware, MuJoCo when --simulation is set, or mock if requested."""
    if global_config.simulation:
        return manipulator(
            hw_id,
            6,
            adapter_type="sim_mujoco",
            address=str(PIPER_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
        )
    address = global_config.can_port or "can0"
    if mock_without_address and not global_config.can_port:
        return mock_arm(hw_id, 6, gripper=gripper, home_joints=home_joints)
    return manipulator(
        hw_id,
        6,
        adapter_type="piper",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
    )


def a750(hw_id: str = "arm", *, mock_without_address: bool = False) -> HardwareComponent:
    """A-750 hardware or mock when no device path is configured."""
    home_joints = [0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0]
    if mock_without_address and not global_config.device_path:
        return mock_arm(
            hw_id,
            6,
            gripper=True,
            gripper_suffix="finger",
            home_joints=home_joints,
        )
    return manipulator(
        hw_id,
        6,
        adapter_type="a750",
        address=global_config.device_path or "/dev/ttyACM0",
        gripper=True,
        gripper_suffix="finger",
        home_joints=home_joints,
    )


def mock_twist_base(hw_id: str = "base") -> HardwareComponent:
    """Mock holonomic twist base (3-DOF: vx, vy, wz)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="mock_twist_base",
    )
