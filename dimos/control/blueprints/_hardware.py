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

"""Hardware component factories for coordinator blueprints."""

from __future__ import annotations

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_gripper_joints,
    make_joints,
    make_twist_base_joints,
)
from dimos.core.global_config import global_config
from dimos.utils.data import LfsPath

XARM7_IP = global_config.xarm7_ip
XARM6_IP = global_config.xarm6_ip
CAN_PORT = global_config.can_port

PIPER_MODEL_PATH = LfsPath("piper_description/mujoco_model/piper_no_gripper_description.xml")
XARM6_MODEL_PATH = LfsPath("xarm_description/urdf/xarm6/xarm6.urdf")
XARM7_MODEL_PATH = LfsPath("xarm_description/urdf/xarm7/xarm7.urdf")


def mock_arm(hw_id: str = "arm", n_joints: int = 7) -> HardwareComponent:
    """Mock manipulator (no real hardware)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, n_joints),
        adapter_type="mock",
    )


def xarm7(hw_id: str = "arm", *, gripper: bool = False) -> HardwareComponent:
    """XArm7 real hardware (7-DOF)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, 7),
        adapter_type="xarm",
        address=XARM7_IP,
        auto_enable=True,
        gripper_joints=make_gripper_joints(hw_id) if gripper else [],
    )


def xarm6(hw_id: str = "arm", *, gripper: bool = False) -> HardwareComponent:
    """XArm6 real hardware (6-DOF)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, 6),
        adapter_type="xarm",
        address=XARM6_IP,
        auto_enable=True,
        gripper_joints=make_gripper_joints(hw_id) if gripper else [],
    )


def piper(hw_id: str = "arm") -> HardwareComponent:
    """Piper arm (6-DOF, CAN bus)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, 6),
        adapter_type="piper",
        address=CAN_PORT,
        auto_enable=True,
    )


def mock_twist_base(hw_id: str = "base") -> HardwareComponent:
    """Mock holonomic twist base (3-DOF: vx, vy, wz)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="mock_twist_base",
    )
