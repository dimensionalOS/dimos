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

"""MakerMods Metal Arm model and mock-default hardware configuration."""

from __future__ import annotations

from typing import Literal

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

METAL_SOURCE = RobotDescriptionSource(
    url="https://github.com/makermods-robotics/metal-python-ros",
    ref="bccf3c82a21453a7b1784a94966456e16ff0f008",
)
METAL_MODEL = RobotModel.from_file(
    METAL_SOURCE / "metal_ros2/src/metal_description/urdf/metal_with_gripper.urdf",
    package_paths={"metal_description": METAL_SOURCE / "metal_ros2/src/metal_description"},
).with_fixed_joints("joint7", "joint8")
METAL_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
METAL_GRIPPER_JOINT = "arm/gripper"
METAL_JOINTS = (*METAL_ARM_JOINTS, METAL_GRIPPER_JOINT)
METAL_HARDWARE_ID = "metal"


def metal_hardware(
    *,
    can_port: str | None = None,
    gripper_opening_direction: Literal["increasing_position", "decreasing_position"] = (
        "increasing_position"
    ),
    gripper_current: float = 0.1,
) -> HardwareComponent:
    """Select physical hardware with an explicit CAN selector."""
    adapter_kwargs: dict[str, object] = {}
    if can_port is not None:
        if not can_port.strip():
            raise ValueError("Metal CAN selector must not be empty")
        if not 0.0 < gripper_current <= 1.0:
            raise ValueError("Metal gripper current must be in (0, 1] per-unit")
        adapter_kwargs = {
            "runtime_config": DamiaoRuntimeConfig(
                bus_devices={"metal": can_port}, gravity_comp=False
            ),
            "gripper_opening_direction": gripper_opening_direction,
            "gripper_current": gripper_current,
        }
    return HardwareComponent(
        hardware_id=METAL_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(METAL_JOINTS),
        adapter_type="mock_whole_body" if can_port is None else "makermods_metal",
        auto_enable=True,
        limits=JointLimits(
            position_lower=[*([None] * 6), 0.0],
            position_upper=[*([None] * 6), 1.0],
            velocity_max=[None] * 7,
        )
        if can_port is None
        else None,
        adapter_kwargs=adapter_kwargs,
        # MakerMods LeRobot follow gains; gripper uses the shared opening controller.
        wb_config=WholeBodyConfig(
            kp=(100.0, 100.0, 100.0, 50.0, 20.0, 20.0, 0.0),
            kd=(5.0, 5.0, 5.0, 5.0, 0.6, 0.6, 0.0),
        ),
    )
