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

"""Seeed B601-DM model and mock-default hardware configuration."""

from __future__ import annotations

from typing import Literal

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

B601_DM_SOURCE = RobotDescriptionSource(
    url="https://github.com/Seeed-Projects/reBot-DevArm",
    ref="8494bdfa91f4e3712bb7e7743e7bbcc63b01b9af",
)
B601_DM_MODEL = RobotModel.from_file(
    B601_DM_SOURCE / "Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf",
).with_fixed_joints("finger_left", "finger_right")
B601_DM_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
B601_DM_GRIPPER_JOINT = "arm/gripper"
B601_DM_JOINTS = (*B601_DM_ARM_JOINTS, B601_DM_GRIPPER_JOINT)
B601_DM_HARDWARE_ID = "b601_dm"


def b601_dm_hardware(
    *,
    can_port: str | None = None,
    gripper_opening_direction: Literal["increasing_position", "decreasing_position"] | None = None,
    gripper_current: float | None = None,
) -> HardwareComponent:
    """Require explicit CAN and gripper settings before selecting physical hardware."""
    adapter_kwargs: dict[str, object] = {}
    if can_port is not None:
        if not can_port.strip():
            raise ValueError("B601 CAN selector must not be empty")
        if gripper_opening_direction is None or gripper_current is None:
            raise ValueError(
                "physical B601 requires gripper opening direction and per-unit current"
            )
        if not 0.0 < gripper_current <= 1.0:
            raise ValueError("B601 gripper current must be in (0, 1] per-unit")
        adapter_kwargs = {
            "runtime_config": DamiaoRuntimeConfig(
                bus_devices={"b601_dm": can_port}, gravity_comp=False
            ),
            "gripper_opening_direction": gripper_opening_direction,
            "gripper_current": gripper_current,
        }
    return HardwareComponent(
        hardware_id=B601_DM_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(B601_DM_JOINTS),
        adapter_type="mock_whole_body" if can_port is None else "seeed_b601_dm",
        auto_enable=True,
        limits=JointLimits(
            position_lower=[*([None] * 6), 0.0],
            position_upper=[*([None] * 6), 1.0],
            velocity_max=[None] * 7,
        )
        if can_port is None
        else None,
        adapter_kwargs=adapter_kwargs,
        # Seeed MIT kp values. First three kd values are capped at the shared
        # codec's 5.0 maximum rather than forwarding the SDK's 8.0 unchanged.
        wb_config=WholeBodyConfig(
            kp=(120.0, 120.0, 120.0, 18.0, 18.0, 18.0, 0.0),
            kd=(5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 0.0),
        ),
    )
