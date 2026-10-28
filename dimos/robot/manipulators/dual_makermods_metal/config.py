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

"""Two independently connected Metal arms, with canonical per-side names."""

from typing import Literal

from dimos.control.components import HardwareComponent
from dimos.robot.manipulators.makermods_metal.config import METAL_ARM_JOINTS, metal_hardware

SIDES = ("left", "right")
ARM_JOINTS = {side: tuple(f"{side}_{joint}" for joint in METAL_ARM_JOINTS) for side in SIDES}
ALL_ARM_JOINTS = (*ARM_JOINTS["left"], *ARM_JOINTS["right"])


def dual_metal_hardware(
    *,
    left_can_port: str | None = None,
    right_can_port: str | None = None,
    left_gripper_opening_direction: Literal["increasing_position", "decreasing_position"]
    | None = None,
    right_gripper_opening_direction: Literal["increasing_position", "decreasing_position"]
    | None = None,
) -> list[HardwareComponent]:
    """Require two distinct explicit CAN selectors before selecting physical arms."""
    ports = (left_can_port, right_can_port)
    if any(port is not None for port in ports):
        if any(port is None or not port.strip() for port in ports):
            raise ValueError("dual Metal requires both non-empty CAN selectors")
        if left_can_port == right_can_port:
            raise ValueError("dual Metal requires distinct left and right CAN selectors")
        if left_gripper_opening_direction is None or right_gripper_opening_direction is None:
            raise ValueError("dual Metal requires both gripper opening directions")
    components = []
    for side, port, direction in zip(
        SIDES, ports, (left_gripper_opening_direction, right_gripper_opening_direction), strict=True
    ):
        component = metal_hardware(can_port=port, gripper_opening_direction=direction)
        component.hardware_id = f"{side}_arm"
        # ConnectedWholeBody maps canonical component names to the adapter by index.
        component.joints = [*ARM_JOINTS[side], f"{side}_arm/gripper"]
        components.append(component)
    return components
