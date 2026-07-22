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

"""Galaxea A1Z planning model configuration helpers."""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.core.global_config import global_config


def make_galaxea_a1z_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    gripper: bool = True,
    auto_enable: bool = True,
    adapter_kwargs: dict[str, object] | None = None,
) -> HardwareComponent:
    kwargs: dict[str, object] = {"gripper": gripper} if adapter_type == "galaxea_a1z" else {}
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, 6),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        # G1Z gripper needs the a1z SDK's 'gripper' branch
        gripper_joints=[f"{hw_id}/gripper"] if gripper else [],
        adapter_kwargs=kwargs,
    )


def galaxea_a1z_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = True,
    mock_without_address: bool = False,
    dynamics_urdf_path: str | None = None,
    adapter_kwargs: dict[str, object] | None = None,
) -> HardwareComponent:
    if global_config.simulation:
        # TODO: Add sim support when A1Z MuJoCo model is available
        return make_galaxea_a1z_hardware(hw_id, gripper=gripper)
    address = global_config.can_port or "a1zcan"
    if mock_without_address and not global_config.can_port:
        return make_galaxea_a1z_hardware(hw_id, gripper=gripper)
    resolved_adapter_kwargs = dict(adapter_kwargs or {})
    if dynamics_urdf_path is not None:
        resolved_adapter_kwargs["urdf_path"] = dynamics_urdf_path
    return make_galaxea_a1z_hardware(
        hw_id,
        adapter_type="galaxea_a1z",
        address=address,
        gripper=gripper,
        adapter_kwargs=resolved_adapter_kwargs,
    )
