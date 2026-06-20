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

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from dimos.hardware.damiao.arm_adapter import DamiaoArmAdapter
from dimos.hardware.damiao.runtime import (
    _DEFAULT_ADDRESS,
    _DEFAULT_STATE_CACHE_TTL_S,
    _DEFAULT_TICK_DEADLINE_US,
    DamiaoBindingUnavailableError,
)
from dimos.hardware.damiao.specs import DamiaoArmSpec, DamiaoMotorSpec, DamiaoRobotSpec

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry


class OpenArmRSBindingUnavailableError(DamiaoBindingUnavailableError):
    pass


class OpenArmRSAdapter(DamiaoArmAdapter):
    _adapter_type: str = "openarm_rs"
    _binding_error_type: type[RuntimeError] = OpenArmRSBindingUnavailableError
    _DEFAULT_OPENARM_MOTORS: tuple[DamiaoMotorSpec, ...] = (
        DamiaoMotorSpec("joint1", "DM8006", 0x01, 0x11),
        DamiaoMotorSpec("joint2", "DM8006", 0x02, 0x12),
        DamiaoMotorSpec("joint3", "DM4340", 0x03, 0x13),
        DamiaoMotorSpec("joint4", "DM4340", 0x04, 0x14),
        DamiaoMotorSpec("joint5", "DM4310", 0x05, 0x15),
        DamiaoMotorSpec("joint6", "DM4310", 0x06, 0x16),
        DamiaoMotorSpec("joint7", "DM4310", 0x07, 0x17),
    )
    _POSITION_LOWER_LEFT: tuple[float, ...] = (-3.45, -3.30, -1.50, -0.01, -1.50, -0.75, -1.50)
    _POSITION_UPPER_LEFT: tuple[float, ...] = (1.35, 0.15, 1.50, 2.40, 1.50, 0.75, 1.50)
    _POSITION_LOWER_RIGHT: tuple[float, ...] = (-1.35, -0.15, -1.50, -0.01, -1.50, -0.75, -1.50)
    _POSITION_UPPER_RIGHT: tuple[float, ...] = (3.45, 3.30, 1.50, 2.40, 1.50, 0.75, 1.50)
    _DEFAULT_VELOCITY_MAX: tuple[float, ...] = (45.0, 45.0, 8.0, 8.0, 30.0, 30.0, 30.0)
    _DEFAULT_KP: tuple[float, ...] = (70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0)
    _DEFAULT_KD: tuple[float, ...] = (2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5)

    def __init__(
        self,
        address: str | Path | None = _DEFAULT_ADDRESS,
        dof: int = 7,
        *,
        hardware_id: str = "arm",
        config_path: str | Path | None = None,
        arm_name: str = "arm",
        bus_name: str = "can",
        fd: bool | None = None,
        canfd: bool = True,
        side: str = "left",
        use_mock_bus: bool = False,
        motor_specs: list[dict[str, object] | DamiaoMotorSpec] | None = None,
        position_lower: list[float] | None = None,
        position_upper: list[float] | None = None,
        velocity_max: list[float] | None = None,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        gravity_comp: bool = True,
        tick_deadline_us: int = _DEFAULT_TICK_DEADLINE_US,
        state_cache_ttl_s: float = _DEFAULT_STATE_CACHE_TTL_S,
        gravity_model_path: str | Path | None = None,
        gravity_torque_limits: list[float] | None = None,
    ) -> None:
        if dof != len(self._DEFAULT_OPENARM_MOTORS):
            raise ValueError(f"OpenArmRSAdapter only supports 7 DOF (got {dof})")
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        if motor_specs is not None:
            raise ValueError("openarm_rs is OpenArm-only and does not accept custom motor_specs")
        if position_lower is not None or position_upper is not None or velocity_max is not None:
            raise ValueError(
                "openarm_rs uses fixed OpenArm limits; custom limits require a separate adapter"
            )
        arm_spec = DamiaoArmSpec.from_values(
            name="openarm_rs",
            vendor="Enactic",
            model="OpenArm RS v10",
            motors=self._DEFAULT_OPENARM_MOTORS,
            position_lower=self._POSITION_LOWER_LEFT
            if side == "left"
            else self._POSITION_LOWER_RIGHT,
            position_upper=self._POSITION_UPPER_LEFT
            if side == "left"
            else self._POSITION_UPPER_RIGHT,
            velocity_max=self._DEFAULT_VELOCITY_MAX,
            kp=kp if kp is not None else self._DEFAULT_KP,
            kd=kd if kd is not None else self._DEFAULT_KD,
            gravity_model_path=gravity_model_path,
            gravity_torque_limits=gravity_torque_limits,
            bus_name=bus_name,
            arm_name=arm_name,
            fd=canfd if fd is None else fd,
        )
        robot_spec = DamiaoRobotSpec.from_arm_spec(
            arm_spec,
            address=str(address) if address is not None else _DEFAULT_ADDRESS,
        )
        super().__init__(
            robot_spec=robot_spec,
            group_name=arm_name,
            hardware_id=hardware_id,
            config_path=config_path,
            use_mock_bus=use_mock_bus,
            gravity_comp=gravity_comp,
            tick_deadline_us=tick_deadline_us,
            state_cache_ttl_s=state_cache_ttl_s,
        )


def register(registry: AdapterRegistry) -> None:
    registry.register("openarm_rs", OpenArmRSAdapter)


__all__ = [
    "OpenArmRSAdapter",
    "OpenArmRSBindingUnavailableError",
    "register",
]
