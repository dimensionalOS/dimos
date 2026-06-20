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

from dimos.hardware.damiao.runtime import (
    _DEFAULT_STATE_CACHE_TTL_S,
    _DEFAULT_TICK_DEADLINE_US,
    DamiaoBindingUnavailableError,
)
from dimos.hardware.damiao.specs import (
    DamiaoBusSpec,
    DamiaoJointGroupSpec,
    DamiaoMotorSpec,
    DamiaoRobotSpec,
)
from dimos.hardware.damiao.whole_body_adapter import DamiaoWholeBodyAdapter
from dimos.utils.data import LfsPath

if TYPE_CHECKING:
    from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry


class OpenArmDualBindingUnavailableError(DamiaoBindingUnavailableError):
    pass


class OpenArmDualWholeBodyAdapter(DamiaoWholeBodyAdapter):
    """Dual-arm OpenArm whole-body adapter over the shared Damiao runtime."""

    _adapter_type: str = "openarm_dual"
    _binding_error_type: type[RuntimeError] = OpenArmDualBindingUnavailableError

    _LEFT_GROUP = "left_arm"
    _RIGHT_GROUP = "right_arm"
    _DEFAULT_MOTOR_TYPES: tuple[str, ...] = (
        "DM8006",
        "DM8006",
        "DM4340",
        "DM4340",
        "DM4310",
        "DM4310",
        "DM4310",
    )
    _DEFAULT_SEND_IDS: tuple[int, ...] = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07)
    _DEFAULT_RECV_IDS: tuple[int, ...] = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
    _POSITION_LOWER_LEFT: tuple[float, ...] = (-3.45, -3.30, -1.50, -0.01, -1.50, -0.75, -1.50)
    _POSITION_UPPER_LEFT: tuple[float, ...] = (1.35, 0.15, 1.50, 2.40, 1.50, 0.75, 1.50)
    _POSITION_LOWER_RIGHT: tuple[float, ...] = (-1.35, -0.15, -1.50, -0.01, -1.50, -0.75, -1.50)
    _POSITION_UPPER_RIGHT: tuple[float, ...] = (3.45, 3.30, 1.50, 2.40, 1.50, 0.75, 1.50)
    _DEFAULT_VELOCITY_MAX: tuple[float, ...] = (45.0, 45.0, 8.0, 8.0, 30.0, 30.0, 30.0)
    _DEFAULT_KP: tuple[float, ...] = (70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0)
    _DEFAULT_KD: tuple[float, ...] = (2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5)
    _OPENARM_PKG = LfsPath("openarm_description")
    _LEFT_GRAVITY_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_left.urdf"
    _RIGHT_GRAVITY_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_right.urdf"

    def __init__(
        self,
        address: str | Path | None = None,
        dof: int = 14,
        *,
        hardware_id: str = "openarm",
        domain_id: int = 0,
        left_address: str | Path = "can1",
        right_address: str | Path = "can0",
        canfd: bool = True,
        gravity_comp: bool = True,
        use_mock_bus: bool = False,
        tick_deadline_us: int = _DEFAULT_TICK_DEADLINE_US,
        state_cache_ttl_s: float = _DEFAULT_STATE_CACHE_TTL_S,
    ) -> None:
        del address, domain_id
        robot_spec = self._make_robot_spec(
            left_address=left_address,
            right_address=right_address,
            canfd=canfd,
        )
        super().__init__(
            robot_spec=robot_spec,
            group_names=(self._LEFT_GROUP, self._RIGHT_GROUP),
            dof=dof,
            hardware_id=hardware_id,
            gravity_comp=gravity_comp,
            use_mock_bus=use_mock_bus,
            tick_deadline_us=tick_deadline_us,
            state_cache_ttl_s=state_cache_ttl_s,
        )

    @classmethod
    def _make_robot_spec(
        cls,
        *,
        left_address: str | Path,
        right_address: str | Path,
        canfd: bool,
    ) -> DamiaoRobotSpec:
        return DamiaoRobotSpec(
            name="openarm_dual",
            vendor="Enactic",
            model="OpenArm RS v10 Dual",
            buses={
                "left_can": DamiaoBusSpec(address=left_address, fd=canfd),
                "right_can": DamiaoBusSpec(address=right_address, fd=canfd),
            },
            groups={
                cls._LEFT_GROUP: DamiaoJointGroupSpec(
                    bus_name="left_can",
                    motors=cls._motors("openarm_left"),
                    position_lower=cls._POSITION_LOWER_LEFT,
                    position_upper=cls._POSITION_UPPER_LEFT,
                    velocity_max=cls._DEFAULT_VELOCITY_MAX,
                    kp=cls._DEFAULT_KP,
                    kd=cls._DEFAULT_KD,
                    gravity_model_path=cls._LEFT_GRAVITY_MODEL,
                ),
                cls._RIGHT_GROUP: DamiaoJointGroupSpec(
                    bus_name="right_can",
                    motors=cls._motors("openarm_right"),
                    position_lower=cls._POSITION_LOWER_RIGHT,
                    position_upper=cls._POSITION_UPPER_RIGHT,
                    velocity_max=cls._DEFAULT_VELOCITY_MAX,
                    kp=cls._DEFAULT_KP,
                    kd=cls._DEFAULT_KD,
                    gravity_model_path=cls._RIGHT_GRAVITY_MODEL,
                ),
            },
            requires_binding=True,
        )

    @classmethod
    def _motors(cls, prefix: str) -> tuple[DamiaoMotorSpec, ...]:
        return tuple(
            DamiaoMotorSpec(
                name=f"{prefix}_joint{index + 1}",
                type=motor_type,
                send_id=send_id,
                recv_id=recv_id,
            )
            for index, (motor_type, send_id, recv_id) in enumerate(
                zip(
                    cls._DEFAULT_MOTOR_TYPES,
                    cls._DEFAULT_SEND_IDS,
                    cls._DEFAULT_RECV_IDS,
                    strict=True,
                )
            )
        )


def register(registry: WholeBodyAdapterRegistry) -> None:
    registry.register("openarm_dual", OpenArmDualWholeBodyAdapter)


__all__ = ["OpenArmDualBindingUnavailableError", "OpenArmDualWholeBodyAdapter", "register"]
