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

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import errno
from pathlib import Path
import time
from typing import Any

import can_motor_control
from can_motor_control import damiao
import numpy as np
import pinocchio  # type: ignore[import-not-found]

from dimos.hardware.damiao.config import DamiaoArmConfig, DamiaoRuntimeConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_ARM_NAME = "arm"
_BUS_NAME = "can"
_ENOBUFS_RETRY_DELAYS_S = (0.001, 0.002, 0.003)
_MIN_RECOMMENDED_TX_QUEUE_LEN = 1_000
_MOTOR_TYPES_BY_NAME = {
    "DM3507": damiao.MotorType.DM3507,
    "DM4310": damiao.MotorType.DM4310,
    "DM4310_48V": damiao.MotorType.DM4310_48V,
    "DM4340": damiao.MotorType.DM4340,
    "DM4340_48V": damiao.MotorType.DM4340_48V,
    "DM6006": damiao.MotorType.DM6006,
    "DM8006": damiao.MotorType.DM8006,
    "DM8009": damiao.MotorType.DM8009,
    "DM10010L": damiao.MotorType.DM10010L,
    "DM10010": damiao.MotorType.DM10010,
    "DMH3510": damiao.MotorType.DMH3510,
    "DMH6215": damiao.MotorType.DMH6215,
    "DMG6220": damiao.MotorType.DMG6220,
}
_MOTOR_TYPES_BY_VALUE = {
    int(motor_type): motor_type for motor_type in _MOTOR_TYPES_BY_NAME.values()
}


def _is_enobufs(exc: BaseException) -> bool:
    current: BaseException | None = exc
    while current is not None:
        if isinstance(current, OSError) and current.errno == errno.ENOBUFS:
            return True
        message = str(current).lower()
        if "no buffer space available" in message or "os error 105" in message:
            return True
        current = current.__cause__ or current.__context__
    return False


def _retry_enobufs(operation: Callable[[], None]) -> None:
    for delay_s in (*_ENOBUFS_RETRY_DELAYS_S, None):
        try:
            operation()
            return
        except Exception as exc:
            if delay_s is None or not _is_enobufs(exc):
                raise
            time.sleep(delay_s)


@dataclass(frozen=True)
class DamiaoGroupState:
    """State vectors for one Damiao arm."""

    q: list[float]
    dq: list[float]
    tau: list[float]


class DamiaoArmRuntime:
    """Binding-backed runtime for one Damiao arm."""

    def __init__(
        self,
        *,
        arm_config: DamiaoArmConfig,
        runtime_config: DamiaoRuntimeConfig,
        adapter_type: str = "damiao",
    ) -> None:
        self._arm_config = arm_config
        self._runtime_config = runtime_config
        self._adapter_type = adapter_type
        self._robot: Any | None = None
        self._arm: Any | None = None
        self._state_cache: DamiaoGroupState | None = None
        self._state_cache_time = 0.0
        self._connected = False
        self._enabled = False

    @property
    def arm_config(self) -> DamiaoArmConfig:
        return self._arm_config

    def connect(self) -> bool:
        """Connect the binding robot and cache its arm handle."""

        try:
            robot = self._build_robot()
            robot.connect()
            arm = robot[_ARM_NAME]
            if len(arm) != self._arm_config.dof:
                raise RuntimeError(
                    f"can_motor_control arm has {len(arm)} joints, expected {self._arm_config.dof}"
                )
            self._robot = robot
            self._arm = arm
            self._connected = True
            self.refresh_state(force=True)
        except Exception:
            logger.exception("damiao runtime connect failed", adapter=self._adapter_type)
            self.disconnect()
            return False
        return True

    def _build_robot(self) -> Any:
        if self._runtime_config.config_path is not None:
            return can_motor_control.Robot.from_config(str(self._runtime_config.config_path))

        address = self._runtime_config.address
        self._warn_if_small_tx_queue(address)
        transport: can_motor_control.MockCanBus | can_motor_control.SocketCanBus
        if self._runtime_config.use_mock_bus:
            transport = (
                can_motor_control.MockCanBus.new_fd(address)
                if self._arm_config.fd
                else can_motor_control.MockCanBus(address)
            )
        else:
            transport = can_motor_control.SocketCanBus(address, fd=self._arm_config.fd)
        motors = [
            can_motor_control.MotorSpec(
                motor.name,
                int(self._resolve_motor_type(motor.type)),
                motor.send_id,
                motor.effective_recv_id,
            )
            for motor in self._arm_config.motors
        ]
        return (
            can_motor_control.Robot.builder()
            .add_bus(_BUS_NAME, transport, damiao.DamiaoCodec())
            .add_arm(_ARM_NAME, bus=_BUS_NAME, motors=motors)
            .build()
        )

    def _warn_if_small_tx_queue(self, address: str) -> None:
        if self._runtime_config.use_mock_bus:
            return
        queue_path = Path("/sys/class/net") / address / "tx_queue_len"
        try:
            queue_len = int(queue_path.read_text().strip())
        except (OSError, ValueError):
            return
        if queue_len < _MIN_RECOMMENDED_TX_QUEUE_LEN:
            logger.warning(
                "CAN transmit queue is too small for reliable motor activation",
                interface=address,
                txqueuelen=queue_len,
                recommended=_MIN_RECOMMENDED_TX_QUEUE_LEN,
                setup_command=f"dimos can setup {address}",
            )

    @staticmethod
    def _resolve_motor_type(motor_type: str | int) -> damiao.MotorType:
        try:
            if isinstance(motor_type, str):
                return _MOTOR_TYPES_BY_NAME[motor_type]
            return _MOTOR_TYPES_BY_VALUE[motor_type]
        except KeyError as exc:
            raise ValueError(f"Unknown Damiao motor type {motor_type!r}") from exc

    def disconnect(self) -> None:
        """Disable and drop the underlying binding robot."""

        disabled = True
        if self._robot is not None:
            try:
                self._robot.disable()
            except Exception:
                logger.warning("damiao runtime disable on disconnect failed", exc_info=True)
                disabled = False
        self._enabled = False if disabled else True
        self._connected = False
        self._robot = None
        self._arm = None
        self._state_cache = None
        self._state_cache_time = 0.0

    def is_connected(self) -> bool:
        return self._connected

    def enable(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.set_mode("mit")
            self._robot.tick(self._runtime_config.tick_deadline_us)
            self._robot.enable()
            self._robot.tick(self._runtime_config.tick_deadline_us)
        except Exception:
            logger.exception("damiao runtime enable failed", adapter=self._adapter_type)
            try:
                disabled = self._robot.disable()
            except Exception:
                logger.warning("damiao runtime rollback disable failed", exc_info=True)
                disabled = False
            if disabled is not True:
                logger.error("damiao runtime partial enable could not disable hardware")
                self._enabled = True
            else:
                self._enabled = False
            return False
        self._enabled = True
        return True

    def disable(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.disable()
        except Exception:
            logger.exception("damiao runtime disable failed", adapter=self._adapter_type)
            return False
        self._enabled = False
        return True

    def is_enabled(self) -> bool:
        return self._enabled

    def refresh_state(self, *, force: bool = False) -> DamiaoGroupState:
        if self._robot is None or self._arm is None:
            raise RuntimeError("DamiaoArmRuntime is not connected")
        now = time.monotonic()
        if (
            not force
            and self._state_cache is not None
            and now - self._state_cache_time <= self._runtime_config.state_cache_ttl_s
        ):
            return self._state_cache
        self._arm.refresh()
        self._robot.tick(self._runtime_config.tick_deadline_us)
        state = DamiaoGroupState(
            q=self._arm.positions().astype(np.float64).tolist(),
            dq=self._arm.velocities().astype(np.float64).tolist(),
            tau=self._arm.torques().astype(np.float64).tolist(),
        )
        if any(len(values) != self._arm_config.dof for values in (state.q, state.dq, state.tau)):
            raise RuntimeError("state length does not match configured arm DOF")
        if any(
            not np.isfinite(values).all()
            for values in (np.asarray(state.q), np.asarray(state.dq), np.asarray(state.tau))
        ):
            raise RuntimeError("state contains non-finite values")
        self._state_cache = state
        self._state_cache_time = time.monotonic()
        return state

    def write_mit_commands(
        self,
        *,
        q: Sequence[float],
        dq: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
        tau: Sequence[float],
    ) -> bool:
        """Write one MIT command frame to the arm."""

        if self._robot is None or self._arm is None or not self._enabled:
            return False
        if any(len(values) != self._arm_config.dof for values in (q, dq, kp, kd, tau)):
            raise ValueError("command length does not match configured arm DOF")
        try:

            def send() -> None:
                assert self._arm is not None
                assert self._robot is not None
                self._arm.mit_control(np.column_stack([kp, kd, q, dq, tau]).astype(np.float64))
                self._robot.tick(self._runtime_config.tick_deadline_us)

            _retry_enobufs(send)
        except Exception:
            logger.exception("damiao runtime MIT command failed")
            return False
        self._state_cache = None
        self._state_cache_time = 0.0
        return True

    def load_gravity_model(self, model_path: str | Path) -> tuple[object, object]:
        """Load a Pinocchio gravity model for the arm."""

        model = pinocchio.buildModelFromUrdf(str(model_path))
        return model, model.createData()
