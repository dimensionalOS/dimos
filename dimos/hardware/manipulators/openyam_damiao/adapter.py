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

"""OpenYAM's six-axis Damiao adapter."""

from __future__ import annotations

import math
from pathlib import Path

import attrs

from dimos.hardware.damiao.arm_adapter import DamiaoArmAdapter
from dimos.hardware.damiao.config import (
    DamiaoArmConfig,
    DamiaoGripperConfig,
    DamiaoMotorConfig,
    DamiaoRuntimeConfig,
)
from dimos.robot.model_parser import parse_model
from dimos.utils.data import LfsPath
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_OPENYAM_MODEL_PATH = Path(LfsPath("yam_description")) / "urdf/yam_gripper.urdf.xacro"
_OPENYAM_PACKAGE_PATHS = {"yam_description": Path(LfsPath("yam_description"))}

ARM_MOTOR_CONFIGS = tuple(
    DamiaoMotorConfig(
        name=f"yam_joint{index}",
        type="DM4340" if index <= 3 else "DM4310",
        send_id=index,
    )
    for index in range(1, 7)
)
GRIPPER_MOTOR_CONFIG = DamiaoMotorConfig("yam_gripper", "DM4310", 0x08, 0x18)
OPENYAM_GRIPPER_CONFIG = DamiaoGripperConfig(
    motor=GRIPPER_MOTOR_CONFIG,
    opening_direction="decreasing_position",
    default_current=0.15,
)


def _active_arm_limits() -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Read arm limits from the active planning Xacro, failing closed."""
    model = parse_model(_OPENYAM_MODEL_PATH, package_paths=_OPENYAM_PACKAGE_PATHS)
    names = [joint.name for joint in model.joints]
    if len(names) != len(set(names)):
        raise ValueError("active OpenYAM Xacro contains duplicate joint names")
    joints = [model.get_joint(f"yam_joint{index}") for index in range(1, 7)]
    resolved = [joint for joint in joints if joint is not None]
    if len(resolved) != 6:
        raise ValueError("active OpenYAM Xacro does not define all six arm joints")
    limits: list[tuple[float, float, float]] = []
    for joint in resolved:
        lower = joint.lower_limit
        upper = joint.upper_limit
        velocity = joint.velocity_limit
        if lower is None or upper is None or velocity is None:
            raise ValueError("active OpenYAM Xacro has incomplete or nonfinite arm limits")
        if not all(math.isfinite(value) for value in (lower, upper, velocity)):
            raise ValueError("active OpenYAM Xacro has incomplete or nonfinite arm limits")
        if lower > upper:
            raise ValueError(f"active OpenYAM Xacro has inverted limits for {joint.name}")
        if velocity <= 0:
            raise ValueError(f"active OpenYAM Xacro has nonpositive velocity for {joint.name}")
        limits.append((lower, upper, velocity))
    return (
        tuple(lower for lower, _, _ in limits),
        tuple(upper for _, upper, _ in limits),
        tuple(velocity for _, _, velocity in limits),
    )


def make_openyam_damiao_arm_config() -> DamiaoArmConfig:
    """Build the canonical OpenYAM arm profile from the active planning model."""

    lower, upper, velocity = _active_arm_limits()
    return DamiaoArmConfig(
        name="openyam",
        vendor="Damiao",
        model="OpenYAM",
        motors=ARM_MOTOR_CONFIGS,
        position_lower=lower,
        position_upper=upper,
        velocity_max=velocity,
        kp=(80.0, 80.0, 80.0, 10.0, 10.0, 10.0),
        kd=(5.0, 5.0, 5.0, 1.5, 1.5, 1.5),
        gripper=OPENYAM_GRIPPER_CONFIG,
    )


class OpenYamDamiaoAdapter(DamiaoArmAdapter):
    """Six-DOF OpenYAM arm with calibrated normalized gripper IO."""

    def __init__(
        self,
        address: str | Path | None = None,
        *,
        runtime_config: DamiaoRuntimeConfig | None = None,
        dof: int | None = None,
        hardware_id: str = "arm",
    ) -> None:
        runtime_config = runtime_config or DamiaoRuntimeConfig()
        if address is not None:
            runtime_config = attrs.evolve(runtime_config, address=str(address))
        if runtime_config.gravity_comp and (
            runtime_config.gravity_model_path is None
            or not runtime_config.gravity_model_path.is_file()
        ):
            raise ValueError("OpenYAM gravity compensation requires a valid model path")
        super().__init__(
            arm_config=make_openyam_damiao_arm_config(),
            runtime_config=runtime_config,
            dof=dof,
            hardware_id=hardware_id,
        )

        self._write_armed_by_read = False

    def refresh_state(self, *, force: bool = False) -> tuple[list[float], list[float], list[float]]:
        """Read feedback and arm exactly one subsequent motor write."""
        self._write_armed_by_read = False
        state = super().refresh_state(force=force)
        self._write_armed_by_read = True
        return state

    def write_mit_commands(
        self,
        *,
        q: list[float],
        dq: list[float],
        kp: list[float],
        kd: list[float],
        tau: list[float],
    ) -> bool:
        """Forward a command only after a successful feedback read."""
        self._validate_command_lengths(q=q, dq=dq, kp=kp, kd=kd, tau=tau)
        if not self._write_armed_by_read:
            logger.error("OpenYAM rejected motor write without fresh position feedback")
            return False
        self._write_armed_by_read = False
        return super().write_mit_commands(q=q, dq=dq, kp=kp, kd=kd, tau=tau)

    def read_gripper_position(self) -> float | None:
        """Read normalized gripper opening, where zero is closed and one is open."""
        if self._runtime is None:
            return None
        return self._runtime.read_gripper_opening()

    def write_gripper_position(self, position: float) -> bool:
        """Command normalized gripper opening, where zero is closed and one is open."""
        if self._runtime is None:
            return False
        return self._runtime.write_gripper_opening(position)


OpenYAMDamiaoAdapter = OpenYamDamiaoAdapter
