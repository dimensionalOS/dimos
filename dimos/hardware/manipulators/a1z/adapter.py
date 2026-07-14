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

"""Galaxea A1Z ManipulatorAdapter — wraps the open-source ``a1z`` CAN SDK.

The A1Z SDK (github.com/userguide-galaxea/GALAXEA-A1Z) is a pure-Python
library that talks classic CAN (1 Mbps, MIT force-position protocol, motor
IDs 0x01-0x06) directly to the arm's six motors. Its ``ArmRobot`` runs an
internal control thread (~250 Hz) that does PD tracking + gravity
compensation of the commanded joint positions, with built-in temperature /
stale-feedback / command-flood emergency stops.

This adapter is a thin bridge to that library:

* ``connect()``      -> ``get_a1z_robot(can_channel=address)`` (opens the bus)
* ``write_enable``   -> ``ArmRobot.start()`` / ``ArmRobot.stop()`` (motors on/off)
* ``read_joint_*``   -> ``ArmRobot.get_joint_state()`` (already SI: rad, rad/s, Nm)
* ``write_joint_positions`` -> ``ArmRobot.command_joint_pos()`` (streamed target)
* ``write_stop``     -> ``ArmRobot.estop()`` (soft e-stop, gravity comp still holds)

All values crossing this boundary are SI (the SDK is natively radians / Nm),
so no unit conversion is needed. The SDK is an optional dependency imported
lazily inside ``connect()`` so a missing install fails loudly there rather
than at module import.
"""

from __future__ import annotations

from typing import Any

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DOF = 6

# A1Z joint limits (rad), from the SDK's get_robot.py / A1Z_Flange URDF.
_POS_LOWER = [-2.094, 0.0, -3.142, -1.309, -1.484, -2.007]
_POS_UPPER = [2.094, 3.142, 0.0, 1.309, 1.484, 2.007]
# MotorA (j1-3) vel max 18 rad/s, MotorB (j4-6) 30 rad/s; use conservative caps.
_VEL_MAX = [18.0, 18.0, 18.0, 10.0, 30.0, 30.0]


class A1ZAdapter:
    """Galaxea A1Z hardware adapter (implements the ManipulatorAdapter Protocol).

    Position-hold mode by default (``zero_gravity_mode=False``): the SDK holds
    the commanded joint positions with PD + gravity compensation, which is what
    the dimos ControlCoordinator expects when it streams position targets. Set
    ``zero_gravity=True`` for a floating, hand-guidable arm (e.g. teach mode),
    but note that commanded positions are not tracked in that mode.
    """

    def __init__(
        self,
        address: str = "can0",
        dof: int = _DOF,
        *,
        zero_gravity: bool = False,
        gravity_comp_factor: float = 1.0,
        control_freq_hz: int = 250,
        initial_positions: list[float] | None = None,
        **_: object,
    ) -> None:
        if dof != _DOF:
            raise ValueError(f"A1ZAdapter only supports {_DOF} DOF (got {dof})")
        self._can_channel = address or "can0"
        self._dof = dof
        self._zero_gravity = zero_gravity
        self._gravity_comp_factor = gravity_comp_factor
        self._control_freq_hz = control_freq_hz
        # initial_positions is accepted for interface parity (mock uses it); the
        # A1Z reads real absolute encoder state on start(), so it is not applied.
        self._robot: Any = None
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.SERVO_POSITION

    def connect(self) -> bool:
        """Open the CAN bus and construct the A1Z robot (motors stay disabled)."""
        try:
            from a1z.robots.get_robot import get_a1z_robot
        except ImportError:
            logger.error(
                "a1z SDK not installed. Install from "
                "github.com/userguide-galaxea/GALAXEA-A1Z (pip install -e .)"
            )
            return False
        try:
            self._robot = get_a1z_robot(
                can_channel=self._can_channel,
                gravity_comp_factor=self._gravity_comp_factor,
                zero_gravity_mode=self._zero_gravity,
                control_freq_hz=self._control_freq_hz,
            )
            self._connected = True
            logger.info(f"A1Z connected on CAN channel {self._can_channel}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to A1Z on {self._can_channel}: {e}")
            self._robot = None
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Disable motors and release the bus."""
        if self._robot is not None:
            try:
                if self._enabled:
                    self._robot.stop()
            except Exception:
                pass
            finally:
                self._enabled = False
                self._connected = False
                self._robot = None

    def is_connected(self) -> bool:
        return self._connected and self._robot is not None

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        return self.write_enable(False)

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(vendor="Galaxea", model="A1Z", dof=self._dof)

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=list(_POS_LOWER),
            position_upper=list(_POS_UPPER),
            velocity_max=list(_VEL_MAX),
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        # The A1Z is a streamed position servo; POSITION / SERVO_POSITION map to
        # the same command path. Other modes are not exposed by this adapter.
        if mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            self._control_mode = mode
            return True
        return False

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        if self._robot is None:
            raise RuntimeError("A1Z not connected")
        return [float(x) for x in self._robot.get_joint_state()["pos"][: self._dof]]

    def read_joint_velocities(self) -> list[float]:
        if self._robot is None:
            return [0.0] * self._dof
        return [float(x) for x in self._robot.get_joint_state()["vel"][: self._dof]]

    def read_joint_efforts(self) -> list[float]:
        if self._robot is None:
            return [0.0] * self._dof
        return [float(x) for x in self._robot.get_joint_state()["eff"][: self._dof]]

    def read_state(self) -> dict[str, int]:
        return {"state": 2 if self._in_error() else 0, "mode": 0}

    def _in_error(self) -> bool:
        if self._robot is None:
            return False
        try:
            codes = self._robot.get_joint_state().get("error_codes")
            return codes is not None and any(int(c) != 0 for c in codes)
        except Exception:
            return False

    def read_error(self) -> tuple[int, str]:
        if self._robot is None:
            return 0, ""
        try:
            codes = self._robot.get_joint_state().get("error_codes")
            if codes is not None:
                for i, c in enumerate(codes):
                    if int(c) != 0:
                        return int(c), f"A1Z joint{i + 1} error {int(c)}"
        except Exception:
            pass
        return 0, ""

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        # ``velocity`` is accepted for Protocol parity but not applied: the SDK's
        # command_joint_pos is a fixed-gain position servo with no per-command
        # speed. Speed is governed by the streamed target rate and the PD gains.
        if self._robot is None or not self._enabled or len(positions) != self._dof:
            return False
        try:
            import numpy as np

            self._robot.command_joint_pos(np.asarray(positions, dtype=float))
            return True
        except Exception as e:
            logger.error(f"A1Z write_joint_positions failed: {e}")
            return False

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        # Not exposed by the A1Z SDK's high-level API (it is a position servo).
        return False

    def write_stop(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.estop()
            return True
        except Exception as e:
            logger.error(f"A1Z estop failed: {e}")
            return False

    def write_enable(self, enable: bool) -> bool:
        if self._robot is None:
            return False
        try:
            if enable and not self._enabled:
                self._robot.start()
                self._enabled = True
            elif not enable and self._enabled:
                self._robot.stop()
                self._enabled = False
            return True
        except Exception as e:
            logger.error(f"A1Z write_enable({enable}) failed: {e}")
            return False

    def read_enabled(self) -> bool:
        return self._enabled

    def write_clear_errors(self) -> bool:
        # The SDK re-arms via release() after a soft estop.
        if self._robot is None:
            return False
        try:
            release = getattr(self._robot, "release", None)
            if callable(release):
                release()
                return True
        except Exception:
            pass
        return False

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_gripper_position(self) -> float | None:
        return None  # A1Z_Flange has no gripper

    def write_gripper_position(self, position: float) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None
