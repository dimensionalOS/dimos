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

"""OpenYAM (Anvil Robotics) ManipulatorAdapter — Damiao MIT mode. SI units.

See ``driver.py`` for the hardware assumptions baked into the defaults and
how to override them; run the probe script before first enable.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

from dimos.hardware.manipulators.openyam.driver import (
    CANABLE_PRODUCT_ID,
    CANABLE_VENDOR_ID,
    CTRL_MODE_MIT,
    DEFAULT_BITRATE,
    DEFAULT_GRIPPER_MOTOR_TYPE,
    DEFAULT_GRIPPER_SEND_ID,
    DamiaoMotor,
    MotorType,
    YamBus,
    gs_usb_mac_bus_factory,
    make_yam_motors,
    resolve_transport,
)
from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)
from dimos.utils.data import LfsPath
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _socketcan_iface_up(name: str) -> bool:
    try:
        flags_path = Path("/sys/class/net") / name / "flags"
        if not flags_path.exists():
            return False
        return (int(flags_path.read_text().strip(), 16) & 0x1) == 0x1
    except OSError:
        return False


# Default MIT gains per joint for POSITION mode (kp ∈ [0,500], kd ∈ [0,5]).
# Conservative starting point for an unverified arm — raise once tracking is
# confirmed sluggish rather than the other way around.
_DEFAULT_KP = [80.0, 80.0, 80.0, 50.0, 50.0, 50.0]
_DEFAULT_KD = [1.2, 1.2, 1.2, 0.8, 0.8, 0.8]
_GRIPPER_KP = 20.0
_GRIPPER_KD = 0.5

# Joint limits from yam_description/urdf/yam_gripper_gravity.urdf (the same
# model the canonical Linux adapter validates against). Joints 2 and 3 are
# asymmetric — zero at one hard stop.
_DEFAULT_POS_LOWER = [-3.92699, 0.0, 0.0, -1.65806, -1.5708, -2.35619]
_DEFAULT_POS_UPPER = [1.5708, 3.66519, 4.01426, 1.65806, 1.5708, 1.8326]
_DEFAULT_VEL_MAX = [3.0, 10.0, 3.0, 10.0, 3.0, 10.0]

# Pinocchio gravity model (mass data from the canonical PR's URDF; lazy LFS)
_GRAVITY_URDF = LfsPath("yam_description/urdf/yam_gripper_gravity.urdf")

_STATE_MAX_AGE_S = 0.1
# Keepalive fires when nothing was transmitted for this long; well inside
# the freshness window so idle reads never go stale.
_KEEPALIVE_AFTER_S = 0.04


class OpenYamAdapter:
    """6-DOF OpenYAM + CAN gripper on one bus.

    ``address`` accepts a SocketCAN name (``can0``), a ``gs_usb[:N]``
    selector, or a serial device path for SLCAN — see
    ``driver.resolve_transport`` for how each maps to a python-can backend.
    """

    def __init__(
        self,
        address: str = "can0",
        dof: int = 6,
        *,
        interface: str | None = None,
        bitrate: int = DEFAULT_BITRATE,
        fd: bool = False,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        arm_motor_types: list[str] | None = None,
        motor_ids: list[int] | None = None,
        recv_id_offset: int = 0x10,
        gripper_motor_id: int | None = DEFAULT_GRIPPER_SEND_ID,
        gripper_motor_type: str = DEFAULT_GRIPPER_MOTOR_TYPE.value,
        position_lower: list[float] | None = None,
        position_upper: list[float] | None = None,
        auto_set_mit_mode: bool = False,
        gravity_comp: bool = True,
        usb_vendor_id: int = CANABLE_VENDOR_ID,
        usb_product_id: int = CANABLE_PRODUCT_ID,
        **_: Any,
    ) -> None:
        if dof != 6:
            raise ValueError(f"OpenYamAdapter only supports 6 DOF (got {dof})")
        self._address = address
        self._dof = dof
        self._fd = fd
        resolved_interface, channel, bus_kwargs = resolve_transport(
            address, interface=interface, bitrate=bitrate
        )
        self._interface = resolved_interface
        self._channel = channel
        self._bus_kwargs = bus_kwargs
        # gs_usb goes through the shared libusb bus (quirk-handling, macOS
        # validated) instead of python-can's stock gs_usb backend.
        self._bus_factory = (
            gs_usb_mac_bus_factory(
                bitrate=bitrate,
                vendor_id=usb_vendor_id,
                product_id=usb_product_id,
            )
            if resolved_interface == "gs_usb"
            else None
        )
        self._kp = list(kp) if kp is not None else list(_DEFAULT_KP)
        self._kd = list(kd) if kd is not None else list(_DEFAULT_KD)
        if len(self._kp) != dof or len(self._kd) != dof:
            raise ValueError(f"kp/kd must be length {dof}")
        self._pos_lower = list(position_lower) if position_lower else list(_DEFAULT_POS_LOWER)
        self._pos_upper = list(position_upper) if position_upper else list(_DEFAULT_POS_UPPER)
        self._auto_set_mit_mode = auto_set_mit_mode
        self._gravity_comp = gravity_comp
        # Pinocchio model, loaded lazily in connect()
        self._pin_model: Any = None
        self._pin_data: Any = None

        types = [MotorType(t) for t in arm_motor_types] if arm_motor_types is not None else None
        self._motors = make_yam_motors(
            dof,
            arm_motor_types=types,
            motor_ids=motor_ids,
            recv_id_offset=recv_id_offset,
        )
        self._gripper_motor: DamiaoMotor | None = None
        if gripper_motor_id is not None:
            self._gripper_motor = DamiaoMotor(
                gripper_motor_id,
                MotorType(gripper_motor_type),
                recv_id=gripper_motor_id | recv_id_offset,
            )

        self._bus: YamBus | None = None
        self._control_mode: ControlMode = ControlMode.POSITION
        self._enabled: bool = False
        # Last successful position command — anchor for VELOCITY mode
        self._last_cmd_q: list[float] | None = None
        self._last_gripper_cmd: float | None = None
        # Keepalive: Damiao motors only report state in reply to command
        # frames, so an idle host (e.g. a coordinator with no active task,
        # which writes nothing) starves its own state reads forever. When
        # nothing has been transmitted for _KEEPALIVE_AFTER_S, re-send the
        # hold command (enabled) or an idempotent disable pulse (disabled).
        self._last_tx = 0.0
        self._keepalive_stop = threading.Event()
        self._keepalive_thread: threading.Thread | None = None

    # ------------------------------------------------------------------ setup

    def connect(self) -> bool:
        # Preflight only applies to SocketCAN, where bringing the interface up
        # needs root and so can't be done here.
        if self._interface == "socketcan" and not _socketcan_iface_up(self._channel):
            logger.error(
                f"SocketCAN interface '{self._channel}' is not UP. "
                f"Run: sudo ./dimos/robot/manipulators/openyam/scripts/openyam_can_up.sh {self._channel}"
            )
            return False

        all_motors = list(self._motors)
        if self._gripper_motor is not None:
            all_motors.append(self._gripper_motor)
        try:
            self._bus = YamBus(
                channel=self._channel,
                motors=all_motors,
                fd=self._fd,
                interface=self._interface,
                bus_kwargs=self._bus_kwargs,
                bus_factory=self._bus_factory,
            )
            self._bus.open()
        except Exception as e:
            logger.error(f"OpenYAM @{self._address} ({self._interface}) connect failed: {e}")
            if self._interface == "gs_usb":
                logger.error(
                    "gs_usb needs the 'gs_usb' pip package and libusb "
                    "(macOS: brew install libusb). If the dongle runs serial "
                    "firmware instead of candlelight, pass its /dev/tty* path "
                    "as address to use slcan."
                )
            self._bus = None
            return False

        # Off by default: YAM motors ship in MIT mode, and the OpenArm-style
        # CTRL_MODE register broadcast is suspected of faulting their
        # firmware (first physical motion succeeded only with this skipped).
        if self._auto_set_mit_mode:
            try:
                for m in all_motors:
                    self._bus.write_ctrl_mode(m.send_id, CTRL_MODE_MIT)
            except Exception as e:
                logger.error(f"failed to set MIT mode on {self._address}: {e}")
                self._bus.close()
                self._bus = None
                return False

        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, name="openyam-keepalive", daemon=True
        )
        self._keepalive_thread.start()

        if self._gravity_comp:
            try:
                import pinocchio

                self._pin_model = pinocchio.buildModelFromUrdf(str(_GRAVITY_URDF))
                self._pin_data = self._pin_model.createData()
                if self._pin_model.nq != self._dof:
                    raise ValueError(f"gravity model nq={self._pin_model.nq} != dof={self._dof}")
                logger.info(f"OpenYAM gravity compensation enabled (nq={self._pin_model.nq})")
            except Exception as e:
                logger.warning(f"gravity comp disabled — {e}")
                self._pin_model = None
                self._pin_data = None

        return True

    def _keepalive_loop(self) -> None:
        while not self._keepalive_stop.is_set():
            time.sleep(_KEEPALIVE_AFTER_S / 2)
            bus = self._bus
            if bus is None:
                continue
            if time.monotonic() - self._last_tx < _KEEPALIVE_AFTER_S:
                continue
            try:
                if self._enabled and self._last_cmd_q is not None:
                    self.write_joint_positions(self._last_cmd_q)
                elif self._enabled:
                    # Enabled but never commanded: hold at measured or, if
                    # state is unknown, elicit it with an idempotent enable.
                    self.write_stop() or self.write_enable(True)
                else:
                    # Idempotent while disabled; every motor replies state.
                    bus.disable_all()
                    self._last_tx = time.monotonic()
            except Exception:
                pass  # transient bus hiccup; next cycle retries

    def disconnect(self) -> None:
        self._keepalive_stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=1.0)
            self._keepalive_thread = None
        if self._bus is None:
            return
        try:
            self._bus.disable_all()
        except Exception:
            pass
        self._enabled = False
        self._bus.close()
        self._bus = None

    def is_connected(self) -> bool:
        return self._bus is not None

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        stopped = self.write_stop()
        disabled = self.write_enable(False)
        return stopped and disabled

    # ------------------------------------------------------------------- info

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Anvil Robotics",
            model="OpenYAM",
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=list(self._pos_lower),
            position_upper=list(self._pos_upper),
            velocity_max=list(_DEFAULT_VEL_MAX),
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        # Runs exclusively in Damiao MIT register mode; dimos ControlModes are
        # emulated by tuning kp/kd/q/dq/tau per MIT frame.
        if mode in (
            ControlMode.POSITION,
            ControlMode.SERVO_POSITION,
            ControlMode.VELOCITY,
            ControlMode.TORQUE,
        ):
            self._control_mode = mode
            return True
        return False

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    # ------------------------------------------------------------------- read

    def _states_or_raise(self) -> list[Any]:
        # Raises on missing/stale data so hardware_interface.py can retry
        # (init) or skip the tick (steady-state).
        if self._bus is None:
            raise RuntimeError("OpenYamAdapter not connected")
        now = time.monotonic()
        states = [self._bus.get_state(m.send_id) for m in self._motors]
        for i, s in enumerate(states):
            if s is None:
                raise RuntimeError(f"motor {i + 1} has no state yet")
            if now - s.timestamp > _STATE_MAX_AGE_S:
                age_ms = (now - s.timestamp) * 1000
                raise RuntimeError(f"motor {i + 1} state stale ({age_ms:.0f} ms)")
        return states

    def read_joint_positions(self) -> list[float]:
        return [s.q for s in self._states_or_raise()]

    def read_joint_velocities(self) -> list[float]:
        return [s.dq for s in self._states_or_raise()]

    def read_joint_efforts(self) -> list[float]:
        return [s.tau for s in self._states_or_raise()]

    def read_state(self) -> dict[str, int]:
        if self._bus is None:
            return {"state": 0, "mode": 0}
        states = [self._bus.get_state(m.send_id) for m in self._motors]
        t_rotor = max((s.t_rotor for s in states if s is not None), default=0)
        return {
            "state": 1 if self._enabled else 0,
            "mode": 1,  # MIT
            "t_rotor_max": int(t_rotor),
        }

    def read_error(self) -> tuple[int, str]:
        # Damiao state frames carry no structured error code; surface a soft
        # thermal warning from the temperature fields.
        if self._bus is None:
            return 0, ""
        states = [self._bus.get_state(m.send_id) for m in self._motors]
        t_rotor = max((s.t_rotor for s in states if s is not None), default=0)
        if t_rotor >= 85:
            return 1, f"rotor over-temperature ({t_rotor}°C)"
        return 0, ""

    def read_enabled(self) -> bool:
        return self._enabled

    # ------------------------------------------------------------------ write

    def _gravity_torques(self) -> list[float]:
        # Pinocchio G(q) at the measured pose, clamped to motor torque
        # limits. Zeros when the model is unavailable or state is stale —
        # PD-only control still works, just with steady-state sag.
        if self._pin_model is None or self._pin_data is None:
            return [0.0] * self._dof
        import pinocchio

        try:
            q = np.array(self.read_joint_positions(), dtype=np.float64)
        except RuntimeError:
            return [0.0] * self._dof
        tau_g = pinocchio.computeGeneralizedGravity(self._pin_model, self._pin_data, q)
        return [
            float(np.clip(tau_g[i], -m.limits[2], m.limits[2])) for i, m in enumerate(self._motors)
        ]

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        if self._bus is None or not self._enabled:
            return False
        if len(positions) != self._dof:
            return False
        velocity = max(0.0, min(1.0, velocity))
        tau_ff = self._gravity_torques()
        for motor, q, kp, kd, tau in zip(
            self._motors, positions, self._kp, self._kd, tau_ff, strict=False
        ):
            self._bus.send_mit_one(motor.send_id, (q, 0.0, kp * velocity, kd, tau))
        self._last_cmd_q = list(positions)
        self._last_tx = time.monotonic()
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        # MIT velocity tracking: kp=0, dq direct, q anchored at the last
        # commanded position so the motor doesn't drift.
        if self._bus is None or not self._enabled:
            return False
        if len(velocities) != self._dof:
            return False
        if self._last_cmd_q is None:
            try:
                self._last_cmd_q = self.read_joint_positions()
            except RuntimeError:
                return False
        anchor = self._last_cmd_q
        tau_ff = self._gravity_torques()
        for motor, q_anchor, dq, kd, tau in zip(
            self._motors, anchor, velocities, self._kd, tau_ff, strict=False
        ):
            self._bus.send_mit_one(motor.send_id, (q_anchor, dq, 0.0, kd, tau))
        self._last_tx = time.monotonic()
        return True

    def write_stop(self) -> bool:
        if self._bus is None:
            return False
        # Without current positions we can't safely command "hold here".
        try:
            q_now = self.read_joint_positions()
        except RuntimeError:
            return False
        tau_ff = self._gravity_torques()
        for motor, q, kp, kd, tau in zip(
            self._motors, q_now, self._kp, self._kd, tau_ff, strict=False
        ):
            self._bus.send_mit_one(motor.send_id, (q, 0.0, kp, kd, tau))
        self._last_cmd_q = q_now
        self._last_tx = time.monotonic()
        return True

    def write_enable(self, enable: bool) -> bool:
        if self._bus is None:
            return False
        self._enabled = False
        try:
            if enable:
                self._bus.enable_all()
            else:
                self._bus.disable_all()
        except Exception:
            return False
        self._enabled = enable
        self._last_tx = time.monotonic()
        return True

    def write_clear_errors(self) -> bool:
        # No separate clear-error command; disable/enable is the recovery path.
        if self._bus is None:
            return False
        self._enabled = False
        try:
            self._bus.disable_all()
            self._bus.enable_all()
        except Exception:
            return False
        self._enabled = True
        self._last_tx = time.monotonic()
        return True

    # -------------------------------------------------------------- cartesian

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    # ---------------------------------------------------------------- gripper

    def read_gripper_position(self) -> float | None:
        """Gripper motor position in rad (adapter-native units)."""
        if self._bus is None or self._gripper_motor is None:
            return None
        state = self._bus.get_state(self._gripper_motor.send_id)
        return None if state is None else state.q

    def write_gripper_position(self, position: float) -> bool:
        """Command the gripper motor to ``position`` rad via an MIT frame."""
        if self._bus is None or not self._enabled or self._gripper_motor is None:
            return False
        self._bus.send_mit_one(
            self._gripper_motor.send_id,
            (position, 0.0, _GRIPPER_KP, _GRIPPER_KD, 0.0),
        )
        self._last_gripper_cmd = position
        self._last_tx = time.monotonic()
        return True

    def read_force_torque(self) -> list[float] | None:
        return None
