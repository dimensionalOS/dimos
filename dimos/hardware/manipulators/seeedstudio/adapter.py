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

"""Seeed Studio reBot B601-DM adapter - implements ManipulatorAdapter protocol.

Transport: HDSC USB serial CAN bridge to seven Damiao motors (see protocol.py).
Units: arm joints in radians; the trailing gripper entry is jaw opening in
metres (0.0 closed), converted to and from gripper motor radians here.

Lifecycle mapping:
- connect(): open the serial port and read every motor's parameters. Motors
  stay unpowered and nothing is written.
- activate() / write_enable(True): check every motor is in POS_VEL mode and
  the measured pose is within limits, seed POS_VEL targets at that pose, then
  enable and wait for every motor to confirm.
- write_stop(): hold the freshly measured pose.
- deactivate() / write_enable(False): disable the motors this adapter enabled.

Faults are handled by kind. A motor that reports a fault or drops out of the
enabled state disables every motor. Lost communication (repeated missed
replies, failed writes, a closed port) does not: the motors keep holding
their last POS_VEL target and the adapter refuses further commands until it
is deactivated and activated again.

The adapter never changes firmware mode, gains or zero. POS_VEL mode has no
torque limit of its own; closing the gripper stops advancing once the motor
effort shows it has met resistance.

SAFETY: the arm has no brakes. Disabling motors (deactivate, disconnect, a
motor fault) lets the arm fall. Support or rest it first. With communication
lost, only the 24 V supply removes holding torque.
"""

from __future__ import annotations

from dataclasses import replace
from enum import Enum
import math
import struct
import threading
import time
from typing import Any

from dimos.hardware.manipulators.seeedstudio.protocol import (
    DISABLE,
    ENABLE,
    FEEDBACK_IDS,
    MOTOR_IDS,
    POS_VEL_MODE,
    POS_VEL_OFFSET,
    STATUS_NAMES,
    STREAM_PACKET_INTERVAL,
    DmSerialTransport,
    Feedback,
    MotorParameters,
)
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.spec import JointLimits
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ARM_DOF = 6
B601_DOF = ARM_DOF + 1

# Vendor URDF limits (reBotArm_control_py urdf/DM/urdf/ReBot_Arm_DM.urdf),
# except joint2's upper bound: at its folded mechanical rest this arm's joint2
# encoder reads about +0.0055 rad, just past the URDF's 0.0.
JOINT2_REST_MARGIN = 0.02
ARM_LOWER = (-2.8, -3.14, -3.14, -1.87, -1.57, -3.14)
ARM_UPPER = (2.8, JOINT2_REST_MARGIN, 0.0, 1.57, 1.57, 3.14)
# POS_VEL velocity ceiling; streamed trajectory targets set the actual speed.
ARM_VELOCITY_MAX = 1.0

# Gripper motor stops measured by hand: -0.15 rad closed, -5.93 rad open (more
# negative is more open). Commanded travel is the vendor grasp driver's 4.9 rad
# open angle (reBot-DevArm-Grasp drivers/robot/grasp_driver.py), starting
# 0.1 rad short of the closed stop. Jaw opening follows the vendor URDF: each
# finger travels 0 to 28.5 mm.
GRIPPER_STOP_CLOSED_RAD = -0.15
GRIPPER_STOP_OPEN_RAD = -5.93
# After power-up the motor reports its angle within one turn, and this travel
# crosses the +/-pi wrap. Readings are shifted by whole turns into the travel,
# with the remaining slack split evenly beyond both stops.
_GRIPPER_WRAP_SLACK_RAD = (2 * math.pi - (GRIPPER_STOP_CLOSED_RAD - GRIPPER_STOP_OPEN_RAD)) / 2
GRIPPER_WRAP_UPPER_RAD = GRIPPER_STOP_CLOSED_RAD + _GRIPPER_WRAP_SLACK_RAD
GRIPPER_CLOSED_RAD = GRIPPER_STOP_CLOSED_RAD - 0.1
GRIPPER_OPEN_RAD = GRIPPER_CLOSED_RAD - 4.9
GRIPPER_MAX_OPENING_M = 2 * 0.0285
GRIPPER_RAD_PER_M = (GRIPPER_CLOSED_RAD - GRIPPER_OPEN_RAD) / GRIPPER_MAX_OPENING_M
GRIPPER_VELOCITY_RAD_S = 2.0
# Closing is held at the measured position once effort exceeds this.
GRIPPER_STALL_EFFORT_NM = 1.5
_GRIPPER_CLOSING_DEADBAND_RAD = 0.05

READ_CACHE_SECONDS = 0.02
ENABLE_SETTLE_SECONDS = 0.1
ENABLE_CONFIRM_TIMEOUT = 0.5
_SEED_VELOCITY_RAD_S = 0.1
# The bridge occasionally drops a single reply; only repeated misses count.
FEEDBACK_ATTEMPTS = 3


class _Output(Enum):
    """What the motors are doing, as far as this adapter knows."""

    DISABLED = "disabled"
    ENABLED = "enabled"
    # Communication lost while enabled: motors hold their last target.
    HELD = "held"


def gripper_turn_offset(reported_rad: float) -> float:
    """Whole turns to subtract from a reported gripper angle to land in its travel."""
    turns = math.ceil((reported_rad - GRIPPER_WRAP_UPPER_RAD) / (2 * math.pi))
    return turns * 2 * math.pi


def gripper_opening_to_motor(opening_m: float) -> float:
    """Jaw opening in metres to gripper motor radians, clamped to travel."""
    opening = min(GRIPPER_MAX_OPENING_M, max(0.0, opening_m))
    return GRIPPER_CLOSED_RAD - opening * GRIPPER_RAD_PER_M


def gripper_motor_to_opening(position_rad: float) -> float:
    """Gripper motor radians to jaw opening in metres, clamped to travel."""
    opening = (GRIPPER_CLOSED_RAD - position_rad) / GRIPPER_RAD_PER_M
    return min(GRIPPER_MAX_OPENING_M, max(0.0, opening))


class SeeedStudioAdapter:
    """Seeed Studio reBot B601-DM: six arm joints plus a gripper.

    Implements ManipulatorAdapter protocol via duck typing. Position commands
    use each motor's POS_VEL mode. Velocity, torque and Cartesian commands
    are not supported.
    """

    def __init__(
        self,
        address: str,
        dof: int = B601_DOF,
        *,
        timeout: float = 0.15,
        max_feedback_age: float = 0.25,
        **_: Any,
    ) -> None:
        if dof != B601_DOF:
            raise ValueError(f"B601-DM owns six arm joints plus a gripper (dof={B601_DOF})")
        if not math.isfinite(max_feedback_age) or max_feedback_age <= 0:
            raise ValueError("max_feedback_age must be finite and positive")
        self._bus = DmSerialTransport(address, timeout)
        self._max_feedback_age = max_feedback_age
        self._lock = threading.RLock()
        self._parameters: tuple[MotorParameters, ...] = ()
        self._feedback: tuple[Feedback, ...] = ()
        self._snapshot_at = 0.0
        self._control_mode = ControlMode.POSITION
        self._output = _Output.DISABLED
        self._gripper_hold: float | None = None
        # Reported minus physical gripper angle, fixed per connection.
        self._gripper_offset = 0.0
        self._error = ""

    def connect(self) -> bool:
        """Open the serial port and read all seven motors. Nothing is written."""
        with self._lock:
            if self.is_connected():
                return True
            self._error = ""
            try:
                self._bus.open()
                self._parameters = tuple(
                    self._bus.parameters(mid, fid)
                    for mid, fid in zip(MOTOR_IDS, FEEDBACK_IDS, strict=True)
                )
                self._gripper_offset = 0.0
                reported = self._snapshot(force=True)[ARM_DOF].position
                self._gripper_offset = gripper_turn_offset(reported)
                if self._gripper_offset:
                    logger.info(
                        "B601-DM gripper angle unwrapped by whole turns",
                        reported_rad=round(reported, 4),
                        offset_rad=round(self._gripper_offset, 4),
                    )
                self._snapshot(force=True)
            except (OSError, ValueError) as exc:
                self._error = str(exc)
                self._parameters = ()
                self._feedback = ()
                self._bus.close()
                logger.error("B601-DM connection failed", error=self._error)
                return False
            return True

    def disconnect(self) -> None:
        """Disable motors this adapter enabled, then close the port.

        SAFETY: the arm has no brakes and will fall when motors disable.
        """
        with self._lock:
            try:
                if self._output is not _Output.DISABLED and not self.write_enable(False):
                    logger.error("B601-DM shutdown could not confirm all motors disabled")
            finally:
                self._bus.close()
                self._parameters = ()
                self._feedback = ()
                self._output = _Output.DISABLED

    def is_connected(self) -> bool:
        with self._lock:
            return self._bus.is_open() and len(self._parameters) == B601_DOF

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        """Disable motors. SAFETY: the arm has no brakes; rest it first."""
        return self.write_enable(False)

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(vendor="Seeed Studio", model="reBot B601-DM", dof=B601_DOF)

    def get_dof(self) -> int:
        return B601_DOF

    def get_limits(self) -> JointLimits:
        """Arm limits in radians, then the gripper's jaw opening in metres."""
        return JointLimits(
            position_lower=[*ARM_LOWER, 0.0],
            position_upper=[*ARM_UPPER, GRIPPER_MAX_OPENING_M],
            velocity_max=[ARM_VELOCITY_MAX] * ARM_DOF + [0.0],
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        """POSITION and SERVO_POSITION both stream POS_VEL targets."""
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        with self._lock:
            self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        with self._lock:
            return self._control_mode

    def read_joint_positions(self) -> list[float]:
        """Arm positions in radians, then the gripper opening in metres."""
        with self._lock:
            feedback = self._snapshot()
            return [f.position for f in feedback[:ARM_DOF]] + [
                gripper_motor_to_opening(feedback[ARM_DOF].position)
            ]

    def read_joint_velocities(self) -> list[float]:
        """Arm velocities in rad/s, then the gripper opening rate in m/s."""
        with self._lock:
            feedback = self._snapshot()
            return [f.velocity for f in feedback[:ARM_DOF]] + [
                -feedback[ARM_DOF].velocity / GRIPPER_RAD_PER_M
            ]

    def read_joint_efforts(self) -> list[float]:
        """Motor efforts in Nm, gripper motor last."""
        with self._lock:
            return [f.effort for f in self._snapshot()]

    def read_state(self) -> dict[str, int]:
        """Read robot state (0=idle, 1=enabled, 2=error)."""
        with self._lock:
            error_code, _ = self.read_error()
            if not self.is_connected():
                return {"state": 0, "mode": POS_VEL_MODE, "error_code": error_code}
            feedback = self._feedback or self._snapshot()
            return {
                "state": 2 if error_code else int(self._output is _Output.ENABLED),
                "mode": POS_VEL_MODE,
                "error_code": error_code,
                "temp_mos_max": max(f.mos_temperature for f in feedback),
                "temp_rotor_max": max(f.rotor_temperature for f in feedback),
            }

    def read_error(self) -> tuple[int, str]:
        """Read error code and message. (0, '') means no error."""
        with self._lock:
            if self._error:
                return -1, self._error
            if not self.is_connected():
                return 0, ""
            try:
                for mid, f in zip(MOTOR_IDS, self._snapshot(), strict=True):
                    if f.status not in (0, 1):
                        return f.status, f"Motor {mid} fault status {f.status:#x}"
            except (OSError, ValueError) as exc:
                return -1, str(exc)
            return 0, ""

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        """Command arm radians and gripper metres.

        Args:
            positions: Six arm targets in radians, then jaw opening in metres
            velocity: Fraction (0-1] of the arm's POS_VEL velocity ceiling
        """
        with self._lock:
            if (
                self._output is not _Output.ENABLED
                or len(positions) != B601_DOF
                or not all(math.isfinite(q) for q in positions)
                or not math.isfinite(velocity)
                or not 0 < velocity <= 1
            ):
                return False
            arm = positions[:ARM_DOF]
            if not all(lo <= q <= hi for q, lo, hi in zip(arm, ARM_LOWER, ARM_UPPER, strict=True)):
                return False
            try:
                feedback = self._snapshot()
                if self._output is not _Output.ENABLED:
                    return False
                gripper = self._gripper_target(
                    gripper_opening_to_motor(positions[ARM_DOF]), feedback[ARM_DOF]
                )
                self._send_positions(
                    [*arm, gripper],
                    [velocity * ARM_VELOCITY_MAX] * ARM_DOF + [GRIPPER_VELOCITY_RAD_S],
                )
                return True
            except (OSError, ValueError) as exc:
                # A failed read has already recorded the more precise cause.
                if self._output is _Output.ENABLED:
                    self._hold(f"Command failed: {exc}")
                return False

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Not supported - motors run in POS_VEL mode."""
        return False

    def write_stop(self) -> bool:
        """Hold the freshly measured pose. This is not a safety-rated E-stop."""
        with self._lock:
            if self._output is not _Output.ENABLED:
                return False
            try:
                feedback = self._snapshot(force=True)
                if self._output is not _Output.ENABLED:
                    return False
                self._send_positions(
                    [f.position for f in feedback],
                    [_SEED_VELOCITY_RAD_S] * B601_DOF,
                )
                return True
            except (OSError, ValueError) as exc:
                # A failed read has already recorded the more precise cause.
                if self._output is _Output.ENABLED:
                    self._hold(f"Command failed: {exc}")
                return False

    def write_enable(self, enable: bool) -> bool:
        """Enable with a measured-pose seed, or disable motors this adapter enabled.

        SAFETY: disabling removes holding torque and the arm falls freely.
        """
        with self._lock:
            if not self.is_connected():
                # Nothing to disable unless motors were left enabled or held.
                return not enable and self._output is _Output.DISABLED
            if not enable:
                # Never disable motors another process enabled.
                if self._output is _Output.DISABLED:
                    return True
                sent = self._disable_all()
                try:
                    disabled = all(f.status == 0 for f in self._snapshot(force=True))
                except (OSError, ValueError):
                    return False
                return sent and disabled
            if self._output is _Output.ENABLED:
                return self.read_enabled()
            if self._output is _Output.HELD:
                self._error = "Communication was lost; deactivate before activating again"
                return False
            if not self._enable():
                logger.error("B601-DM activation failed", reason=self._error)
                return False
            return True

    def read_enabled(self) -> bool:
        with self._lock:
            if self._output is not _Output.ENABLED:
                return False
            try:
                return all(f.status == 1 for f in self._snapshot())
            except (OSError, ValueError):
                return False

    def write_clear_errors(self) -> bool:
        """Not supported - inspect faults with `dimos hardware seeedstudio doctor`."""
        return False

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Not supported - Cartesian state comes from the planning model."""
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        """Not supported - Cartesian targets go through the planning stack."""
        return False

    def read_force_torque(self) -> list[float] | None:
        """Not supported - no F/T sensor (per-motor efforts via read_joint_efforts)."""
        return None

    def _enable(self) -> bool:
        try:
            # Recheck the actual mode on every activation, never rewrite it.
            if any(
                self._bus.read_register(p.motor_id, p.feedback_id, 10) != POS_VEL_MODE
                for p in self._parameters
            ):
                self._error = "All seven motors must already be configured for POS_VEL mode"
                return False
            feedback = self._snapshot(force=True)
            arm = [f.position for f in feedback[:ARM_DOF]]
            outside = [
                f"joint{i + 1}={q:.3f}"
                for i, (q, lo, hi) in enumerate(zip(arm, ARM_LOWER, ARM_UPPER, strict=True))
                if not lo <= q <= hi
            ]
            if outside:
                self._error = f"Start pose outside joint limits: {', '.join(outside)}"
                return False
            if any(f.status != 0 for f in feedback):
                self._error = "Activation requires disabled, fault-free motors"
                return False
            # Seed the measured pose before enabling; never command a zero pose.
            self._send_positions([f.position for f in feedback], [_SEED_VELOCITY_RAD_S] * B601_DOF)
            self._output = _Output.ENABLED
            for mid in MOTOR_IDS:
                self._bus.send(mid, ENABLE)
            time.sleep(ENABLE_SETTLE_SECONDS)
            if not self._confirm_enabled():
                self._fault("One or more motors did not confirm enabling")
                return False
            self._gripper_hold = None
            self._error = ""
            return True
        except (OSError, ValueError) as exc:
            if self._output is not _Output.DISABLED:
                self._fault(f"Activation failed: {exc}")
            else:
                self._error = str(exc)
            return False

    def _confirm_enabled(self) -> bool:
        """Allow delayed enable feedback, but never treat a fault as success."""
        deadline = time.monotonic() + ENABLE_CONFIRM_TIMEOUT
        while True:
            feedback = self._read_feedback()
            if all(f.status == 1 for f in feedback):
                self._store(feedback)
                return True
            if any(f.status not in (0, 1) for f in feedback):
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.02, remaining))

    def _gripper_target(self, target: float, measured: Feedback) -> float:
        """Stop advancing a close once the jaws meet resistance.

        Closing increases motor radians. A stalled close is held at the
        measured position until a more open target is requested.
        """
        if self._gripper_hold is not None:
            if target < self._gripper_hold:
                self._gripper_hold = None
            else:
                return self._gripper_hold
        closing = target > measured.position + _GRIPPER_CLOSING_DEADBAND_RAD
        if closing and abs(measured.effort) > GRIPPER_STALL_EFFORT_NM:
            self._gripper_hold = measured.position
            return measured.position
        return target

    def _send_positions(self, positions: list[float], velocities: list[float]) -> None:
        """Send physical positions; the gripper target is shifted into the motor's frame."""
        positions = [*positions[:ARM_DOF], positions[ARM_DOF] + self._gripper_offset]
        for mid, position, velocity in zip(MOTOR_IDS, positions, velocities, strict=True):
            self._bus.send(
                mid + POS_VEL_OFFSET,
                struct.pack("<ff", position, velocity),
                STREAM_PACKET_INTERVAL,
            )

    def _read_feedback(self) -> tuple[Feedback, ...]:
        """One sweep of all motors, with the gripper in its physical frame."""
        feedback = tuple(self._read_motor(p) for p in self._parameters)
        if any(time.monotonic() - f.received_at > self._max_feedback_age for f in feedback):
            raise TimeoutError("B601-DM feedback expired during the joint sweep")
        gripper = feedback[ARM_DOF]
        return (
            *feedback[:ARM_DOF],
            replace(gripper, position=gripper.position - self._gripper_offset),
        )

    def _read_motor(self, params: MotorParameters) -> Feedback:
        for _ in range(FEEDBACK_ATTEMPTS - 1):
            try:
                return self._bus.feedback(params)
            except TimeoutError:
                logger.warning("B601-DM missed a feedback reply", motor_id=params.motor_id)
        return self._bus.feedback(params)

    def _store(self, feedback: tuple[Feedback, ...]) -> None:
        self._feedback = feedback
        self._snapshot_at = time.monotonic()

    def _snapshot(self, *, force: bool = False) -> tuple[Feedback, ...]:
        if not self.is_connected():
            self._feedback = ()
            if self._output is _Output.ENABLED:
                self._hold("Serial connection lost")
            raise ConnectionError("B601-DM is not connected to all seven motors")
        if (
            not force
            and self._feedback
            and time.monotonic() - self._snapshot_at <= READ_CACHE_SECONDS
        ):
            return self._feedback
        try:
            feedback = self._read_feedback()
        except (OSError, ValueError) as exc:
            self._feedback = ()
            if self._output is _Output.ENABLED:
                self._hold(f"Feedback lost: {exc}")
            else:
                self._error = str(exc)
            raise
        if self._output is _Output.ENABLED and any(f.status != 1 for f in feedback):
            statuses = {
                f"motor_{mid}": STATUS_NAMES.get(f.status, f.status)
                for mid, f in zip(MOTOR_IDS, feedback, strict=True)
                if f.status != 1
            }
            self._fault(f"Motor disabled or faulted during motion: {statuses}")
        self._store(feedback)
        return feedback

    def _hold(self, reason: str) -> None:
        """Stop commanding after lost communication; motors keep their last target."""
        self._error = reason
        self._output = _Output.HELD
        self._gripper_hold = None
        logger.error(
            "B601-DM communication lost; motors hold their last target and commands are refused",
            reason=reason,
        )

    def _fault(self, reason: str) -> None:
        """Record and log why motion stopped, then disable every motor."""
        self._error = reason
        logger.error("B601-DM disabling all motors", reason=reason)
        self._disable_all()

    def _disable_all(self) -> bool:
        """Attempt every motor even if an earlier serial write fails."""
        self._output = _Output.DISABLED
        self._gripper_hold = None
        success = True
        for mid in MOTOR_IDS:
            try:
                self._bus.send(mid, DISABLE)
            except OSError as exc:
                self._error = str(exc)
                logger.error("B601-DM disable failed", motor_id=mid, error=str(exc))
                success = False
        return success
