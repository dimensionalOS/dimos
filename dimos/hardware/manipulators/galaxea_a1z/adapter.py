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

"""Galaxea A1Z adapter - implements ManipulatorAdapter protocol.

SDK Units: angles=radians, velocity=rad/s, torque=Nm (SI throughout - no
conversion needed).

The a1z SDK runs its own 250 Hz control thread (MIT PD + gravity compensation)
with built-in safety watchdogs (joint/velocity/temperature limits, stale
feedback, loop-frequency). This adapter issues commands to that loop rather
than driving CAN directly.

Lifecycle mapping:
- connect(): construct the robot (opens the CAN bus; motors stay unpowered)
- activate() / write_enable(True): enable motors and start the control loop
- write_stop(): latching soft e-stop (pins current position; commands rejected
  until write_clear_errors())
- deactivate() / write_enable(False): stop the control loop and DISABLE motors

SAFETY: the A1 arm has no brakes. Disabling motors (deactivate, write_enable
(False), disconnect) lets the arm fall freely. Lower or support the arm first.

Teaching mode is selected with ``A1ZConfig.teaching``. It uses the vendor's
zero-gravity startup and can optionally place the gripper in free-drive.
"""

from __future__ import annotations

from functools import cached_property
from pathlib import Path
import platform
import threading
import time
from typing import Any, Literal, cast

import a1z
from a1z.robots.arm_robot import ArmRobot
from a1z.robots.get_robot import get_a1z_robot
from a1z.robots.kinematics import Kinematics
import numpy as np

from dimos.hardware.manipulators.galaxea_a1z.config import A1ZConfig
from dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus import gs_usb_can_bus
from dimos.hardware.manipulators.spec import (
    ControlMode,
    ManipulatorInfo,
)
from dimos.hardware.spec import JointLimits

# Joint limits from a1z/robots/get_robot.py (_JOINT_LIMITS)
_POSITION_LOWER = [-2.094, 0.0, -3.142, -1.484, -1.484, -2.007]
_POSITION_UPPER = [2.094, 3.142, 0.0, 1.484, 1.484, 2.007]
# Per-joint velocity caps from ArmRobot defaults (~70% of motor hardware max)
_VELOCITY_MAX = [12.0, 12.0, 12.0, 7.0, 20.0, 20.0]

# Max average speed for planned moves. move_joints uses minimum-jerk
# interpolation with peak velocity 1.875x average and rejects speeds whose
# peak exceeds the SDK's 4.0 rad/s streaming cap, so keep 1.875 * max <= 4.0.
_PLANNED_SPEED_MAX_RAD_S = 2.0

# Motor error codes 0x0 (disabled) and 0x1 (normal) are healthy; anything
# else is a fault (matches ArmRobot._check_motor_errors).
_HEALTHY_MOTOR_CODES = (0, 1)
_A1Z_DOF = 6
_SDK_CONTROL_FREQ_HZ = 250

_STARTUP_FEEDBACK_TIMEOUT_S = 0.5
_STARTUP_RAMP_DURATION_S = 1.0
_STARTUP_SAMPLE_PERIOD_S = 0.02
_STARTUP_MAX_VELOCITY_RAD_S = 0.5
_STARTUP_SETTLED_VELOCITY_RAD_S = 0.1
_STARTUP_SETTLING_TIMEOUT_S = 1.0
_STARTUP_STAGED_SAMPLES = 5
_STARTUP_SETTLED_SAMPLES = 5

_SYS_CLASS_NET = Path("/sys/class/net")
_A1Z_SOCKETCAN_DRIVER = "gs_usb"


def _socketcan_channel_error(channel: str) -> str | None:
    """Return why a channel is unsafe for A1Z SocketCAN, or None if ready."""
    interface_path = _SYS_CLASS_NET / channel
    try:
        flags = int((interface_path / "flags").read_text(), 16)
    except FileNotFoundError:
        return (
            f"SocketCAN interface {channel!r} does not exist. The HHS adapter must be "
            "bound to the Linux gs_usb driver; pass its interface with --can-port."
        )
    except (OSError, ValueError) as exc:
        return f"cannot read SocketCAN interface {channel!r}: {exc}"

    try:
        driver = (interface_path / "device" / "driver").resolve(strict=True).name
    except OSError as exc:
        return f"cannot verify the kernel driver for SocketCAN interface {channel!r}: {exc}"

    if driver != _A1Z_SOCKETCAN_DRIVER:
        return (
            f"SocketCAN interface {channel!r} belongs to kernel driver {driver!r}, not "
            f"the HHS adapter driver {_A1Z_SOCKETCAN_DRIVER!r}. Pass the HHS SocketCAN "
            "interface with --can-port."
        )
    if not flags & 0x1:
        return (
            f"SocketCAN interface {channel!r} is DOWN. Configure it for 1 Mbit/s and "
            "bring it UP before starting DimOS."
        )
    return None


class GalaxeaA1ZAdapter:
    """Galaxea A1Z 6-DOF arm adapter.

    Implements ManipulatorAdapter protocol via duck typing.
    No inheritance required - just matching method signatures.

    Supported control modes:
    - POSITION: minimum-jerk planned move (SDK move_joints, runs in a
      background thread so the call does not block)
    - SERVO_POSITION: high-frequency joint position streaming
      (SDK command_joint_pos)
    """

    def __init__(
        self,
        address: str = "a1zcan",
        *,
        config: A1ZConfig | None = None,
        dof: int = _A1Z_DOF,
    ) -> None:
        if not address:
            raise ValueError("A1Z CAN interface must not be empty")
        self._config = config or A1ZConfig()
        expected_dof = _A1Z_DOF + int(self._config.gripper is not None)
        if dof != expected_dof:
            raise ValueError(
                f"A1Z joint count must match its device configuration (got {dof}, expected {expected_dof})"
            )
        self._dof = dof
        self._gripper_dof = dof - _A1Z_DOF
        self._can_channel = address
        self._transport: Literal["gs_usb", "socketcan"] = (
            "gs_usb" if platform.system() == "Darwin" else "socketcan"
        )
        self._robot: ArmRobot
        self._connected: bool = False
        self._control_mode: ControlMode = ControlMode.POSITION
        self._move_thread: threading.Thread | None = None
        self._move_lock = threading.Lock()

    def connect(self) -> bool:
        """Open the CAN bus and construct the robot. Motors stay unpowered."""
        if self._connected:
            return True
        if self._transport == "socketcan":
            channel_error = _socketcan_channel_error(self._can_channel)
            if channel_error is not None:
                print(f"ERROR: Galaxea A1Z SocketCAN configuration: {channel_error}")
                return False

        try:
            if self._transport == "gs_usb":
                with gs_usb_can_bus():
                    self._robot = self._create_robot()
            else:
                self._robot = self._create_robot()
            self._connected = True
            print(f"Galaxea A1Z connected via {self._transport} (channel {self._can_channel})")
            return True
        except TypeError as e:
            print(
                "ERROR: installed a1z SDK does not support the gripper - "
                "run `dimos hardware a1z doctor --software-only`: "
                f"{e}"
            )
            return False
        except Exception as e:
            print(f"ERROR: Failed to connect to Galaxea A1Z on {self._can_channel}: {e}")
            return False

    def _create_robot(self) -> ArmRobot:
        gripper = self._config.gripper
        return get_a1z_robot(
            can_channel=self._can_channel,
            gravity_comp_factor=self._config.gravity_comp_factor,
            zero_gravity_mode=self._config.teaching is not None,
            control_freq_hz=_SDK_CONTROL_FREQ_HZ,
            urdf_path=self._config.urdf_path,
            default_kp=np.asarray(self._config.default_kp, dtype=float),
            default_kd=np.asarray(self._config.default_kd, dtype=float),
            with_gripper=gripper is not None,
            gripper_max_torque=gripper.max_torque if gripper else 0.5,
        )

    def disconnect(self) -> None:
        """Stop the control loop, disable motors, and close the CAN bus.

        SAFETY: the arm has no brakes and will fall when motors disable.
        """
        if not self._connected:
            return
        try:
            if self._robot.is_running:
                self._robot.stop()
        except Exception:
            pass
        self._ensure_motors_disabled()
        try:
            # ArmRobot.stop() does not close the bus; shut it down so the
            # CAN channel is reusable without recreating the process.
            self._robot._bus.shutdown()
        except Exception:
            pass
        finally:
            self._connected = False

    def is_connected(self) -> bool:
        """Check if connected (CAN bus open, robot constructed)."""
        return self._connected

    def activate(self) -> bool:
        """Enable motors and start the SDK control loop."""
        return self.write_enable(True)

    def deactivate(self) -> bool:
        """Stop the control loop and disable motors.

        SAFETY: the arm has no brakes and will fall when motors disable.
        Lower the arm (e.g. move_joints to a rest pose) before calling.
        """
        return self.write_enable(False)

    def get_info(self) -> ManipulatorInfo:
        """Get manipulator info."""
        return ManipulatorInfo(vendor="Galaxea", model="A1Z", dof=self._dof)

    def get_dof(self) -> int:
        """Total joints owned by this adapter."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Arm limits in radians, then the gripper's jaw opening in metres."""
        gripper = self._config.gripper
        gripper_upper = [gripper.max_opening_m] if self._gripper_dof and gripper else []
        return JointLimits(
            position_lower=list(_POSITION_LOWER) + [0.0] * self._gripper_dof,
            position_upper=list(_POSITION_UPPER) + gripper_upper,
            velocity_max=list(_VELOCITY_MAX) + [0.0] * self._gripper_dof,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set control mode. Only POSITION and SERVO_POSITION are supported.

        Both map onto the same underlying MIT position+PD loop, so switching
        needs no SDK call.
        """
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        """Arm positions in radians, then the gripper opening in metres."""
        positions = cast("list[float]", self._joint_state()["pos"].tolist())
        if self._gripper_dof:
            positions.append(self._read_gripper())
        return positions

    def read_joint_velocities(self) -> list[float]:
        """Arm velocities in rad/s; the gripper reports 0.0 (no feedback)."""
        values = cast("list[float]", self._joint_state()["vel"].tolist())
        return values + [0.0] * self._gripper_dof

    def read_joint_efforts(self) -> list[float]:
        """Arm efforts in Nm; the gripper reports 0.0 (no feedback)."""
        values = cast("list[float]", self._joint_state()["eff"].tolist())
        return values + [0.0] * self._gripper_dof

    def read_state(self) -> dict[str, int]:
        """Read robot state (0=idle, 1=running, 2=error/estopped)."""
        if not self._connected:
            return {"state": 0, "mode": 0, "error_code": 0}

        error_code, _ = self.read_error()
        if error_code != 0 or self._robot.is_estopped:
            state = 2
        elif self._robot.is_running:
            state = 1
        else:
            state = 0

        joint_state = self._joint_state()
        return {
            "state": state,
            "mode": 0,
            "error_code": error_code,
            "temp_mos_max": int(max(joint_state["temp_mos"].tolist())),
            "temp_rotor_max": int(max(joint_state["temp_rotor"].tolist())),
        }

    def read_error(self) -> tuple[int, str]:
        """Read error code and message. (0, '') means no error."""
        if not self._connected:
            return 0, ""

        codes = self._joint_state()["error_codes"].tolist()
        for i, code in enumerate(codes):
            if int(code) not in _HEALTHY_MOTOR_CODES:
                return int(code), f"Motor fault on joint {i + 1}: code 0x{int(code):x}"
        if self._robot.is_estopped:
            return 1, "Soft e-stop latched (write_clear_errors to release)"
        return 0, ""

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Command joint positions (radians).

        POSITION mode: minimum-jerk planned move in a background thread;
        returns False if a planned move is already in progress.
        SERVO_POSITION mode: single streamed position target.

        Args:
            positions: Target positions, arm in radians then gripper in metres
            velocity: Speed as fraction of max planned speed (0-1)
        """
        if not self._connected or not self._robot.is_running or self._robot.is_estopped:
            return False
        if len(positions) != self._dof:
            return False

        arm = positions[:_A1Z_DOF]
        grip = positions[_A1Z_DOF:]
        if grip:
            self._write_gripper(grip[0])

        target = np.asarray(arm, dtype=float)

        if self._control_mode == ControlMode.SERVO_POSITION:
            self._robot.command_joint_pos(target)
            return True

        # POSITION mode: reject overlapping planned moves
        if not self._move_lock.acquire(blocking=False):
            return False

        speed = max(0.05, min(1.0, velocity)) * _PLANNED_SPEED_MAX_RAD_S

        def _move() -> None:
            try:
                self._robot.move_joints(target, speed=speed)
            except Exception as e:
                print(f"Galaxea A1Z planned move failed: {e}")
            finally:
                self._move_lock.release()

        self._move_thread = threading.Thread(target=_move, name="a1z_planned_move", daemon=True)
        self._move_thread.start()
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Not supported - the a1z SDK has no velocity command API."""
        return False

    def write_stop(self) -> bool:
        """Latching soft e-stop: pins current position, rejects commands.

        Release with write_clear_errors() or write_enable(True).
        """
        if not self._connected:
            return False
        try:
            self._robot.estop()
            return True
        except Exception:
            return False

    def write_enable(self, enable: bool) -> bool:
        """Enable (start control loop) or disable (stop loop, motors off).

        SAFETY: disabling powers off motors and the arm falls freely.
        """
        if not self._connected:
            # Disabling with no robot is a no-op success (already torn down);
            # enabling still requires a connection.
            return not enable

        try:
            if enable:
                if self._robot.is_running:
                    if self._robot.is_estopped:
                        self._robot.release()
                elif self._config.teaching is not None:
                    # The vendor's zero-gravity startup deliberately allows
                    # motion while gravity compensation takes over.  The
                    # position-hold safe start below requires the arm to
                    # settle, so it is only valid for position-controlled
                    # operation (planning/replay), not hand teaching.
                    self._robot.start()
                else:
                    self._safe_start()
                teaching = self._config.teaching
                if (
                    teaching
                    and teaching.gripper_free_drive
                    and not self.set_gripper_free_drive(True)
                ):
                    self._robot.stop()
                    self._ensure_motors_disabled()
                    raise RuntimeError(
                        "gripper free-drive requested, but the installed A1Z SDK does not "
                        "support set_gripper_free_drive()"
                    )
                return True
            else:
                if self._config.teaching and self._config.teaching.gripper_free_drive:
                    self.set_gripper_free_drive(False)
                if self._robot.is_running:
                    self._robot.stop()
                self._ensure_motors_disabled()
                return True
        except Exception as e:
            print(f"Galaxea A1Z enable={enable} failed: {e}")
            return False

    def read_enabled(self) -> bool:
        """Check if the control loop is running and not e-stopped."""
        return self._connected and self._robot.is_running and not self._robot.is_estopped

    def write_clear_errors(self) -> bool:
        """Release the soft e-stop latch.

        Motor-level faults cannot be cleared here; use the SDK's
        tools/motor_diag.py --clear-error with the arm in a safe pose.
        """
        if not self._connected:
            return False
        try:
            if self._robot.is_estopped:
                self._robot.release()
            return True
        except Exception:
            return False

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose via forward kinematics on the bundled URDF.

        Returns:
            Dict with keys: x, y, z (meters), roll, pitch, yaw (radians)
            None if not connected or pinocchio is unavailable
        """
        if not self._connected:
            return None

        try:
            kin = self._kinematics
            q = np.asarray(self.read_joint_positions())
            T = kin.fk(q)  # 4x4 homogeneous transform
            R = T[:3, :3]
            return {
                "x": float(T[0, 3]),
                "y": float(T[1, 3]),
                "z": float(T[2, 3]),
                "roll": float(np.arctan2(R[2, 1], R[2, 2])),
                "pitch": float(np.arctan2(-R[2, 0], np.hypot(R[2, 1], R[2, 2]))),
                "yaw": float(np.arctan2(R[1, 0], R[0, 0])),
            }
        except Exception:
            return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Not supported - cartesian targets go through the planning stack."""
        return False

    def _read_gripper(self) -> float:
        """Read the gripper opening in metres (SDK reports a 0-1 fraction)."""
        gripper = self._config.gripper
        if not self._connected or gripper is None:
            return 0.0

        try:
            fraction = self._robot.gripper.get_feedback_norm()
        except Exception:
            return 0.0
        if fraction is None:
            return 0.0
        return float(fraction) * gripper.max_opening_m

    def _write_gripper(self, position: float) -> bool:
        """Command the gripper opening in metres."""
        gripper = self._config.gripper
        if (
            not self._connected
            or gripper is None
            or not self._robot.is_running
            or self._robot.is_estopped
        ):
            return False
        fraction = max(0.0, min(1.0, position / gripper.max_opening_m))
        try:
            self._robot.command_gripper(fraction)
            return True
        except Exception as e:
            print(f"Galaxea A1Z gripper command failed: {e}")
            return False

    def read_force_torque(self) -> list[float] | None:
        """Not supported - no F/T sensor (per-joint efforts via read_joint_efforts)."""
        return None

    def set_gripper_free_drive(self, enabled: bool) -> bool:
        """Toggle gripper free-drive (zero-torque) mode for hand teaching.

        Requires gripper=True and the SDK's 'gripper' branch.
        """
        robot = self._require_robot()
        if self._config.gripper is None:
            return False
        robot.set_gripper_free_drive(enabled)
        return True

    def _arm_motors(self) -> list[Any]:
        chain = self._robot._motor_chain
        return [*chain._motor_a_list, *chain._motor_b_list]

    def _ensure_motors_disabled(self) -> None:
        """Re-send disable frames to every motor, gripper included.

        The SDK's shutdown sends the gripper's disable frame exactly once;
        on a busy or degraded bus that frame can be lost, leaving the gripper
        energized and unsupervised (observed twice on hardware). The SDK
        double-sends arm-motor disables for this very reason but not the
        gripper's, so we re-send all of them here.
        """
        if not self._connected:
            return
        motors = self._arm_motors()
        if self._config.gripper is not None:
            motors.append(self._robot.gripper._motor)
        for _ in range(2):
            for motor in motors:
                try:
                    motor.disable()
                except Exception:
                    pass

    def _safe_start(self) -> None:
        """Start the SDK loop with measured-pose hold before model feedforward.

        The SDK's start() reads feedback once after a fixed 50 ms wait and
        position-holds whatever it read; if a motor's first report is late
        (typical on USB transports), the hold target defaults to zero and
        the arm snaps to neutral at full gain. Observed on hardware.

        Start with both position gain and model feedforward at zero, wait for
        every motor to report, and validate the measured state. Establish a
        measured-pose PD hold while feedforward is still zero, verify that
        hold, and only then ramp the configured gravity factor. This ordering
        matters: kp=0 alone is not zero force because the SDK's gravity
        feedforward remains active independently of position gain.

        The A1Z has no brakes. It must be supported during activation and any
        failed activation disables the motors after first removing commanded
        gain and model feedforward.
        """
        robot = self._robot
        robot_info = robot.get_robot_info()
        default_kp = np.asarray(robot_info["default_kp"], dtype=float)
        default_kd = np.asarray(robot_info["default_kd"], dtype=float)

        configured_gravity_factor = float(robot.gravity_comp_factor)
        robot.gravity_comp_factor = 0.0

        try:
            robot.start(initial_kp=np.zeros(_A1Z_DOF), initial_kd=default_kd * 0.5)

            deadline = time.monotonic() + _STARTUP_FEEDBACK_TIMEOUT_S
            while time.monotonic() < deadline and not self._all_motors_reported():
                time.sleep(_STARTUP_SAMPLE_PERIOD_S)
            if not self._all_motors_reported():
                raise RuntimeError(
                    f"no feedback from all motors within {_STARTUP_FEEDBACK_TIMEOUT_S:.1f} s"
                )

            hold_pos, _ = self._validated_startup_state(
                robot.get_joint_state(),
                phase="initial feedback",
            )

            # The measured target has zero position error at this instant,
            # so full holding gains add stiffness without requesting a
            # position step. Teaching mode uses the vendor startup instead.
            robot.command_joint_state(
                {
                    "pos": hold_pos.copy(),
                    "vel": np.zeros(_A1Z_DOF),
                    "kp": default_kp,
                    "kd": default_kd,
                }
            )
            ramp_steps = max(
                1,
                round(_STARTUP_RAMP_DURATION_S / _STARTUP_SAMPLE_PERIOD_S),
            )
            gravity_schedule = [0.0] * _STARTUP_STAGED_SAMPLES + [
                configured_gravity_factor * step / ramp_steps for step in range(1, ramp_steps + 1)
            ]
            for sample, gravity_factor in enumerate(gravity_schedule, start=1):
                robot.gravity_comp_factor = gravity_factor
                self._sample_startup_state(phase=f"staged gravity {sample}/{len(gravity_schedule)}")

            self._wait_for_startup_settling()
        except Exception:
            self._quiesce_and_stop_after_failed_start()
            raise

    def _wait_for_startup_settling(self) -> None:
        """Wait for consecutive stable samples after the gravity ramp.

        Encoder-derived velocity occasionally contains an isolated sample just
        above the settled threshold. Keep the hard startup velocity ceiling on
        every sample, but only declare the arm settled after a consecutive
        stable window. Sustained motion still fails within a bounded timeout.
        """
        max_samples = max(
            _STARTUP_SETTLED_SAMPLES,
            round(_STARTUP_SETTLING_TIMEOUT_S / _STARTUP_SAMPLE_PERIOD_S),
        )
        stable_samples = 0
        last_pos = np.zeros(_A1Z_DOF)
        last_vel = np.zeros(_A1Z_DOF)
        for sample in range(1, max_samples + 1):
            last_pos, last_vel = self._sample_startup_state(
                phase=f"settling {sample}/{max_samples}"
            )
            if np.all(np.abs(last_vel) <= _STARTUP_SETTLED_VELOCITY_RAD_S):
                stable_samples += 1
                if stable_samples >= _STARTUP_SETTLED_SAMPLES:
                    return
            else:
                stable_samples = 0

        raise RuntimeError(
            f"arm did not settle within {_STARTUP_SETTLING_TIMEOUT_S:.1f} s; "
            f"required {_STARTUP_SETTLED_SAMPLES} consecutive samples at or below "
            f"{_STARTUP_SETTLED_VELOCITY_RAD_S:.3f} rad/s; "
            f"positions={np.round(last_pos, 3).tolist()}, "
            f"velocities={np.round(last_vel, 3).tolist()}"
        )

    def _validated_startup_state(
        self,
        state: dict[str, Any],
        *,
        phase: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Validate one startup sample and return position and velocity."""
        if not self._connected or not self._robot.is_running:
            raise RuntimeError(f"SDK control loop stopped during {phase}")
        pos = np.asarray(state["pos"], dtype=float)
        vel = np.asarray(state["vel"], dtype=float)
        expected_shape = (_A1Z_DOF,)
        if pos.shape != expected_shape or vel.shape != expected_shape:
            raise RuntimeError(
                f"invalid state shape during {phase}: pos={pos.shape}, vel={vel.shape}"
            )
        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(vel)):
            raise RuntimeError(
                f"non-finite state during {phase}: "
                f"pos={np.round(pos, 3).tolist()}, vel={np.round(vel, 3).tolist()}"
            )

        lower = np.asarray(_POSITION_LOWER) - 0.15
        upper = np.asarray(_POSITION_UPPER) + 0.15
        outside = (pos < lower) | (pos > upper)
        if np.any(outside):
            offenders = ", ".join(f"joint{i + 1}={pos[i]:.3f}" for i in np.flatnonzero(outside))
            raise RuntimeError(
                f"start pose outside joint limits during {phase}: {offenders}; "
                f"positions={np.round(pos, 3).tolist()}"
            )

        too_fast = np.abs(vel) > _STARTUP_MAX_VELOCITY_RAD_S
        if np.any(too_fast):
            offenders = ", ".join(
                f"joint{i + 1}={vel[i]:.3f} rad/s" for i in np.flatnonzero(too_fast)
            )
            raise RuntimeError(
                f"arm moving during {phase}: {offenders} "
                f"(limit {_STARTUP_MAX_VELOCITY_RAD_S:.3f} rad/s); "
                f"positions={np.round(pos, 3).tolist()}, "
                f"velocities={np.round(vel, 3).tolist()}"
            )
        return pos, vel

    def _sample_startup_state(self, *, phase: str) -> tuple[np.ndarray, np.ndarray]:
        """Sleep for one SDK sample, then read and validate the arm state."""
        time.sleep(_STARTUP_SAMPLE_PERIOD_S)
        return self._validated_startup_state(self._robot.get_joint_state(), phase=phase)

    def _quiesce_and_stop_after_failed_start(self) -> None:
        """Remove commanded force before disabling after activation failure."""
        if not self._connected:
            return
        robot = self._robot
        robot.gravity_comp_factor = 0.0
        try:
            robot.estop()
        except Exception:
            pass
        try:
            robot.stop()
        except Exception:
            self._ensure_motors_disabled()

    def _all_motors_reported(self) -> bool:
        """True once every motor in the SDK chain has sent real feedback."""
        return all(motor.last_feedback is not None for motor in self._arm_motors())

    def _require_robot(self) -> ArmRobot:
        if not self._connected:
            raise RuntimeError("Not connected")
        return self._robot

    def _joint_state(self) -> dict[str, np.ndarray]:
        return cast("dict[str, np.ndarray]", self._require_robot().get_joint_state())

    @cached_property
    def _kinematics(self) -> Kinematics:
        """Lazily build and cache the FK solver from the SDK's bundled URDF."""
        urdf = self._config.urdf_path or str(
            Path(a1z.__file__).parent / "robot_models" / "a1z" / "A1Z_Flange.urdf"
        )
        return Kinematics(str(urdf))


def create_galaxea_a1z_adapter(
    *,
    address: str | None = None,
    config: A1ZConfig | None = None,
    dof: int = _A1Z_DOF,
    **_: object,
) -> GalaxeaA1ZAdapter:
    """Create the fixed six-axis A1Z adapter from coordinator metadata."""
    return GalaxeaA1ZAdapter(address=address or "a1zcan", config=config, dof=dof)
