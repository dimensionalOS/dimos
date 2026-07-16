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

Zero-gravity (teaching) mode is chosen at construction time via the
zero_gravity kwarg and cannot be switched at runtime; teach recording requires
zero_gravity=True.
"""

from __future__ import annotations

from collections.abc import Iterator
import contextlib
from pathlib import Path
import platform
import threading
import time
from typing import Any

import numpy as np

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)

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

# G1Z gripper max opening used to convert the SDK's normalized position
# (0.0=closed, 1.0=open) to meters. Measured on hardware: 10 cm jaw gap at
# full open. Override via the gripper_max_opening_m constructor kwarg.
_GRIPPER_MAX_OPENING_M = 0.1


def _socketcan_channel_is_up(channel: str) -> bool:
    """True if the channel exists as a network interface and is IFF_UP."""
    try:
        flags = int(Path("/sys/class/net", channel, "flags").read_text(), 16)
    except (OSError, ValueError):
        return False
    return bool(flags & 0x1)


def _gs_usb_adapter_claimable() -> bool:
    """True if the HHS USB-CANFD adapter is on the USB bus and no kernel
    driver owns it (a bound kernel driver means socketcan is the right path
    and userspace could not claim the interface anyway)."""
    try:
        import usb.core

        from dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus import (
            GALAXEA_PRODUCT_ID,
            GALAXEA_VENDOR_ID,
        )

        device = usb.core.find(idVendor=GALAXEA_VENDOR_ID, idProduct=GALAXEA_PRODUCT_ID)
        if device is None:
            return False
        try:
            return not device.is_kernel_driver_active(0)
        except NotImplementedError:
            return True
        except usb.core.USBError:
            print(
                "Galaxea A1Z: found the USB-CANFD adapter but cannot open it "
                "(insufficient permissions). Grant access with a udev rule: "
                'echo \'SUBSYSTEM=="usb", ATTRS{idVendor}=="a8fa", '
                'ATTRS{idProduct}=="8598", MODE="0666"\' | '
                "sudo tee /etc/udev/rules.d/99-canfd.rules && "
                "sudo udevadm control --reload-rules && sudo udevadm trigger"
            )
            return False
    except Exception:
        return False


def _resolve_auto_transport(channel: str) -> str:
    """Pick the CAN transport for transport="auto".

    socketcan stays the default whenever the channel is up (stock Linux
    kernels bind gs_usb adapters through socketcan). When it is not, fall
    back to the userspace gs_usb transport if the HHS adapter is on the USB
    bus with no kernel driver bound - some kernels ship a gs_usb driver that
    cannot drive this device (VID/PID not in its table, and pre-6.x gs_usb
    hardcodes TX endpoint 0x02 where this adapter uses 0x01). A channel that
    merely exists is not enough: an unrelated on-board CAN controller can
    expose the same name with nothing wired to it.
    """
    if platform.system() == "Darwin":
        return "gs_usb"
    if _socketcan_channel_is_up(channel):
        return "socketcan"
    if _gs_usb_adapter_claimable():
        print(
            f"Galaxea A1Z: socketcan channel {channel!r} is not up but the "
            "HHS USB-CANFD adapter is present with no kernel driver bound - "
            "falling back to the userspace gs_usb transport"
        )
        return "gs_usb"
    return "socketcan"


@contextlib.contextmanager
def _gs_usb_can_bus() -> Iterator[None]:
    """Route the SDK's CAN bus construction through GsUsbMacBus.

    get_a1z_robot() hardcodes bustype="socketcan" (Linux-only) when opening
    the bus, so on macOS we swap can.interface.Bus for the userspace gs_usb
    transport for the duration of the factory call. Everything else in the
    vendor stack is transport-agnostic.
    """
    import can

    from dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus import GsUsbMacBus

    original_bus = can.interface.Bus

    def _bus_factory(*args: Any, **kwargs: Any) -> GsUsbMacBus:
        return GsUsbMacBus(bitrate=kwargs.get("bitrate", 1_000_000))

    can.interface.Bus = _bus_factory  # type: ignore[assignment]
    try:
        yield
    finally:
        can.interface.Bus = original_bus  # type: ignore[assignment]


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
        address: str = "can0",
        dof: int = 6,
        *,
        gravity_comp_factor: float = 1.0,
        zero_gravity: bool = False,
        control_freq_hz: int = 250,
        urdf_path: str | None = None,
        gripper: bool = False,
        gripper_max_torque: float = 2.0,
        gripper_max_opening_m: float = _GRIPPER_MAX_OPENING_M,
        transport: str = "auto",
        safe_start: bool = True,
        **_: object,
    ) -> None:
        if transport not in ("auto", "socketcan", "gs_usb"):
            raise ValueError(f"Unknown transport {transport!r}")
        if dof != 6:
            raise ValueError(f"GalaxeaA1ZAdapter only supports 6 DOF (got {dof})")
        self._can_channel = address
        self._dof = dof
        self._gravity_comp_factor = gravity_comp_factor
        self._zero_gravity = zero_gravity
        self._control_freq_hz = control_freq_hz
        self._urdf_path = urdf_path
        self._gripper = gripper
        self._gripper_max_torque = gripper_max_torque
        self._gripper_max_opening_m = gripper_max_opening_m
        if transport == "auto":
            transport = _resolve_auto_transport(address)
        self._transport = transport
        # Dimensional addition on top of the vendor SDK (not Galaxea behavior):
        # verified zero-force startup that prevents the enable-snap described
        # in _safe_start(). Set safe_start=False for vendor-stock start().
        self._safe_start_enabled = safe_start
        self._robot: Any = None
        self._connected: bool = False
        self._control_mode: ControlMode = ControlMode.POSITION
        self._move_thread: threading.Thread | None = None
        self._move_lock = threading.Lock()
        self._kinematics: Any = None

    def connect(self) -> bool:
        """Open the CAN bus and construct the robot. Motors stay unpowered."""
        try:
            from a1z.robots.get_robot import get_a1z_robot
        except ImportError:
            print(
                "ERROR: a1z SDK not installed. Install from github.com/userguide-galaxea/GALAXEA-A1Z"
            )
            return False

        kwargs: dict[str, Any] = {
            "can_channel": self._can_channel,
            "gravity_comp_factor": self._gravity_comp_factor,
            "zero_gravity_mode": self._zero_gravity,
            "control_freq_hz": self._control_freq_hz,
            "urdf_path": self._urdf_path,
        }
        if self._gripper:
            # Only on the SDK's 'gripper' branch; main raises TypeError.
            kwargs["with_gripper"] = True
            kwargs["gripper_max_torque"] = self._gripper_max_torque

        try:
            if self._transport == "gs_usb":
                with _gs_usb_can_bus():
                    self._robot = get_a1z_robot(**kwargs)
            else:
                self._robot = get_a1z_robot(**kwargs)
            self._connected = True
            print(f"Galaxea A1Z connected via {self._transport} (channel {self._can_channel})")
            return True
        except TypeError as e:
            print(
                "ERROR: installed a1z SDK does not support the gripper - "
                f"install the SDK's 'gripper' branch: {e}"
            )
            self._robot = None
            return False
        except Exception as e:
            print(f"ERROR: Failed to connect to Galaxea A1Z on {self._can_channel}: {e}")
            self._robot = None
            return False

    def disconnect(self) -> None:
        """Stop the control loop, disable motors, and close the CAN bus.

        SAFETY: the arm has no brakes and will fall when motors disable.
        """
        if self._robot:
            try:
                if self._robot.is_running:
                    self._robot.stop()
            except Exception:
                pass
            self._ensure_motors_disabled()
            try:
                # ArmRobot.stop() does not close the bus; shut it down so the
                # CAN channel is reusable without recreating the process.
                bus = getattr(self._robot, "_bus", None)
                if bus is not None:
                    bus.shutdown()
            except Exception:
                pass
            finally:
                self._robot = None
                self._connected = False

    def is_connected(self) -> bool:
        """Check if connected (CAN bus open, robot constructed)."""
        return self._connected and self._robot is not None

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
        """Get degrees of freedom."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Get joint limits."""
        return JointLimits(
            position_lower=list(_POSITION_LOWER),
            position_upper=list(_POSITION_UPPER),
            velocity_max=list(_VELOCITY_MAX),
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
        """Read current joint positions (radians)."""
        return self._joint_state()["pos"].tolist()

    def read_joint_velocities(self) -> list[float]:
        """Read current joint velocities (rad/s)."""
        return self._joint_state()["vel"].tolist()

    def read_joint_efforts(self) -> list[float]:
        """Read current joint efforts (Nm)."""
        return self._joint_state()["eff"].tolist()

    def read_state(self) -> dict[str, int]:
        """Read robot state (0=idle, 1=running, 2=error/estopped)."""
        if not self._robot:
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
        if not self._robot:
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
            positions: Target positions in radians
            velocity: Speed as fraction of max planned speed (0-1)
        """
        if not self._robot or not self._robot.is_running or self._robot.is_estopped:
            return False

        target = np.asarray(positions, dtype=float)

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
        if not self._robot:
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
        if not self._robot:
            return False

        try:
            if enable:
                if self._robot.is_running:
                    if self._robot.is_estopped:
                        self._robot.release()
                elif self._safe_start_enabled:
                    self._safe_start()
                else:
                    # Vendor-stock startup; can snap to zero if a motor's
                    # first feedback is late (see _safe_start docstring).
                    self._robot.start()
                return True
            else:
                if self._robot.is_running:
                    self._robot.stop()
                self._ensure_motors_disabled()
                return True
        except Exception as e:
            print(f"Galaxea A1Z enable={enable} failed: {e}")
            return False

    def read_enabled(self) -> bool:
        """Check if the control loop is running and not e-stopped."""
        return bool(self._robot and self._robot.is_running and not self._robot.is_estopped)

    def write_clear_errors(self) -> bool:
        """Release the soft e-stop latch.

        Motor-level faults cannot be cleared here; use the SDK's
        tools/motor_diag.py --clear-error with the arm in a safe pose.
        """
        if not self._robot:
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
        if not self._robot:
            return None

        kin = self._get_kinematics()
        if kin is None:
            return None

        try:
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

    def read_gripper_position(self) -> float | None:
        """Read gripper opening (meters). None if no gripper attached.

        Converts the SDK's normalized position (0.0=closed, 1.0=open).
        Requires the adapter constructed with gripper=True and the SDK's
        'gripper' branch.
        """
        if not self._robot or not self._gripper:
            return None
        try:
            fraction = self._robot.get_gripper_pos()
        except Exception:
            return None
        if fraction is None:
            return None
        return float(fraction) * self._gripper_max_opening_m

    def write_gripper_position(self, position: float) -> bool:
        """Command gripper opening (meters). False if no gripper attached."""
        if not self._robot or not self._gripper or not self._robot.is_running:
            return False
        fraction = max(0.0, min(1.0, position / self._gripper_max_opening_m))
        try:
            self._robot.command_gripper(fraction)
            return True
        except Exception as e:
            print(f"Galaxea A1Z gripper command failed: {e}")
            return False

    def read_force_torque(self) -> list[float] | None:
        """Not supported - no F/T sensor (per-joint efforts via read_joint_efforts)."""
        return None

    # --- A1Z-specific extensions (beyond ManipulatorAdapter protocol) ---

    def start_teach_recording(self, sample_hz: int = 50) -> None:
        """Start recording joint positions for teach-and-play.

        Requires the adapter constructed with zero_gravity=True so the arm
        can be hand-guided.
        """
        self._require_robot().start_recording(sample_hz=sample_hz)

    def stop_teach_recording(self) -> list[tuple[float, np.ndarray]]:
        """Stop recording and return the trajectory as (timestamp_s, pos_rad) tuples."""
        return self._require_robot().stop_recording()

    def play_trajectory(self, trajectory: list, speed_factor: float = 1.0) -> None:
        """Replay a recorded trajectory (blocking)."""
        self._require_robot().play_trajectory(trajectory, speed_factor=speed_factor)

    def save_recording(self, trajectory: list, path: str) -> None:
        """Save a recorded trajectory to a JSON file."""
        self._require_robot().save_recording(trajectory, path)

    def load_recording(self, path: str) -> list:
        """Load a recorded trajectory from a JSON file."""
        return self._require_robot().load_recording(path)

    def set_gripper_free_drive(self, enabled: bool) -> bool:
        """Toggle gripper free-drive (zero-torque) mode for hand teaching.

        Requires gripper=True and the SDK's 'gripper' branch.
        """
        robot = self._require_robot()
        if not self._gripper or not hasattr(robot, "set_gripper_free_drive"):
            return False
        robot.set_gripper_free_drive(enabled)
        return True

    # --- internals ---

    def _ensure_motors_disabled(self) -> None:
        """Re-send disable frames to every motor, gripper included.

        The SDK's shutdown sends the gripper's disable frame exactly once;
        on a busy or degraded bus that frame can be lost, leaving the gripper
        energized and unsupervised (observed twice on hardware). The SDK
        double-sends arm-motor disables for this very reason but not the
        gripper's, so we re-send all of them here.
        """
        robot = self._robot
        if robot is None:
            return
        motors: list[Any] = []
        chain = getattr(robot, "_motor_chain", None)
        if chain is not None:
            motors += list(getattr(chain, "_motor_a_list", []))
            motors += list(getattr(chain, "_motor_b_list", []))
        gripper = getattr(robot, "gripper", None)
        if gripper is not None:
            motors.append(gripper._motor)
        for _ in range(2):
            for motor in motors:
                try:
                    motor.disable()
                except Exception:
                    pass

    def _safe_start(self) -> None:
        """Start the SDK control loop without ever commanding force toward
        an unverified position.

        The SDK's start() reads feedback once after a fixed 50 ms wait and
        position-holds whatever it read; if a motor's first report is late
        (typical on USB transports), the hold target defaults to zero and
        the arm snaps to neutral at full gain. Observed on hardware.

        Sequence here: start with kp=0 (gravity comp only - a position snap
        is physically impossible), wait until every motor has actually
        reported, verify the pose is inside limits and near-stationary, then
        engage the hold gains at the measured pose (zero error, zero jerk).
        Raises RuntimeError (and disables) instead of moving if verification
        fails.
        """
        robot = self._robot
        dof = self._dof
        default_kp = np.asarray(
            getattr(robot, "_default_kp", np.array([30.0, 30.0, 30.0, 20.0, 5.0, 5.0])),
            dtype=float,
        )
        default_kd = np.asarray(
            getattr(robot, "_default_kd", np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])),
            dtype=float,
        )

        robot.start(initial_kp=np.zeros(dof), initial_kd=default_kd * 0.5)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not self._all_motors_reported():
            time.sleep(0.02)
        if not self._all_motors_reported():
            robot.stop()
            raise RuntimeError("no feedback from all motors within 2 s; motors disabled")

        state = robot.get_joint_state()
        pos = np.asarray(state["pos"], dtype=float)
        vel = np.asarray(state["vel"], dtype=float)
        lower = np.asarray(_POSITION_LOWER) - 0.15
        upper = np.asarray(_POSITION_UPPER) + 0.15
        if np.any(pos < lower) or np.any(pos > upper):
            robot.stop()
            raise RuntimeError(
                f"start pose {np.round(pos, 3).tolist()} outside joint limits - "
                "move the arm inside its range by hand, then retry; motors disabled"
            )
        if np.any(np.abs(vel) > 0.5):
            robot.stop()
            raise RuntimeError("arm is moving during startup; motors disabled")

        if not self._zero_gravity:
            # Engage hold gains at the *measured* pose: error ~0, no jerk.
            robot.command_joint_state(
                {
                    "pos": pos.copy(),
                    "vel": np.zeros(dof),
                    "kp": default_kp,
                    "kd": default_kd,
                }
            )

    def _all_motors_reported(self) -> bool:
        """True once every motor in the SDK chain has sent real feedback."""
        chain = getattr(self._robot, "_motor_chain", None)
        if chain is None:
            return True  # can't verify on this SDK version; rely on limit checks
        motors = list(getattr(chain, "_motor_a_list", [])) + list(
            getattr(chain, "_motor_b_list", [])
        )
        if not motors:
            return True
        return all(m.last_feedback is not None for m in motors)

    def _require_robot(self) -> Any:
        if not self._robot:
            raise RuntimeError("Not connected")
        return self._robot

    def _joint_state(self) -> dict[str, np.ndarray]:
        return self._require_robot().get_joint_state()

    def _get_kinematics(self) -> Any:
        """Lazily build and cache the FK solver from the SDK's bundled URDF."""
        if self._kinematics is not None:
            return self._kinematics
        try:
            import a1z
            from a1z.robots.kinematics import Kinematics

            urdf = self._urdf_path or str(
                Path(a1z.__file__).parent / "robot_models" / "a1z" / "A1Z_Flange.urdf"
            )
            self._kinematics = Kinematics(urdf)
        except Exception:
            return None
        return self._kinematics
