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

"""A Galaxea A1Z arm as a connection module, over CAN with Galaxea's a1z SDK."""

from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
import platform
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.limits import model_limit, movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.hardware.manipulators.galaxea_a1z.config import A1ZConfig, A1ZGripperConfig
from dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus import gs_usb_can_bus
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.a1z.config import A1Z_DOF, A1Z_GRIPPER_OPENING_M, a1z_model
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The device's gains, gravity compensation and gripper torque, as Galaxea tunes them.
_DEVICE = A1ZConfig(gripper=A1ZGripperConfig())
_SDK_CONTROL_HZ = 250
# Motor error codes 0x0 (off) and 0x1 (normal) are healthy; any other is a fault.
_HEALTHY_MOTOR_CODES = (0, 1)

_READ_HZ = 100.0
# Joint targets are refused once the latest joint reading is older than this,
# in seconds: ten missed readings.
_READING_MAX_AGE_S = 10 / _READ_HZ
# The gripper is read on every 10th reading of the joints.
_GRIPPER_READ_EVERY = 10

# Starting the arm: how long every motor has to report, in seconds; then one
# state sample every _STARTUP_SAMPLE_PERIOD_S while the arm holds where it is,
# gravity compensation stays off for _STARTUP_STAGED_SAMPLES, then ramps in
# over _STARTUP_RAMP_DURATION_S; then _STARTUP_SETTLED_SAMPLES samples in a row
# under _STARTUP_SETTLED_VELOCITY_RAD_S, within _STARTUP_SETTLING_TIMEOUT_S.
# A joint faster than _STARTUP_MAX_VELOCITY_RAD_S, or past its limits by more
# than _STARTUP_LIMIT_MARGIN_RAD, stops the start.
_STARTUP_FEEDBACK_TIMEOUT_S = 0.5
_STARTUP_SAMPLE_PERIOD_S = 0.02
_STARTUP_STAGED_SAMPLES = 5
_STARTUP_RAMP_DURATION_S = 1.0
_STARTUP_SETTLED_SAMPLES = 5
_STARTUP_SETTLED_VELOCITY_RAD_S = 0.1
_STARTUP_SETTLING_TIMEOUT_S = 1.0
_STARTUP_MAX_VELOCITY_RAD_S = 0.5
_STARTUP_LIMIT_MARGIN_RAD = 0.15

# Going home, by the SDK's move_joints: its average speed in rad/s (it peaks at
# 1.875 times that and refuses a peak over 4 rad/s), slower when starting than
# when stopping, which DimOS gives about 5 s. How close every joint must come, in rad.
_HOME_SPEED_RAD_S = 0.5
_PARK_SPEED_RAD_S = 1.0
_HOME_TOLERANCE_RAD = 0.05
_POLL_S = 0.01

_SYS_CLASS_NET = Path("/sys/class/net")
# The Linux kernel driver of the HHS USB-CANFD adapter Galaxea ships.
_HHS_DRIVER = "gs_usb"


class A1ZError(RuntimeError):
    """The arm, or its SDK, refused or reports a problem; the message says what."""


class A1ZConnectionConfig(ConnectionConfig):
    """Where the A1Z is, and whether its gripper is on.

    Attributes:
        can_port: The CAN interface the arm is on, e.g. "a1zcan", which
            ``dimos hardware a1z configure-can`` makes. On macOS the HHS
            adapter is opened over USB, whatever the name.
        gripper: The G1Z gripper is on its flange. It is driven as the joint
            ``gripper``, the jaw opening in metres (0 closed, 0.1 open).
        gravity_comp_factor: How much of the arm's weight the SDK carries, 0 to 1.
        home: The joint angles the arm moves to after it starts and before it
            stops, in rad, one per joint. All zeros, the default, is the arm
            folded at rest. ``None`` leaves it where it is, so it drops on stop.
        jump_limit_s: How far a joint target may be from where the joint is,
            in seconds at the joint's top speed (its URDF velocity limit). A
            target further away is refused.
    """

    can_port: str
    gripper: bool = True
    source: str = "arm"
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
    gravity_comp_factor: float = Field(default=_DEVICE.gravity_comp_factor, ge=0.0, le=1.0)
    home: tuple[float, ...] | None = (0.0,) * A1Z_DOF
    jump_limit_s: float = 0.1

    @model_validator(mode="after")
    def _model_from_gripper(self) -> Self:
        if self.home is not None and len(self.home) != A1Z_DOF:
            raise ValueError(f"home gives {len(self.home)} joint angles; the A1Z has {A1Z_DOF}")
        if self.model is None:
            self.model = a1z_model(self.gripper)
        return self


class A1ZConnection(ConnectionModule):
    """An A1Z driven over CAN at ``config.can_port``, by joint targets only.

    Joint targets come in on ``position_command`` in rad; the gripper is the
    joint ``gripper``, its jaw opening in m. ``joint_state`` goes out about 100
    times a second, the gripper in a message of its own. At start the arm holds
    where it is, takes up its weight, then moves to ``config.home`` if set.
    When commands stop it holds where it is. Once the SDK stops the arm (a
    motor fault, its soft e-stop) it takes no command until ``clear_errors()``.
    Stopping moves it home, if set, and turns its motors off: the A1Z has no
    brakes, so it falls unless it rests on something or is held.
    """

    config: A1ZConnectionConfig

    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        prefix = f"{self.config.source}{SEPARATOR}"
        self._joints = [f"{prefix}arm_joint{i}" for i in range(1, A1Z_DOF + 1)]
        self._gripper = f"{prefix}gripper" if self.config.gripper else None
        self._robot: Any = None
        # Set when the SDK stops the arm or a motor reports a fault; only
        # clear_errors() clears it.
        self._fault: str | None = None
        self._gripper_sent: float | None = None
        # The joints' latest measured positions (rad), replaced whole by the
        # reading thread.
        self._measured: list[float] = []
        # time.monotonic() when _measured was read.
        self._measured_at = float("-inf")
        # How far each joint's target may be from where it is, in rad.
        self._reach: list[float] = []
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="A1ZConnection-read"
        )
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None:
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
        robot = self._robot
        if robot is not None:
            try:
                self._park()
            except Exception as error:
                logger.warning(f"A1ZConnection: the arm did not go home before stopping: {error}")
            self._robot = None
            self._close(robot)

    @rpc
    def clear_errors(self) -> None:
        """Let the arm take commands again once whatever stopped it is dealt with.

        Releases the SDK's soft e-stop, holding the arm where it is. If the
        SDK's control loop stopped, its motors are off: the arm is started
        again as at start-up, so rest it or hold it first.

        Raises:
            A1ZError: If a motor still reports a fault, or the start fails.
        """
        with self._hardware_lock:
            robot = self._robot
            if robot is None:
                raise A1ZError("not connected")
            if not robot.is_running:
                try:
                    self._safe_start()
                except Exception:
                    self._stop_after_failed_start(robot)
                    raise
            elif robot.is_estopped:
                robot.release()
                robot.command_joint_pos(self._read_joints()[0])
            problem = _motor_fault(robot.get_joint_state())
            if problem is not None:
                raise A1ZError(problem)
            self._gripper_sent = None
            self._fault = None

    def connect(self) -> None:
        model = self.config.model
        assert model is not None, "A1ZConnectionConfig fills in the model"
        prefix = f"{self.config.source}{SEPARATOR}"
        top_speeds = {
            f"{prefix}{joint.name}": model_limit(joint, Interface.VELOCITY).hi or 0.0
            for joint in movable_joints(model)
        }
        self._reach = [self.config.jump_limit_s * top_speeds[joint] for joint in self._joints]
        if self.config.home is not None:
            self._check_home(self.config.home)
        robot = _open_sdk(
            self.config.can_port,
            gripper=self.config.gripper,
            gravity_comp_factor=self.config.gravity_comp_factor,
        )
        self._robot = robot
        try:
            self._safe_start()
            if self.config.home is not None:
                self._go_home(self.config.home, _HOME_SPEED_RAD_S)
            self._measured = self._read_joints()[0].tolist()
            self._measured_at = time.monotonic()
        except Exception:
            self._robot = None
            self._stop_after_failed_start(robot)
            self._close(robot)
            raise

    def write(self, values: Values) -> None:
        self._check_running()
        targets = [values.get(f"{joint}/{Interface.POSITION}") for joint in self._joints]
        gripper = None
        if self._gripper is not None:
            gripper = values.get(f"{self._gripper}/{Interface.POSITION}")
        if any(target is not None for target in targets):
            # Joints left out, and the jump guard, go by where the joints were last read.
            age = time.monotonic() - self._measured_at
            if age > _READING_MAX_AGE_S:
                raise A1ZError(
                    f"the joints were last read {age:.2f} s ago; joint targets need a "
                    f"reading at most {_READING_MAX_AGE_S} s old"
                )
            # A joint left out stays where it is.
            measured = self._measured
            angles = [m if t is None else t for t, m in zip(targets, measured, strict=True)]
            self._check_jump(angles, measured)
            self._robot.command_joint_pos(np.asarray(angles, dtype=float))
        if gripper is not None and gripper != self._gripper_sent:
            # The SDK takes the opening as a fraction: 0 closed, 1 open.
            self._robot.command_gripper(min(1.0, max(0.0, gripper / A1Z_GRIPPER_OPENING_M)))
            self._gripper_sent = gripper

    def halt(self) -> None:
        """Hold the arm where it is now, unless the SDK has already stopped it."""
        robot = self._robot
        if robot is None or self._fault is not None or not robot.is_running or robot.is_estopped:
            return
        robot.command_joint_pos(self._read_joints()[0])

    def _check_running(self) -> None:
        """Raise, and latch a fault, unless the SDK's control loop runs out of its soft e-stop."""
        problem = _sdk_problem(self._robot)
        if problem is not None:
            self._latch(problem)
        if self._fault is not None:
            raise A1ZError(self._fault)

    def _latch(self, problem: str) -> None:
        if self._fault is None:
            self._fault = problem
            self._report(problem)

    def _check_home(self, home: tuple[float, ...]) -> None:
        """Refuse a home (rad) outside the model's joint limits, before the arm is touched."""
        for joint, angle in zip(self._joints, home, strict=True):
            limit = self.limits[f"{joint}/{Interface.POSITION}"]
            low = -math.inf if limit.lo is None else limit.lo
            high = math.inf if limit.hi is None else limit.hi
            if not low <= angle <= high:
                raise ValueError(
                    f"home puts {joint} at {angle} rad, outside its limits {limit.lo} to {limit.hi}"
                )

    def _check_jump(self, angles: list[float], measured: list[float]) -> None:
        """Refuse joint targets (rad) too far from where the joints are now (rad)."""
        for joint, target, now, reach in zip(
            self._joints, angles, measured, self._reach, strict=True
        ):
            if abs(target - now) > reach:
                raise A1ZError(
                    f"{joint} told {target:.3f} rad, {abs(target - now):.3f} rad from where it "
                    f"is; at most {reach:.3f} rad away is taken"
                )

    def _safe_start(self) -> None:
        """Start the SDK's control loop so the arm takes its own weight without moving.

        The loop starts with no stiffness and no gravity compensation, since
        the SDK's gravity term pushes even at zero stiffness. Once every motor
        reports and the arm is still and within its limits, it holds where it
        is at full stiffness, gravity compensation ramps in, and the arm must
        settle. Raises if any check fails; the caller turns the motors off.
        """
        robot = self._robot
        info = robot.get_robot_info()
        kp = np.asarray(info["default_kp"], dtype=float)
        kd = np.asarray(info["default_kd"], dtype=float)
        robot.gravity_comp_factor = 0.0
        robot.start(initial_kp=np.zeros(A1Z_DOF), initial_kd=kd * 0.5)
        # A motor's first report can be late; the SDK would then hold zero, not where it is.
        if not _wait_for(self._all_motors_reported, _STARTUP_FEEDBACK_TIMEOUT_S):
            raise A1ZError(
                f"not every motor reported within {_STARTUP_FEEDBACK_TIMEOUT_S} s of starting"
            )
        hold, _ = self._startup_state("the first reading")
        # Holding exactly where it is asks for no motion, whatever the stiffness.
        robot.command_joint_state(
            {"pos": hold.copy(), "vel": np.zeros(A1Z_DOF), "kp": kp, "kd": kd}
        )
        steps = max(1, round(_STARTUP_RAMP_DURATION_S / _STARTUP_SAMPLE_PERIOD_S))
        factors = [0.0] * _STARTUP_STAGED_SAMPLES + [
            self.config.gravity_comp_factor * step / steps for step in range(1, steps + 1)
        ]
        for sample, factor in enumerate(factors, start=1):
            robot.gravity_comp_factor = factor
            time.sleep(_STARTUP_SAMPLE_PERIOD_S)
            self._startup_state(f"gravity ramp {sample}/{len(factors)}")
        samples = max(
            _STARTUP_SETTLED_SAMPLES, round(_STARTUP_SETTLING_TIMEOUT_S / _STARTUP_SAMPLE_PERIOD_S)
        )
        still = 0
        position, velocity = hold, np.zeros(A1Z_DOF)
        for sample in range(1, samples + 1):
            time.sleep(_STARTUP_SAMPLE_PERIOD_S)
            position, velocity = self._startup_state(f"settling {sample}/{samples}")
            # One reading just over the threshold is encoder noise; motion lasts.
            still = still + 1 if np.all(np.abs(velocity) <= _STARTUP_SETTLED_VELOCITY_RAD_S) else 0
            if still >= _STARTUP_SETTLED_SAMPLES:
                return
        raise A1ZError(
            f"the arm did not settle within {_STARTUP_SETTLING_TIMEOUT_S} s: positions "
            f"{np.round(position, 3).tolist()} rad, speeds {np.round(velocity, 3).tolist()} rad/s"
        )

    def _startup_state(self, phase: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """The arm's positions (rad) and speeds (rad/s) while starting.

        Raises:
            A1ZError: If the control loop stopped, a reading is malformed, a
                joint is past its limits or one moves too fast.
        """
        if not self._robot.is_running:
            raise A1ZError(f"the SDK's control loop stopped during {phase}")
        state = self._robot.get_joint_state()
        position = np.asarray(state["pos"], dtype=float)
        velocity = np.asarray(state["vel"], dtype=float)
        if position.shape != (A1Z_DOF,) or velocity.shape != (A1Z_DOF,):
            raise A1ZError(f"a malformed reading during {phase}: {position}, {velocity}")
        if not (np.all(np.isfinite(position)) and np.all(np.isfinite(velocity))):
            raise A1ZError(f"a non-finite reading during {phase}: {position}, {velocity}")
        for joint, angle, speed in zip(self._joints, position, velocity, strict=True):
            limit = self.limits[f"{joint}/{Interface.POSITION}"]
            low = (-math.inf if limit.lo is None else limit.lo) - _STARTUP_LIMIT_MARGIN_RAD
            high = (math.inf if limit.hi is None else limit.hi) + _STARTUP_LIMIT_MARGIN_RAD
            if not low <= angle <= high:
                raise A1ZError(f"{joint} is at {angle:.3f} rad, past its limits, during {phase}")
            if abs(speed) > _STARTUP_MAX_VELOCITY_RAD_S:
                raise A1ZError(
                    f"{joint} moves at {speed:.3f} rad/s during {phase}; at most "
                    f"{_STARTUP_MAX_VELOCITY_RAD_S} rad/s is taken"
                )
        return position, velocity

    def _go_home(self, home: tuple[float, ...], speed: float) -> None:
        """Move the arm to ``home`` (rad) at ``speed`` (rad/s on average), and wait until it is there.

        Raises:
            A1ZError: If the arm is not there in twice the time the move should take.
        """
        start = self._read_joints()[0]
        goal = np.asarray(home, dtype=float)
        self._robot.move_joints(goal, speed=speed)
        timeout = 2 * float(np.max(np.abs(goal - start))) / speed + 1.0
        if not _wait_for(
            lambda: bool(np.all(np.abs(self._read_joints()[0] - goal) <= _HOME_TOLERANCE_RAD)),
            timeout,
        ):
            at = np.round(self._read_joints()[0], 3).tolist()
            raise A1ZError(f"the arm did not reach home within {timeout:.1f} s; it is at {at} rad")

    def _park(self) -> None:
        """Move the arm home before its motors turn off, unless it has a fault,
        is stopped by the SDK or has no home."""
        robot = self._robot
        if (
            self.config.home is None
            or self._fault is not None
            or not robot.is_running
            or robot.is_estopped
        ):
            return
        self._go_home(self.config.home, _PARK_SPEED_RAD_S)

    def _stop_after_failed_start(self, robot: Any) -> None:
        """Take away gravity compensation, then put the arm in its soft e-stop
        and stop the SDK's loop, which turns its motors off."""
        robot.gravity_comp_factor = 0.0
        try:
            robot.estop()
        except Exception as error:
            logger.warning(f"A1ZConnection: no soft e-stop after the failed start: {error}")
        try:
            robot.stop()
        except Exception:
            self._disable_motors(robot)

    def _close(self, robot: Any) -> None:
        """Stop the SDK's loop, turn every motor off and let go of the CAN bus."""
        try:
            if robot.is_running:
                robot.stop()
        except Exception as error:
            logger.warning(f"A1ZConnection: the SDK did not stop: {error}")
        self._disable_motors(robot)
        try:
            # The SDK leaves its bus open; shut it so the CAN port can be opened again.
            robot._bus.shutdown()
        except Exception as error:
            logger.warning(f"A1ZConnection: the CAN bus did not shut down: {error}")

    def _disable_motors(self, robot: Any) -> None:
        """Send every motor, gripper included, its off command twice.

        The SDK sends the gripper's only once, and a busy bus can lose it,
        leaving the gripper powered.
        """
        motors = [*robot._motor_chain._motor_a_list, *robot._motor_chain._motor_b_list]
        if self._gripper is not None:
            motors.append(robot.gripper._motor)
        for _ in range(2):
            for motor in motors:
                try:
                    motor.disable()
                except Exception:
                    pass

    def _all_motors_reported(self) -> bool:
        """Every arm motor has sent a reading since the SDK opened the bus."""
        chain = self._robot._motor_chain
        return all(
            motor.last_feedback is not None
            for motor in (*chain._motor_a_list, *chain._motor_b_list)
        )

    def _read_joints(
        self,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], dict[str, Any]]:
        """The arm joints' positions (rad), speeds (rad/s) and torques (N·m), and the SDK's whole reading."""
        state = self._robot.get_joint_state()
        return (
            np.asarray(state["pos"], dtype=float)[:A1Z_DOF],
            np.asarray(state["vel"], dtype=float)[:A1Z_DOF],
            np.asarray(state["eff"], dtype=float)[:A1Z_DOF],
            state,
        )

    def _read_loop(self) -> None:
        period = 1.0 / _READ_HZ
        due = time.monotonic()
        count = 0
        while not self._stop_reading.wait(max(0.0, due - time.monotonic())):
            due = max(due + period, time.monotonic())
            try:
                positions, speeds, efforts, state = self._read_joints()
                problem = _sdk_problem(self._robot) or _motor_fault(state)
                if problem is not None:
                    self._latch(problem)
                self._measured = positions.tolist()
                self._measured_at = time.monotonic()
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        name=list(self._joints),
                        position=positions.tolist(),
                        velocity=speeds.tolist(),
                        effort=efforts.tolist(),
                    )
                )
                if self._gripper is not None and count % _GRIPPER_READ_EVERY == 0:
                    fraction = self._robot.gripper.get_feedback_norm()
                    if fraction is not None:
                        # The gripper reports no speed or torque, so it has a message of its own.
                        self.joint_state.publish(
                            JointState(
                                ts=time.time(),
                                name=[self._gripper],
                                position=[float(fraction) * A1Z_GRIPPER_OPENING_M],
                            )
                        )
            except Exception as error:
                self._report(f"reading failed: {type(error).__name__}: {error}")
            count += 1


def _open_sdk(port: str, *, gripper: bool, gravity_comp_factor: float) -> Any:
    """Open the A1Z on CAN port ``port`` with Galaxea's SDK; its motors stay off.

    Raises:
        ConnectionError: If the port is not the HHS adapter, up, or the SDK cannot open it.
    """
    # Imported here so blueprints that use this module load without the SDK installed.
    from a1z.robots.get_robot import get_a1z_robot

    darwin = platform.system() == "Darwin"
    if not darwin:
        problem = _socketcan_problem(port)
        if problem is not None:
            raise ConnectionError(problem)
    try:
        if darwin:
            # macOS has no SocketCAN: the SDK's bus goes over USB instead.
            with gs_usb_can_bus():
                return _make_robot(get_a1z_robot, port, gripper, gravity_comp_factor)
        return _make_robot(get_a1z_robot, port, gripper, gravity_comp_factor)
    except TypeError as error:
        raise ConnectionError(
            "the installed a1z SDK cannot drive the gripper; run "
            f"`dimos hardware a1z doctor --software-only`: {error}"
        ) from error
    except Exception as error:
        raise ConnectionError(f"cannot open the A1Z on CAN port {port}: {error}") from error


def _make_robot(
    get_a1z_robot: Callable[..., Any], port: str, gripper: bool, gravity_comp_factor: float
) -> Any:
    """Build the SDK's robot, with Galaxea's gains and the SDK's own dynamics model."""
    assert _DEVICE.gripper is not None
    return get_a1z_robot(
        can_channel=port,
        gravity_comp_factor=gravity_comp_factor,
        zero_gravity_mode=False,
        control_freq_hz=_SDK_CONTROL_HZ,
        urdf_path=None,
        default_kp=np.asarray(_DEVICE.default_kp, dtype=float),
        default_kd=np.asarray(_DEVICE.default_kd, dtype=float),
        with_gripper=gripper,
        gripper_max_torque=_DEVICE.gripper.max_torque,
    )


def _socketcan_problem(port: str) -> str | None:
    """Why SocketCAN interface ``port`` is not the HHS adapter, up, or ``None`` if it is."""
    interface = _SYS_CLASS_NET / port
    try:
        flags = int((interface / "flags").read_text(), 16)
    except FileNotFoundError:
        return (
            f"there is no SocketCAN interface {port!r}; run `dimos hardware a1z configure-can`, "
            "or name the HHS adapter's interface"
        )
    except (OSError, ValueError) as error:
        return f"cannot read SocketCAN interface {port!r}: {error}"
    try:
        driver = (interface / "device" / "driver").resolve(strict=True).name
    except OSError as error:
        return f"cannot tell which driver SocketCAN interface {port!r} belongs to: {error}"
    if driver != _HHS_DRIVER:
        return (
            f"SocketCAN interface {port!r} belongs to driver {driver!r}, not the HHS "
            f"adapter's {_HHS_DRIVER!r}; name the HHS adapter's interface"
        )
    if not flags & 0x1:
        return f"SocketCAN interface {port!r} is down; run `dimos hardware a1z configure-can`"
    return None


def _sdk_problem(robot: Any) -> str | None:
    """Why the SDK takes no command (its loop stopped, its soft e-stop), or ``None``."""
    if not robot.is_running:
        return (
            "the SDK's control loop stopped and the motors are off; support the arm, "
            "then call clear_errors()"
        )
    if robot.is_estopped:
        return "the arm is in its soft e-stop; call clear_errors() once it is safe"
    return None


def _motor_fault(state: dict[str, Any]) -> str | None:
    """Which motor reports a fault in an SDK reading, or ``None``."""
    for joint, code in enumerate(state["error_codes"], start=1):
        if int(code) not in _HEALTHY_MOTOR_CODES:
            return (
                f"the motor of joint {joint} reports fault 0x{int(code):x}; clear it with the "
                "SDK's tools/motor_diag.py --clear-error, then call clear_errors()"
            )
    return None


def _wait_for(done: Callable[[], bool], timeout_s: float) -> bool:
    """Ask ``done`` until it says yes, for at most ``timeout_s`` seconds; whether it did."""
    deadline = time.monotonic() + timeout_s
    while not done():
        if time.monotonic() > deadline:
            return False
        time.sleep(_POLL_S)
    return True
