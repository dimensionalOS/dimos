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

"""An AgileX Piper arm as a connection module, over CAN with AgileX's piper_sdk."""

from __future__ import annotations

from collections.abc import Callable
import math
import threading
import time
from typing import Any

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
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.piper.config import PIPER_DOF, piper_model

# The SDK's units: joint angles in 0.001 degrees, the gripper's gap in 0.001 mm,
# and motor speeds and torques in 0.001 rad/s and 0.001 N·m.
_SDK_PER_RAD = 180_000.0 / math.pi
_SDK_PER_M = 1_000_000.0
_SDK_PER_UNIT = 1000.0

# The arm's control modes: standby (as it powers up) and taking CAN commands.
_STANDBY = 0x00
_CAN_CONTROL = 0x01
# The arm's status when nothing is wrong.
_NORMAL = 0x00
# MotionCtrl_2's move mode for joint targets (MOVE J), and its MIT mode left off.
_MOVE_J = 0x01
_MIT_OFF = 0x00
# MotionCtrl_1's reset: clears the arm's errors and turns its motors off.
_RESET = 0x02
# JointConfig: every joint motor, and the flag that clears their errors.
_ALL_MOTORS = 7
_CLEAR_ERRORS = 0xAE
# GripperCtrl's codes: on, and off with its errors cleared.
_GRIPPER_ON = 0x01
_GRIPPER_OFF_CLEARED = 0x02
# How hard the gripper squeezes, in 0.001 N·m.
_GRIPPER_EFFORT = 1000

_READ_HZ = 100.0
# Joint targets are refused once the latest joint reading is older than this,
# in seconds: ten missed readings.
_READING_MAX_AGE_S = 10 / _READ_HZ
# The gripper is read on every 10th reading of the joints.
_GRIPPER_READ_EVERY = 10
# How long the arm has to answer, turn its motors on or clear its errors, in seconds.
_ANSWER_TIMEOUT_S = 2.0
_POLL_S = 0.005
# How long the arm takes to reset before its motors can be turned on, in seconds.
_RESET_SETTLE_S = 0.5


class PiperError(RuntimeError):
    """The arm refused, or reports a problem; the message says what and why."""


class PiperConfig(ConnectionConfig):
    """Where the Piper is, and whether its gripper is on.

    Attributes:
        can_port: The CAN interface the arm is on, e.g. "can0", already up at
            1 Mbit/s.
        gripper: The gripper is on its flange. It is driven as its URDF joint
            ``gripper``, the gap between the fingers in metres.
        speed_percent: How fast the arm moves to each joint target, in percent
            of its top speed.
        jump_limit_s: How far a joint target may be from where the joint is,
            in seconds at the joint's top speed (its URDF velocity limit). A
            target further away is refused.
    """

    can_port: str
    gripper: bool = False
    source: str = "arm"
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
    speed_percent: int = Field(default=50, ge=1, le=100)
    jump_limit_s: float = 0.1

    @model_validator(mode="after")
    def _model_from_gripper(self) -> Self:
        if self.model is None:
            self.model = piper_model(self.gripper)
        return self


class PiperConnection(ConnectionModule):
    """A Piper driven over CAN at ``config.can_port``, by joint targets only.

    Joint targets come in on ``position_command`` in rad; the gripper is its
    URDF joint ``gripper`` (0 m closed, 0.07 m open). ``joint_state`` goes out
    about 100 times a second, the gripper in a message of its own. Once the
    arm reports a problem, or stops taking commands without DimOS asking
    (teach mode, a power cut, another program), it takes no command until
    ``clear_errors()``. Joint targets are refused while the latest joint
    reading is over 0.1 s old. Stopping leaves the arm holding where it is.
    """

    config: PiperConfig

    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        prefix = f"{self.config.source}{SEPARATOR}"
        self._joints = [f"{prefix}joint{i}" for i in range(1, PIPER_DOF + 1)]
        self._gripper = f"{prefix}gripper" if self.config.gripper else None
        self._sdk: Any = None
        # Set when the arm reports a problem or stops taking commands; only
        # clear_errors() clears it.
        self._fault: str | None = None
        # The arm was last seen taking CAN commands with its motors on. Only
        # the reading thread touches it.
        self._ready = False
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
            target=self._read_loop, daemon=True, name="PiperConnection-read"
        )
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None:
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
        sdk, self._sdk = self._sdk, None
        if sdk is not None:
            # The motors stay on, so the arm holds where it is instead of falling.
            sdk.DisconnectPort()

    @rpc
    def clear_errors(self) -> None:
        """Clear the arm's errors, turn its motors on, and let it take commands again.

        Call it once whatever stopped the arm is dealt with. An arm in an
        emergency stop, an error or teach mode is reset first, which turns its
        motors off for half a second: rest the arm or hold it.

        Raises:
            PiperError: If the arm still reports a problem afterwards.
        """
        with self._hardware_lock:
            sdk = self._sdk
            if sdk is None:
                raise PiperError("not connected")
            if _problem(sdk.GetArmStatus().arm_status) is not None:
                sdk.MotionCtrl_1(_RESET, 0, 0)
                time.sleep(_RESET_SETTLE_S)
            sdk.JointConfig(joint_num=_ALL_MOTORS, clear_err=_CLEAR_ERRORS)
            self._enable()
            heard = sdk.GetArmStatus().time_stamp

            def taking_commands() -> bool:
                report = sdk.GetArmStatus()
                state = report.arm_status
                return (
                    report.time_stamp != heard
                    and _problem(state) is None
                    and state.ctrl_mode == _CAN_CONTROL
                )

            if not _wait_for(taking_commands):
                state = sdk.GetArmStatus().arm_status
                raise PiperError(
                    f"the arm takes no commands {_ANSWER_TIMEOUT_S} s on: it reports "
                    f"{state.arm_status!s} in control mode {state.ctrl_mode!s}"
                )
            self._fault = None

    def connect(self) -> None:
        model = self.config.model
        assert model is not None, "PiperConfig fills in the model"
        prefix = f"{self.config.source}{SEPARATOR}"
        top_speeds = {
            f"{prefix}{joint.name}": model_limit(joint, Interface.VELOCITY).hi or 0.0
            for joint in movable_joints(model)
        }
        self._reach = [self.config.jump_limit_s * top_speeds[joint] for joint in self._joints]
        port = self.config.can_port
        try:
            sdk = _open_sdk(port)
        except ConnectionError as error:
            # The SDK's own message only names the port; the reason is the error it caught.
            raise ConnectionError(
                f"cannot open CAN port {port}: {error.__context__ or error}"
            ) from error
        try:
            self._sdk = sdk
            heard = self._report_times()
            if not _wait_for(
                lambda: all(
                    now != then for now, then in zip(self._report_times(), heard, strict=True)
                )
            ):
                raise ConnectionError(f"no Piper answers on CAN port {port}")
            self._measured = self._read_joints()[1]
            self._measured_at = time.monotonic()
            problem = _problem(sdk.GetArmStatus().arm_status)
            if problem is not None:
                self._fault = problem
                self._report(problem)
            else:
                self._enable()
        except Exception:
            self._sdk = None
            sdk.DisconnectPort()
            raise

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise PiperError(self._fault)
        targets = [values.get(f"{joint}/{Interface.POSITION}") for joint in self._joints]
        gripper = None
        if self._gripper is not None:
            gripper = values.get(f"{self._gripper}/{Interface.POSITION}")
        if any(target is not None for target in targets):
            # Joints left out, and the jump guard, go by where the joints were last read.
            age = time.monotonic() - self._measured_at
            if age > _READING_MAX_AGE_S:
                raise PiperError(
                    f"the joints were last read {age:.2f} s ago; joint targets need a "
                    f"reading at most {_READING_MAX_AGE_S} s old"
                )
            # A joint left out stays where it is.
            measured = self._measured
            angles = [m if t is None else t for t, m in zip(targets, measured, strict=True)]
            self._check_jump(angles, measured)
            self._sdk.JointCtrl(*(round(angle * _SDK_PER_RAD) for angle in angles))
        if gripper is not None and gripper != self._gripper_sent:
            self._sdk.GripperCtrl(round(gripper * _SDK_PER_M), _GRIPPER_EFFORT, _GRIPPER_ON, 0)
            self._gripper_sent = gripper

    def halt(self) -> None:
        """Nothing to send: the arm finishes its last joint target, which the
        jump guard keeps close, and holds there."""

    def _enable(self) -> None:
        """Turn the motors on and make the arm take joint targets, leaving the gripper where it is."""
        if not _wait_for(self._sdk.EnablePiper):
            raise PiperError(f"the arm's motors did not turn on within {_ANSWER_TIMEOUT_S} s")
        self._sdk.MotionCtrl_2(_CAN_CONTROL, _MOVE_J, self.config.speed_percent, _MIT_OFF)
        if self._gripper is not None:
            gap = max(0, round(self._read_gripper()[1] * _SDK_PER_M))
            self._sdk.GripperCtrl(gap, _GRIPPER_EFFORT, _GRIPPER_OFF_CLEARED, 0)
            self._sdk.GripperCtrl(gap, _GRIPPER_EFFORT, _GRIPPER_ON, 0)
            self._gripper_sent = None

    def _check_jump(self, angles: list[float], measured: list[float]) -> None:
        """Refuse joint targets (rad) too far from where the joints are now (rad)."""
        for joint, target, now, reach in zip(
            self._joints, angles, measured, self._reach, strict=True
        ):
            if abs(target - now) > reach:
                raise PiperError(
                    f"{joint} told {target:.3f} rad, {abs(target - now):.3f} rad from where it "
                    f"is; at most {reach:.3f} rad away is taken"
                )

    def _watch_status(self) -> None:
        """Latch a fault once the arm reports a problem, or stops taking
        commands (out of CAN control, or its motors off) without DimOS asking."""
        state = self._sdk.GetArmStatus().arm_status
        ready = state.ctrl_mode == _CAN_CONTROL and all(self._sdk.GetArmEnableStatus())
        problem = _problem(state)
        if problem is None and self._ready and not ready:
            problem = "the arm was stopped outside DimOS; call clear_errors() once it is safe"
        self._ready = ready
        if problem is not None and self._fault is None:
            self._fault = problem
            self._report(problem)

    def _report_times(self) -> list[float]:
        """When the joints, the status and the gripper (if on) last reported:
        their latest CAN frames' times."""
        times = [self._sdk.GetArmJointMsgs().time_stamp, self._sdk.GetArmStatus().time_stamp]
        if self._gripper is not None:
            times.append(self._sdk.GetArmGripperMsgs().time_stamp)
        return times

    def _read_joints(self) -> tuple[float, list[float]]:
        """When the joints last reported (the CAN frame's time), and their positions (rad)."""
        report = self._sdk.GetArmJointMsgs()
        state = report.joint_state
        positions = [getattr(state, f"joint_{i}") / _SDK_PER_RAD for i in range(1, PIPER_DOF + 1)]
        return report.time_stamp, positions

    def _read_motors(self) -> tuple[list[float], list[float]]:
        """The joint motors' speeds (rad/s) and torques (N·m)."""
        report = self._sdk.GetArmHighSpdInfoMsgs()
        motors = [getattr(report, f"motor_{i}") for i in range(1, PIPER_DOF + 1)]
        return (
            [motor.motor_speed / _SDK_PER_UNIT for motor in motors],
            [motor.effort / _SDK_PER_UNIT for motor in motors],
        )

    def _read_gripper(self) -> tuple[float, float]:
        """When the gripper last reported (the CAN frame's time), and its gap (m)."""
        report = self._sdk.GetArmGripperMsgs()
        return report.time_stamp, report.gripper_state.grippers_angle / _SDK_PER_M

    def _read_loop(self) -> None:
        period = 1.0 / _READ_HZ
        due = time.monotonic()
        count = 0
        # The CAN frame times of the last joint and gripper readings published:
        # one that has not changed is not new, and is not published again.
        joints_heard: float | None = None
        gripper_heard: float | None = None
        while not self._stop_reading.wait(max(0.0, due - time.monotonic())):
            due = max(due + period, time.monotonic())
            try:
                self._watch_status()
                heard, positions = self._read_joints()
                if heard != joints_heard:
                    joints_heard = heard
                    self._measured = positions
                    self._measured_at = time.monotonic()
                    speeds, efforts = self._read_motors()
                    self.joint_state.publish(
                        JointState(
                            ts=time.time(),
                            name=list(self._joints),
                            position=positions,
                            velocity=speeds,
                            effort=efforts,
                        )
                    )
                if self._gripper is not None and count % _GRIPPER_READ_EVERY == 0:
                    heard, gap = self._read_gripper()
                    if heard != gripper_heard:
                        gripper_heard = heard
                        # The gripper reports no speed, so it has a message of its own.
                        self.joint_state.publish(
                            JointState(ts=time.time(), name=[self._gripper], position=[gap])
                        )
            except Exception as error:
                self._report(f"reading failed: {type(error).__name__}: {error}")
            count += 1


def _open_sdk(port: str) -> Any:
    """Open CAN port ``port`` with the Piper SDK and start its reading thread."""
    # Imported here so blueprints that use this module load without the SDK installed.
    from piper_sdk import C_PiperInterface_V2

    sdk = C_PiperInterface_V2(can_name=port)
    sdk.ConnectPort()
    return sdk


def _problem(state: Any) -> str | None:
    """What stops the arm taking commands, from its status report, or ``None``."""
    if state.arm_status != _NORMAL:
        return f"the arm reports {state.arm_status!s}; call clear_errors() once it is safe"
    if state.ctrl_mode not in (_STANDBY, _CAN_CONTROL):
        return (
            f"the arm is in control mode {state.ctrl_mode!s}, not taking CAN commands; "
            "call clear_errors() once it is safe"
        )
    return None


def _wait_for(done: Callable[[], bool]) -> bool:
    """Ask ``done`` until it says yes, for at most ``_ANSWER_TIMEOUT_S``; whether it did."""
    deadline = time.monotonic() + _ANSWER_TIMEOUT_S
    while not done():
        if time.monotonic() > deadline:
            return False
        time.sleep(_POLL_S)
    return True
