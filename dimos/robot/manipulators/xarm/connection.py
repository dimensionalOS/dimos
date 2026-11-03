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

"""A UFACTORY arm (xArm 5, 6 or 7, Lite 6, 850) as a connection module, over its Python SDK."""

from __future__ import annotations

from collections.abc import Callable
import threading
import time
from typing import Any

from pydantic import model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self
from xarm.wrapper import XArmAPI

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.config import XARM_ARMS, XArmGripper, XArmType, xarm_model
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The SDK's modes: stream joint targets (set_servo_angle_j), or joint speeds
# (vc_set_joint_velocity). And its states: ready to move, stopped.
_SERVO = 1
_VELOCITY = 4
_READY = 0
_STOPPED = 4

# What a motion command returns while the arm is stopped (state 4 or 5).
_NOT_READY = 9

#: The oldest firmware that reports joint speeds (``get_joint_states``).
MIN_FIRMWARE = (1, 9, 0)

# The xArm Gripper's drive_joint is open at 0 rad and closed at 0.85 rad; the
# SDK counts 850 open down to 0 closed.
_GRIPPER_CLOSED_RAD = 0.85
_GRIPPER_OPEN_SDK = 850.0

_READ_HZ = 100.0
# Joint targets are refused once the latest joint reading is older than this,
# in seconds: ten missed readings.
_READING_MAX_AGE_S = 10 / _READ_HZ
# The gripper is read on every 10th reading of the joints.
_GRIPPER_READ_EVERY = 10


class XArmError(RuntimeError):
    """The arm, or its SDK, refused; the message says what and why."""


class XArmConfig(ConnectionConfig):
    """Which xArm, where it is, and what is on its flange.

    Attributes:
        arm: Which arm, e.g. "xarm7". Its URDF is the robot model.
        ip: The arm's address, e.g. "192.168.1.185".
        gripper: The gripper on its flange, if any. Only "xarm", the xArm
            Gripper, is driven so far.
        jump_limit_s: How far a joint target may be from where the joint is,
            in seconds at the joint's top speed (its URDF velocity limit). A
            target further away is refused.
    """

    arm: XArmType
    ip: str
    gripper: XArmGripper | None = None
    source: str = "arm"
    joint_commands: tuple[Interface, ...] = (Interface.POSITION, Interface.VELOCITY)
    jump_limit_s: float = 0.1

    @model_validator(mode="after")
    def _model_from_arm(self) -> Self:
        if self.gripper == "bio":
            raise ValueError("XArmConnection drives only the xArm Gripper so far, not the BIO")
        if self.gripper is not None and self.arm == "lite6":
            raise ValueError("the Lite 6 gripper has no joint, so XArmConnection cannot drive it")
        if self.model is None:
            self.model = xarm_model(self.arm, self.gripper)
        return self


class XArmConnection(ConnectionModule):
    """A UFACTORY arm driven over its SDK at ``config.ip``.

    Joint targets come in on ``position_command`` and joint speeds on
    ``velocity_command``, in rad and rad/s; the gripper is its URDF joint
    ``drive_joint`` (0 rad open, 0.85 rad closed). ``joint_state`` goes out
    about 100 times a second, the gripper in a message of its own. Once the
    arm reports an error, or is stopped from outside DimOS (xArm Studio, its
    stop button), it takes no command until ``clear_errors()``. Joint targets
    are refused while the latest joint reading is over 0.1 s old.
    """

    config: XArmConfig

    position_command: In[JointState]
    velocity_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        prefix = f"{self.config.source}{SEPARATOR}"
        dof = XARM_ARMS[self.config.arm][0]
        self._joints = [f"{prefix}joint{i}" for i in range(1, dof + 1)]
        self._gripper = f"{prefix}drive_joint" if self.config.gripper else None
        self._sdk: XArmAPI | None = None
        self._mode: int | None = None
        # Set when the arm reports an error or is stopped; only clear_errors()
        # clears it.
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

    @property
    def _connected_sdk(self) -> XArmAPI:
        if self._sdk is None:
            raise XArmError("not connected")
        return self._sdk

    @rpc
    def start(self) -> None:
        super().start()
        self._listen(self.position_command, Interface.POSITION)
        self._listen(self.velocity_command, Interface.VELOCITY)
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="XArmConnection-read"
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
            code = sdk.set_state(_STOPPED)
            if code != 0:
                logger.warning(f"XArmConnection: set_state(stopped) returned code {code}")
            sdk.disconnect()

    @rpc
    def clear_errors(self) -> None:
        """Clear the arm's error and warning, and let it take commands again.

        Call it once whatever stopped the arm, such as a collision, a joint
        past its limit or a stop from xArm Studio, is dealt with.

        Raises:
            XArmError: If the arm refuses.
        """
        with self._hardware_lock:
            if self._sdk is None:
                raise XArmError("not connected")
            # The arm refuses to clear a warning while an error is still set.
            _check("clean_error", self._sdk.clean_error())
            _check("clean_warn", self._sdk.clean_warn())
            self._enable()
            self._fault = None

    def connect(self) -> None:
        self._reach = [
            self.config.jump_limit_s * (self.limits[f"{joint}/{Interface.VELOCITY}"].hi or 0.0)
            for joint in self._joints
        ]
        ip = self.config.ip
        try:
            sdk = _open_sdk(ip)
        except Exception as error:
            raise ConnectionError(f"no xArm answers at {ip}: {error}") from error
        if not sdk.connected:
            raise ConnectionError(f"no xArm answers at {ip}")
        try:
            found = _arm_type(sdk.axis, sdk.device_type)
            if found != self.config.arm:
                raise XArmError(
                    f"the config says {self.config.arm}, but the arm at {ip} is an {found} "
                    f"({sdk.axis} axes, device type {sdk.device_type})"
                )
            firmware = tuple(sdk.version_number)
            if firmware < MIN_FIRMWARE:
                raise XArmError(
                    f"the arm at {ip} runs firmware {'.'.join(map(str, firmware))}; "
                    f"XArmConnection needs {'.'.join(map(str, MIN_FIRMWARE))} or newer"
                )
            self._sdk = sdk
            self._measured = self._read_joints()[0]
            self._measured_at = time.monotonic()
            sdk.register_error_warn_changed_callback(self._on_error_warn)
            if sdk.error_code:
                self._on_error_warn({"error_code": sdk.error_code, "warn_code": sdk.warn_code})
            else:
                self._enable()
        except Exception:
            self._sdk = None
            sdk.disconnect()
            raise

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise XArmError(self._fault)
        speeds = [values.get(f"{joint}/{Interface.VELOCITY}") for joint in self._joints]
        targets = [values.get(f"{joint}/{Interface.POSITION}") for joint in self._joints]
        gripper = None
        if self._gripper is not None:
            gripper = values.get(f"{self._gripper}/{Interface.POSITION}")
            # A coordinator jogging the arm by speed tells the gripper 0 too.
            if values.get(f"{self._gripper}/{Interface.VELOCITY}", 0.0) != 0.0:
                raise XArmError(f"{self._gripper} takes a position, not a speed")
        jogging = any(speed is not None for speed in speeds)
        angles = None
        if any(target is not None for target in targets):
            if jogging:
                raise XArmError("a command gives the arm positions or speeds, not both")
            # Joints left out, and the jump guard, go by where the joints were last read.
            age = time.monotonic() - self._measured_at
            if age > _READING_MAX_AGE_S:
                raise XArmError(
                    f"the joints were last read {age:.2f} s ago; joint targets need a "
                    f"reading at most {_READING_MAX_AGE_S} s old"
                )
            # A joint left out stays where it is.
            measured = self._measured
            angles = [m if t is None else t for t, m in zip(targets, measured, strict=True)]
            self._check_jump(angles, measured)

        if jogging:
            self._use_mode(_VELOCITY)
            self._send_motion(
                "vc_set_joint_velocity",
                lambda: self._connected_sdk.vc_set_joint_velocity(
                    [speed or 0.0 for speed in speeds],
                    is_radian=True,
                    # The arm stops by itself if no newer speed arrives in time.
                    duration=2 * self.config.deadman_timeout_s,
                ),
            )
        elif angles is not None:
            self._use_mode(_SERVO)
            self._send_motion(
                "set_servo_angle_j",
                lambda: self._connected_sdk.set_servo_angle_j(angles, is_radian=True),
            )
        if gripper is not None and gripper != self._gripper_sent:
            pulses = _GRIPPER_OPEN_SDK * (1.0 - gripper / _GRIPPER_CLOSED_RAD)
            _check(
                "set_gripper_position", self._connected_sdk.set_gripper_position(pulses, wait=False)
            )
            self._gripper_sent = gripper

    def halt(self) -> None:
        # In servo mode the arm already holds its last target, and an arm with
        # a fault is stopped.
        if self._mode == _VELOCITY and self._fault is None:
            zeros = [0.0] * len(self._joints)
            _check(
                "vc_set_joint_velocity",
                self._connected_sdk.vc_set_joint_velocity(zeros, is_radian=True),
            )

    def _listen(self, port: In[JointState], field: Interface) -> None:
        """Pass each message on ``port`` to ``command``, reading only its ``field``."""
        self.register_disposable(
            Disposable(
                port.subscribe(lambda msg: self.command(joint_state_to_values(msg, (field,))))
            )
        )

    def _enable(self) -> None:
        """Turn the motors on and make the arm ready for joint targets."""
        _check("motion_enable", self._connected_sdk.motion_enable(enable=True))
        self._mode = None
        self._use_mode(_SERVO)
        if self._gripper is not None:
            _check("set_gripper_enable", self._connected_sdk.set_gripper_enable(True))
            self._gripper_sent = None

    def _use_mode(self, mode: int) -> None:
        """Put the arm in ``mode`` and make it ready, unless it is already in it."""
        if mode == self._mode:
            return
        _check("set_mode", self._connected_sdk.set_mode(mode))
        _check("set_state", self._connected_sdk.set_state(_READY))
        self._mode = mode

    def _send_motion(self, call: str, send: Callable[[], int]) -> None:
        """Send one motion command with ``send``, and raise if the arm refuses it.

        An arm that answers not ready was stopped, by an error or from outside
        DimOS, and takes no command until ``clear_errors()``.
        """
        code = send()
        if code == _NOT_READY and self._fault is None:
            self._fault = "the arm is stopped; call clear_errors() once it is safe to move"
        _check(call, code)

    def _check_jump(self, angles: list[float], measured: list[float]) -> None:
        """Refuse joint targets (rad) too far from where the joints are now (rad)."""
        for joint, target, now, reach in zip(
            self._joints, angles, measured, self._reach, strict=True
        ):
            if abs(target - now) > reach:
                raise XArmError(
                    f"{joint} told {target:.3f} rad, {abs(target - now):.3f} rad from where it "
                    f"is; at most {reach:.3f} rad away is taken"
                )

    def _on_error_warn(self, report: dict[str, int]) -> None:
        """Called by the SDK, on its own thread, when the arm's error or warning changes."""
        error, warn = report.get("error_code", 0), report.get("warn_code", 0)
        if error:
            self._fault = f"the arm reports error {error}; call clear_errors() once it is safe"
            self._report(self._fault)
        if warn:
            logger.warning(f"XArmConnection: the arm reports warning {warn}")

    def _read_joints(self) -> tuple[list[float], list[float], list[float]]:
        """The arm joints' positions (rad), speeds (rad/s) and torques (Nm)."""
        code, states = self._connected_sdk.get_joint_states(is_radian=True)
        _check("get_joint_states", code)
        dof = len(self._joints)
        positions, speeds, efforts = ([float(v) for v in column[:dof]] for column in states)
        return positions, speeds, efforts

    def _read_loop(self) -> None:
        period = 1.0 / _READ_HZ
        due = time.monotonic()
        count = 0
        while not self._stop_reading.wait(max(0.0, due - time.monotonic())):
            due = max(due + period, time.monotonic())
            try:
                positions, speeds, efforts = self._read_joints()
                self._measured = positions
                self._measured_at = time.monotonic()
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
                    code, pulses = self._connected_sdk.get_gripper_position()
                    _check("get_gripper_position", code)
                    # The gripper reports no speed or effort, so it has a message of its own.
                    self.joint_state.publish(
                        JointState(
                            ts=time.time(),
                            name=[self._gripper],
                            position=[_GRIPPER_CLOSED_RAD * (1.0 - pulses / _GRIPPER_OPEN_SDK)],
                        )
                    )
            except Exception as error:
                self._report(f"reading failed: {type(error).__name__}: {error}")
            count += 1


def _open_sdk(ip: str) -> XArmAPI:
    """Connect the SDK to the arm at ``ip``, with angles in radians."""
    return XArmAPI(ip, is_radian=True)


def _arm_type(axis: int, device_type: int) -> str:
    """Which arm the SDK is connected to, from its axis count and device type."""
    if axis == 6 and device_type == 9:
        return "lite6"
    if axis == 6 and device_type == 12:
        return "uf850"
    return f"xarm{axis}"


def _check(call: str, code: int) -> None:
    """Raise if an SDK call returned a code other than 0 (done)."""
    if code != 0:
        raise XArmError(f"{call} returned code {code}")
