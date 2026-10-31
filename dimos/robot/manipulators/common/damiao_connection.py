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

"""Position streams and direct native lifecycle for Damiao robots."""

from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass, field
import importlib
import sys
import threading
import time
from typing import TYPE_CHECKING, Any, ClassVar, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_model import (
    Deployment,
    PhysicalModel,
    deployment,
)

if TYPE_CHECKING:
    import can_motor_control


def _native() -> Any:
    # Mock sessions and blueprint parsing do not need the optional native SDK.
    return importlib.import_module("can_motor_control")


def _codec() -> Any:
    return importlib.import_module("can_motor_control.damiao")


@dataclass(frozen=True)
class MotorProfileSpec:
    id: int
    p_max: float
    v_max: float
    t_max: float


@dataclass(frozen=True)
class JointDrive:
    name: str
    motor: Literal["DM8009", "DM4340", "DM4310"] | MotorProfileSpec
    send_id: int
    recv_id: int
    kp: float
    kd: float


@dataclass(frozen=True)
class ArmSpec:
    name: str
    bus: str
    joints: tuple[JointDrive, ...]


@dataclass(frozen=True)
class GripperSpec:
    name: str
    bus: str
    motor: Literal["DM4310"] | MotorProfileSpec
    send_id: int
    recv_id: int
    direction: Literal["decreasing_position", "increasing_position"]
    current: float


@dataclass(frozen=True)
class DamiaoRobot:
    buses: tuple[str, ...]
    arms: tuple[ArmSpec, ...]
    grippers: tuple[GripperSpec, ...]
    physical_model: RobotModel
    planning: RobotModelConfig
    mock_positions: tuple[float, ...]
    gravity_comp: bool
    feedback_margin_rad: float = 0.05

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for arm in self.arms for joint in arm.joints) + tuple(
            gripper.name for gripper in self.grippers
        )


@dataclass
class _NativeSession:
    robot: can_motor_control.Robot
    arms: dict[str, can_motor_control.Arm] = field(default_factory=dict)
    grippers: dict[str, can_motor_control.Gripper] = field(default_factory=dict)
    active: bool = False


@dataclass
class _MockSession:
    positions: list[float]


class DamiaoConnectionConfig(ConnectionConfig):
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    deadman_timeout_s: float = Field(default=0.1, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def _position_only(self) -> Self:
        if self.joint_commands != (Interface.POSITION,) or self.pd or self.base_range is not None:
            raise ValueError("Damiao connections accept position commands only")
        return self


class DamiaoConnection(ConnectionModule):
    """Own one native robot, its measured states and position targets."""

    robot: ClassVar[DamiaoRobot]
    config: DamiaoConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session: _NativeSession | _MockSession | None = None
        self._physical: PhysicalModel | None = None
        self._targets: dict[str, float] = {}
        self._fault: str | None = None
        self._state_frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @classmethod
    def models(cls, *, source: str = "arm") -> Deployment:
        return deployment(cls.robot, source)

    @abstractmethod
    def _can_devices(self) -> dict[str, str] | None:
        """Return explicit logical-bus selectors, or None for an in-memory session."""

    def _make_can_bus(self, name: str, devices: dict[str, str]) -> Any:
        if sys.platform == "linux":
            return _native().SocketCanBus(devices[name])
        if sys.platform == "darwin":
            return _native().GsUsbBus(
                vendor_id=0x1D50, product_id=0x606F, serial_number=devices[name]
            )
        raise RuntimeError(f"Damiao CAN transport is unsupported on {sys.platform}")

    def connect(self) -> None:
        assert self.config.model is not None
        if {joint.name for joint in movable_joints(self.config.model)} != set(
            self.robot.joint_names
        ):
            raise ValueError("connection model joints must match the vendor topology")
        self._state_frame = self.frame(self.config.model.load().root_link)
        devices = self._can_devices()
        if devices is None or self.config.g.simulation:
            self._session = _MockSession(list(self.robot.mock_positions))
            return
        if set(devices) != set(self.robot.buses) or any(
            not device.strip() for device in devices.values()
        ):
            raise ValueError("CAN selectors must cover exactly the robot buses")
        if len(set(devices.values())) != len(devices):
            raise ValueError("CAN selectors must be distinct")
        self._physical = PhysicalModel(self.robot)
        sdk, damiao = _native(), _codec()
        profiles = {
            motor
            for motor in (
                *(joint.motor for arm in self.robot.arms for joint in arm.joints),
                *(gripper.motor for gripper in self.robot.grippers),
            )
            if isinstance(motor, MotorProfileSpec)
        }
        profile_type = getattr(damiao, "MotorProfile", None)
        if profiles and profile_type is None:
            raise RuntimeError(
                "Metal Arm requires can-motor-control>=0.0.11 with MotorProfile support"
            )
        native_profiles: dict[MotorProfileSpec, Any] = {}
        for profile in profiles:
            assert profile_type is not None
            native_profiles[profile] = profile_type(
                profile.id, p_max=profile.p_max, v_max=profile.v_max, t_max=profile.t_max
            )
        builder = sdk.Robot.builder()
        for bus in self.robot.buses:
            bus_profiles = {
                motor
                for motor in (
                    *(
                        joint.motor
                        for arm in self.robot.arms
                        if arm.bus == bus
                        for joint in arm.joints
                    ),
                    *(gripper.motor for gripper in self.robot.grippers if gripper.bus == bus),
                )
                if isinstance(motor, MotorProfileSpec)
            }
            codec = (
                damiao.DamiaoCodec(
                    profiles=[native_profiles[p] for p in sorted(bus_profiles, key=lambda p: p.id)]
                )
                if bus_profiles
                else damiao.DamiaoCodec()
            )
            builder.add_bus(bus, self._make_can_bus(bus, devices), codec)

        def motor_type(motor: str | MotorProfileSpec) -> Any:
            return (
                native_profiles[motor]
                if isinstance(motor, MotorProfileSpec)
                else getattr(damiao.MotorType, motor)
            )

        for arm in self.robot.arms:
            builder.add_arm(
                arm.name,
                bus=arm.bus,
                motors=[
                    sdk.MotorSpec(joint.name, motor_type(joint.motor), joint.send_id, joint.recv_id)
                    for joint in arm.joints
                ],
            )
        for gripper in self.robot.grippers:
            builder.add_gripper(
                gripper.name,
                bus=gripper.bus,
                motor=sdk.MotorSpec(
                    gripper.name, motor_type(gripper.motor), gripper.send_id, gripper.recv_id
                ),
                opening_direction=gripper.direction,
                default_current=gripper.current,
            )
        native = builder.build()
        session = _NativeSession(native)
        self._session = session
        native.connect()
        for arm in self.robot.arms:
            group = native[arm.name]
            if not isinstance(group, sdk.Arm):
                raise TypeError(f"native group {arm.name!r} is not an Arm")
            session.arms[arm.name] = group
        for gripper in self.robot.grippers:
            group = native[gripper.name]
            if not isinstance(group, sdk.Gripper):
                raise TypeError(f"native group {gripper.name!r} is not a Gripper")
            session.grippers[gripper.name] = group
        self._read_state()

    def _activate(self) -> None:
        session = self._session
        if isinstance(session, _NativeSession):
            assert self._physical is not None
            self._physical.gravity(self._arm_positions(session))
            for arm in session.arms.values():
                arm.set_mode("mit")
            session.robot.enable()
            session.active = True
        self._read_state()

    @rpc
    def start(self) -> None:
        try:
            super().start()
            self.register_disposable(Disposable(self.position_command.subscribe(self._on_position)))
            self._reader = threading.Thread(
                target=self._read_loop, daemon=True, name=f"{self.config.source}-read"
            )
            self._reader.start()
        except Exception:
            try:
                self.stop()
            except Exception as cleanup_error:
                self._report(
                    f"startup cleanup failed: {type(cleanup_error).__name__}: {cleanup_error}"
                )
            raise

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            try:
                super().stop()
            finally:
                with self._hardware_lock:
                    session, self._session = self._session, None
                    try:
                        if isinstance(session, _NativeSession):
                            try:
                                session.robot.disable()
                            finally:
                                session.active = False
                                session.arms.clear()
                                session.grippers.clear()
                    finally:
                        self._physical = None
                        self._targets.clear()
        finally:
            if self._reader is not None and self._reader is not threading.current_thread():
                self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                if self._reader.is_alive():
                    error = f"{self.config.source} reader did not stop before the shutdown timeout"
                    self._report(error)
                    raise RuntimeError(error)

    def _on_position(self, msg: JointState) -> None:
        try:
            values = joint_state_to_values(msg, (Interface.POSITION,))
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        if not self._targets:
            self._targets = dict(zip(self.robot.joint_names, self._read_state()[0], strict=True))
        prefix = f"{self.config.source}/"
        self._targets.update(
            {
                key.removeprefix(prefix).removesuffix("/position"): value
                for key, value in values.items()
            }
        )
        self._send_targets()

    def _send_targets(self) -> None:
        session = self._session
        positions = [self._targets[name] for name in self.robot.joint_names]
        if not np.isfinite(positions).all():
            raise ValueError("position command contains non-finite values")
        arm_count = sum(len(arm.joints) for arm in self.robot.arms)
        if any(not 0 <= q <= 1 for q in positions[arm_count:]):
            raise ValueError("gripper opening must be in [0, 1]")
        if isinstance(session, _MockSession):
            session.positions[:] = positions
            return
        if not isinstance(session, _NativeSession) or not session.active:
            raise RuntimeError("Damiao robot is inactive")
        assert self._physical is not None
        gravity = self._physical.gravity(self._arm_positions(session))
        offset = 0
        arm_rows: list[tuple[str, NDArray[np.float64]]] = []
        for arm in self.robot.arms:
            rows = np.asarray(
                [
                    [joint.kp, joint.kd, positions[offset + index], 0.0, gravity[offset + index]]
                    for index, joint in enumerate(arm.joints)
                ],
                dtype=np.float64,
            )
            if not np.isfinite(rows).all():
                raise ValueError(f"arm {arm.name!r} command contains non-finite values")
            arm_rows.append((arm.name, rows))
            offset += len(arm.joints)
        for name, rows in arm_rows:
            session.arms[name].mit_control(rows)
        for gripper in self.robot.grippers:
            session.grippers[gripper.name].set_opening(positions[offset])
            offset += 1
        session.robot.tick(1_000)

    def halt(self) -> None:
        if self._session is None or self._fault is not None:
            return
        self._targets = dict(zip(self.robot.joint_names, self._read_state()[0], strict=True))
        self._send_targets()

    @staticmethod
    def _arm_positions(session: _NativeSession) -> NDArray[np.float64]:
        return np.concatenate([arm.positions().astype(np.float64) for arm in session.arms.values()])

    def _read_state(self) -> tuple[list[float], list[float], list[float]]:
        try:
            return self._sample_state()
        except Exception as error:
            if self._fault is None:
                self._feedback_failed(error)
            raise

    def _sample_state(self) -> tuple[list[float], list[float], list[float]]:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        session = self._session
        if isinstance(session, _MockSession):
            return (
                list(session.positions),
                [0.0] * len(session.positions),
                [0.0] * len(session.positions),
            )
        if not isinstance(session, _NativeSession):
            raise RuntimeError("Damiao robot is disconnected")
        session.robot.refresh()
        session.robot.tick(1_000)
        q: list[float] = []
        dq: list[float] = []
        tau: list[float] = []
        for arm in self.robot.arms:
            group = session.arms[arm.name]
            arrays = [group.positions(), group.velocities(), group.torques()]
            if any(len(array) != len(arm.joints) for array in arrays):
                raise RuntimeError(f"native arm {arm.name!r} returned the wrong state length")
            for output, array in zip((q, dq, tau), arrays, strict=True):
                output.extend(array.astype(np.float64).tolist())
        assert self._physical is not None
        q = self._physical.normalize(q)
        for gripper in self.robot.grippers:
            opening = float(session.grippers[gripper.name].opening) if session.active else 0.0
            if not np.isfinite(opening) or not 0 <= opening <= 1:
                raise RuntimeError(f"gripper {gripper.name!r} returned invalid opening {opening}")
            q.append(opening)
            dq.append(0.0)
            tau.append(0.0)
        if not np.isfinite([q, dq, tau]).all():
            raise RuntimeError("Damiao feedback contains non-finite values")
        return q, dq, tau

    def _feedback_failed(self, error: Exception) -> None:
        self._fault = f"feedback failed: {type(error).__name__}: {error}"
        self._report(self._fault)
        session = self._session
        if isinstance(session, _NativeSession):
            session.active = False
            try:
                session.robot.disable()
            except Exception as disable_error:
                self._report(f"{self._fault}; disable failed: {disable_error}")

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._session is None:
                        return
                    q, dq, tau = self._read_state()
                if self._stop_reading.is_set():
                    return
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=self._state_frame,
                        name=[f"{self.config.source}/{name}" for name in self.robot.joint_names],
                        position=q,
                        velocity=dq,
                        effort=tau,
                    )
                )
            except Exception as error:
                with self._hardware_lock:
                    if self._fault is None:
                        self._feedback_failed(error)
                return
            self._stop_reading.wait(max(0.0, period - (time.monotonic() - started)))
