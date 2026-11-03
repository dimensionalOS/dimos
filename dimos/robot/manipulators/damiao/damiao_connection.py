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
from dataclasses import replace
import sys
import threading
import time
from typing import Any, ClassVar
import uuid

import can_motor_control
import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import limits as limits_of, movable_joints
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.manipulators.damiao.damiao_model import PhysicalModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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

    arm_joints: ClassVar[dict[str, list[str]]]
    gripper_joints: ClassVar[dict[str, str]]
    bus_names: ClassVar[list[str]]
    kp: ClassVar[dict[str, list[float]]]
    kd: ClassVar[dict[str, list[float]]]
    kinematic_model: ClassVar[RobotModel]
    kinematic_joint_names: ClassVar[list[str]]
    planning: ClassVar[RobotModelConfig]
    gravity_comp: ClassVar[bool] = True
    feedback_clamp_margin_rad: ClassVar[float] = 0.05
    config: DamiaoConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._bus_devices: dict[str, str] = {}
        self._robot: can_motor_control.Robot | None = None
        self._arms: dict[str, can_motor_control.Arm] = {}
        self._grippers: dict[str, can_motor_control.Gripper] = {}
        self._active = False
        self._physical: PhysicalModel | None = None
        self._targets: dict[str, float] = {}
        self._fault: str | None = None
        self._state_frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @property
    def joint_names(self) -> list[str]:
        return [joint for joints in self.arm_joints.values() for joint in joints] + list(
            self.gripper_joints.values()
        )

    @classmethod
    def planning_model(cls, source: str | None = None) -> RobotModelConfig:
        planning = cls.planning.model_copy(deep=True)
        if source is None:
            return planning
        names = {name: f"{source}/{name}" for name in planning.joint_names}
        return planning.model_copy(
            update={
                "model": planning.model.with_renamed_joints(names),
                "joint_names": list(names.values()),
                "planning_groups": [
                    replace(group, joint_names=tuple(names[name] for name in group.joint_names))
                    for group in planning.planning_groups
                ],
                "gripper_hardware_id": source if len(cls.gripper_joints) == 1 else None,
            }
        )

    @classmethod
    def control_model(cls, source: str | None = None) -> RobotModel:
        model = cls.planning.model
        grippers = [
            JointDescription(
                name=name,
                type="revolute",
                parent_link=cls.planning.base_link,
                child_link=f"{name}_control_link",
                lower=0.0,
                upper=1.0,
                velocity=0.0,
                effort=0.0,
            )
            for name in cls.gripper_joints.values()
        ]
        if grippers:
            model = model.with_added_joints(*grippers)
        if source is not None:
            names = [*cls.planning.joint_names, *cls.gripper_joints.values()]
            model = model.with_renamed_joints({name: f"{source}/{name}" for name in names})
        return model

    @abstractmethod
    def _can_devices(self) -> dict[str, str]:
        """Return the explicitly configured logical-bus selectors."""

    @abstractmethod
    def _build_robot(self) -> can_motor_control.Robot:
        """Construct the native robot using the vendor's bus and motor declarations."""

    def _make_can_bus(self, name: str) -> Any:
        devices = self._bus_devices
        if sys.platform == "linux":
            return can_motor_control.SocketCanBus(devices[name])
        if sys.platform == "darwin":
            return can_motor_control.GsUsbBus(
                vendor_id=0x1D50, product_id=0x606F, serial_number=devices[name]
            )
        raise RuntimeError(f"Damiao CAN transport is unsupported on {sys.platform}")

    def connect(self) -> None:
        assert self.config.model is not None
        if {joint.name for joint in movable_joints(self.config.model)} != set(self.joint_names):
            raise ValueError("connection model joints must match the vendor topology")
        self._state_frame = self.frame(self.config.model.load().root_link)
        devices = self._can_devices()
        if set(devices) != set(self.bus_names) or any(
            not device.strip() for device in devices.values()
        ):
            raise ValueError("CAN selectors must cover exactly the robot buses")
        if len(set(devices.values())) != len(devices):
            raise ValueError("CAN selectors must be distinct")
        self._bus_devices = devices
        self._physical = PhysicalModel(
            self.kinematic_model,
            self.kinematic_joint_names,
            gravity_comp=self.gravity_comp,
            feedback_margin_rad=self.feedback_clamp_margin_rad,
        )
        native = self._build_robot()
        self._robot = native
        native.connect()
        for name in self.arm_joints:
            group = native[name]
            if not isinstance(group, can_motor_control.Arm):
                raise TypeError(f"native group {name!r} is not an Arm")
            self._arms[name] = group
        for name in self.gripper_joints:
            group = native[name]
            if not isinstance(group, can_motor_control.Gripper):
                raise TypeError(f"native group {name!r} is not a Gripper")
            self._grippers[name] = group
        self._read_state()

    @rpc
    def start(self) -> None:
        try:
            Module.start(self)
            self._session_id = uuid.uuid4().hex
            self._limits = limits_of(self.config)
            self.connect()
            with self._hardware_lock:
                assert self._robot is not None
                assert self._physical is not None
                self._physical.gravity(self._arm_positions())
                for arm in self._arms.values():
                    arm.set_mode("mit")
                logger.info(
                    "Enabling robot and calibrating grippers",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
                self._robot.enable()
                self._active = True
                self._read_state()
                self.register_disposable(
                    Disposable(self.position_command.subscribe(self._on_position))
                )
                self._reader = threading.Thread(
                    target=self._read_loop, daemon=True, name=f"{self.config.source}-read"
                )
                self._reader.start()
                self._connected = True
                logger.info(
                    "Robot connection ready",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
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
        if (
            self._reader is not None
            and self._reader.is_alive()
            and self._reader is not threading.current_thread()
        ):
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            try:
                super().stop()
            finally:
                with self._hardware_lock:
                    robot, self._robot = self._robot, None
                    try:
                        if robot is not None:
                            robot.disable()
                    finally:
                        self._active = False
                        self._arms.clear()
                        self._grippers.clear()
                        self._physical = None
                        self._targets.clear()
        finally:
            if (
                self._reader is not None
                and self._reader.is_alive()
                and self._reader is not threading.current_thread()
            ):
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
            self._targets = dict(zip(self.joint_names, self._read_state()[0], strict=True))
        prefix = f"{self.config.source}/"
        self._targets.update(
            {
                key.removeprefix(prefix).removesuffix("/position"): value
                for key, value in values.items()
            }
        )
        self._send_targets()

    def _send_targets(self) -> None:
        positions = [self._targets[name] for name in self.joint_names]
        if not np.isfinite(positions).all():
            raise ValueError("position command contains non-finite values")
        arm_count = sum(map(len, self.arm_joints.values()))
        if any(not 0 <= q <= 1 for q in positions[arm_count:]):
            raise ValueError("gripper opening must be in [0, 1]")
        if self._robot is None or not self._active:
            raise RuntimeError("Damiao robot is inactive")
        assert self._physical is not None
        gravity = self._physical.gravity(self._arm_positions())
        offset = 0
        arm_rows: list[tuple[str, NDArray[np.float64]]] = []
        for name, joints in self.arm_joints.items():
            rows = np.asarray(
                [
                    [
                        self.kp[name][index],
                        self.kd[name][index],
                        positions[offset + index],
                        0.0,
                        gravity[offset + index],
                    ]
                    for index in range(len(joints))
                ],
                dtype=np.float64,
            )
            if not np.isfinite(rows).all():
                raise ValueError(f"arm {name!r} command contains non-finite values")
            arm_rows.append((name, rows))
            offset += len(joints)
        for name, rows in arm_rows:
            self._arms[name].mit_control(rows)
        for name in self.gripper_joints:
            self._grippers[name].set_opening(positions[offset])
            offset += 1
        self._robot.tick(1_000)

    def halt(self) -> None:
        if self._robot is None or self._fault is not None:
            return
        self._targets = dict(zip(self.joint_names, self._read_state()[0], strict=True))
        self._send_targets()

    def _arm_positions(self) -> NDArray[np.float64]:
        return np.concatenate([arm.positions().astype(np.float64) for arm in self._arms.values()])

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
        if self._robot is None:
            raise RuntimeError("Damiao robot is disconnected")
        self._robot.refresh()
        self._robot.tick(1_000)
        q: list[float] = []
        dq: list[float] = []
        tau: list[float] = []
        for name, joints in self.arm_joints.items():
            group = self._arms[name]
            arrays = [group.positions(), group.velocities(), group.torques()]
            if any(len(array) != len(joints) for array in arrays):
                raise RuntimeError(f"native arm {name!r} returned the wrong state length")
            for output, array in zip((q, dq, tau), arrays, strict=True):
                output.extend(array.astype(np.float64).tolist())
        assert self._physical is not None
        q = self._physical.normalize(q)
        for name in self.gripper_joints:
            opening = float(self._grippers[name].opening) if self._active else 0.0
            if not np.isfinite(opening) or not 0 <= opening <= 1:
                raise RuntimeError(f"gripper {name!r} returned invalid opening {opening}")
            q.append(opening)
            dq.append(0.0)
            tau.append(0.0)
        if not np.isfinite([q, dq, tau]).all():
            raise RuntimeError("Damiao feedback contains non-finite values")
        return q, dq, tau

    def _feedback_failed(self, error: Exception) -> None:
        self._fault = f"feedback failed: {type(error).__name__}: {error}"
        self._report(self._fault)
        if self._robot is not None:
            self._active = False
            try:
                self._robot.disable()
            except Exception as disable_error:
                self._report(f"{self._fault}; disable failed: {disable_error}")

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._robot is None or not self._connected:
                        return
                    q, dq, tau = self._read_state()
                if self._stop_reading.is_set():
                    return
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=self._state_frame,
                        name=[f"{self.config.source}/{name}" for name in self.joint_names],
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
