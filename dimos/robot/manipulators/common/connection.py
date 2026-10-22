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

"""Shared arm device IO. Public robot connections select their model and SDK."""

from __future__ import annotations

import threading
import time
from typing import Any, ClassVar

from pydantic import Field

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import JOINT_STATE_FIELDS, joint_state_to_values
from dimos.control.contract.ports import JointPort
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.hardware.manipulators.registry import adapter_registry
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import StateFrame, StreamDescriptor
from dimos.sim2.module import SimulationSpec


class ArmConnectionConfig(ConnectionConfig):
    source: str = "arm"
    address: str = Field(default_factory=lambda m: m["g"].robot_ip)
    dof: int = Field(default=7, ge=6, le=7)
    gripper: bool = True
    initial_positions: list[float] | None = None
    auto_enable: bool = True
    position_command: JointPort
    velocity_command: JointPort | None = None
    spawn: str = "workbench"
    base_pose: PoseStamped | None = None
    camera_pointcloud: bool = False
    state_rate_hz: float = 100.0


class ArmConnection(ConnectionModule):
    config: ArmConnectionConfig
    adapter_type: ClassVar[str]
    default_sim_scene: ClassVar[str] = "workbench.xml"
    _simulation: SimulationSpec | None

    position_command: In[JointState]
    velocity_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._simulation = None
        self._sim: SimRobotClient | None = None
        self._device: ManipulatorAdapter | None = None
        self._device_lock = threading.RLock()
        self._names = [
            *(f"{self.config.source}/joint{i}" for i in range(1, self.config.dof + 1)),
            *([f"{self.config.source}/gripper"] if self.config.gripper else []),
        ]
        self._targets: list[float] | None = None
        self._sequence = -1
        self._episode = -1
        self._velocity_at: float | None = None
        self._mode = ControlMode.SERVO_POSITION

    def connect(self) -> None:
        if self.config.g.simulation == "mujoco":
            if self._simulation is None:
                raise RuntimeError("simulated arm requires the collected SimulationModule")
            robot_id = self.config.instance_name or self.name
            descriptor = self._simulation.describe()["robots"][robot_id]
            self._sim = SimRobotClient(StreamDescriptor.from_dict(descriptor), writable=True)
            self._sim.connect()
        else:
            self._device = adapter_registry.create(
                self.adapter_type,
                address=self.config.address,
                dof=len(self._names),
                arm_dof=self.config.dof,
                initial_positions=self.config.initial_positions,
            )
            if not self._device.connect():
                raise RuntimeError(f"{self.adapter_type} connection failed")
            if self.config.auto_enable and not self._device.activate():
                raise RuntimeError(f"{self.adapter_type} activation failed")

    def _observe_sim(self) -> StateFrame:
        assert self._sim is not None
        frame = self._sim.read()
        if self._episode != frame.metadata.episode_id:
            self._episode = frame.metadata.episode_id
            self._targets = frame.values["position"].tolist()
            self._velocity_at = None
            self._mode = ControlMode.SERVO_POSITION
        return frame

    def read_state(self) -> dict[str, float] | ControlValues:
        with self._device_lock:
            if self._sim is not None:
                frame = self._observe_sim()
                if frame.metadata.sequence == self._sequence:
                    return {}
                self._sequence = frame.metadata.sequence
                state = JointState(
                    name=self._names,
                    position=frame.values["position"].tolist(),
                    velocity=frame.values["velocity"].tolist(),
                    effort=frame.values["effort"].tolist(),
                    ts=float(frame.values["wall_time"][0]),
                )
            elif self._device is not None:
                state = JointState(
                    name=self._names,
                    position=self._device.read_joint_positions(),
                    velocity=self._device.read_joint_velocities(),
                    effort=self._device.read_joint_efforts(),
                )
            else:
                return {}
            if self._targets is None:
                self._targets = list(state.position)
            values = joint_state_to_values(state, JOINT_STATE_FIELDS)
            selected = {key: values[key] for key in self.description.state_keys()}
            if self._sim is not None:
                return ControlValues(
                    self.config.source,
                    state.ts,
                    interface_names=list(selected),
                    values=list(selected.values()),
                )
            return selected

    def write(self, values: dict[str, float], *, source_ts: float | None = None) -> None:
        with self._device_lock:
            observed = self._observe_sim() if self._sim is not None else None
            if self._targets is None:
                self.read_state()
            if self._targets is None:
                raise RuntimeError("arm has not reported its initial position")
            velocities = {
                i: values[f"{name}/velocity"]
                for i, name in enumerate(self._names)
                if f"{name}/velocity" in values
            }
            if velocities:
                now = time.monotonic()
                dt = (
                    1 / self.config.state_rate_hz
                    if self._velocity_at is None
                    else min(now - self._velocity_at, self.config.deadman_timeout_s)
                )
                self._velocity_at = now
                if self._sim is not None:
                    assert observed is not None
                    if self._mode != ControlMode.VELOCITY:
                        self._targets = observed.values["position"].tolist()
                    for i, velocity in velocities.items():
                        limit = self.description.limits[f"{self._names[i]}/position"]
                        target = self._targets[i] + velocity * dt
                        if limit.lo is not None:
                            target = max(limit.lo, target)
                        if limit.hi is not None:
                            target = min(limit.hi, target)
                        self._targets[i] = target
                    self._sim.write_positions(self._targets, observed=observed, source_ts=source_ts)
                else:
                    assert self._device is not None
                    if self._mode != ControlMode.VELOCITY:
                        if not self._device.set_control_mode(ControlMode.VELOCITY):
                            raise RuntimeError("arm rejected velocity mode")
                    if not self._device.write_joint_velocities(
                        [velocities.get(i, 0.0) for i in range(len(self._names))]
                    ):
                        raise RuntimeError("arm rejected velocity command")
                self._mode = ControlMode.VELOCITY
                return
            arm_position = any(
                f"{name}/position" in values for name in self._names[: self.config.dof]
            )
            targets = [
                values.get(f"{name}/position", position)
                for name, position in zip(self._names, self._targets, strict=True)
            ]
            if self._sim is not None:
                assert observed is not None
                self._sim.write_positions(targets, observed=observed, source_ts=source_ts)
            elif self._device is not None:
                if self._mode == ControlMode.VELOCITY and not arm_position:
                    self.write_gripper_position(targets[-1])
                    self._targets[-1] = targets[-1]
                    return
                if self._mode != ControlMode.SERVO_POSITION:
                    if not self._device.set_control_mode(ControlMode.SERVO_POSITION):
                        raise RuntimeError("arm rejected position mode")
                if not self._device.write_joint_positions(targets):
                    raise RuntimeError("arm rejected position command")
            else:
                raise RuntimeError("arm is not connected")
            self._targets = targets
            if arm_position:
                self._mode = ControlMode.SERVO_POSITION
                self._velocity_at = None

    def write_gripper_position(self, position: float) -> None:
        """Independent gripper IO for devices supporting simultaneous arm velocity."""
        raise NotImplementedError("this arm does not support independent gripper IO")

    def halt(self) -> None:
        with self._device_lock:
            if self._sim is not None:
                observed = self._sim.read()
                self._targets = observed.values["position"].tolist()
                self._sim.write_positions(self._targets, observed=observed)
            elif self._device is not None:
                if not self._device.write_stop():
                    raise RuntimeError("arm stop failed")
                self._targets = None
            self._mode = ControlMode.SERVO_POSITION
            self._velocity_at = None

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            with self._device_lock:
                if self._sim is not None:
                    self._sim.close()
                    self._sim = None
                if self._device is not None:
                    self._device.disconnect()
                    self._device = None
