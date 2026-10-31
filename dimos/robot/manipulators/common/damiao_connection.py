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

"""Stream-based connections for the existing Damiao whole-body adapters."""

from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass, replace
from hashlib import sha256
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Any, Protocol, runtime_checkable
import xml.etree.ElementTree as ET

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import HardwareComponent
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.hardware.manipulators.spec import ControlMode
from dimos.hardware.whole_body.registry import whole_body_adapter_registry
from dimos.hardware.whole_body.spec import WholeBodyAdapter
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT
from dimos.robot.assets.model import RobotModel


class NormalizedGripperDescription(os.PathLike[str]):
    """Arm geometry plus logical 0-1 gripper coordinates for control routing.

    Added joints move empty links only. Their values are calibrated openings,
    not jaw distances or motor angles. Existing finger geometry stays fixed.
    """

    def __init__(self, model: RobotModel, grippers: tuple[str, ...]) -> None:
        self.model = model
        self.grippers = grippers

    def cbor_encode(self) -> tuple[RobotModel, tuple[str, ...]]:
        return self.model, self.grippers

    @classmethod
    def cbor_decode(
        cls, values: tuple[RobotModel, tuple[str, ...]]
    ) -> NormalizedGripperDescription:
        return cls(*values)

    def __fspath__(self) -> str:
        loaded = self.model.load()
        root = ET.fromstring(loaded.xml)
        names = {joint.name for joint in loaded.joints}
        for name in self.grippers:
            if name in names:
                raise ValueError(f"logical gripper joint already exists: {name}")
            link = f"{name}_control_link"
            ET.SubElement(root, "link", name=link)
            joint = ET.SubElement(root, "joint", name=name, type="revolute")
            ET.SubElement(joint, "parent", link=loaded.root_link)
            ET.SubElement(joint, "child", link=link)
            ET.SubElement(joint, "limit", lower="0", upper="1", velocity="0", effort="0")
        data = ET.tostring(root, encoding="unicode")
        folder = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "derived" / "damiao_control"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{sha256(data.encode()).hexdigest()}.urdf"
        if not path.exists():
            with tempfile.NamedTemporaryFile(mode="w", dir=folder, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
            try:
                temporary.replace(path)
            finally:
                temporary.unlink(missing_ok=True)
        return str(path)


def damiao_control_model(model: RobotModel, grippers: tuple[str, ...]) -> RobotModel:
    """Add calibrated gripper coordinates without resolving the source at import time."""
    return RobotModel.from_file(NormalizedGripperDescription(model, grippers))


@runtime_checkable
class DamiaoAdapter(WholeBodyAdapter, Protocol):
    def activate(self) -> bool: ...
    def deactivate(self) -> bool: ...


@dataclass
class DamiaoSession:
    hardware: ConnectedWholeBody
    adapter: DamiaoAdapter


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
    """Position commands and measured states, with the existing adapter's MIT gains.

    Partial commands retain previous targets, initially seeded from feedback.
    Deadman holds the measured pose. Stop then disables and disconnects the
    adapter. Feedback failures disable the adapter and require a new session.
    """

    config: DamiaoConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session: DamiaoSession | None = None
        self._fault: str | None = None
        self._state_frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @abstractmethod
    def hardware_component(self) -> HardwareComponent:
        """Return the existing vendor adapter configuration, with local port names."""

    def connect(self) -> None:
        component = self.hardware_component()
        assert self.config.model is not None
        if {joint.name for joint in movable_joints(self.config.model)} != set(component.joints):
            raise ValueError("connection model joints must match the vendor topology")
        self._state_frame = self.frame(self.config.model.load().root_link)
        if self.config.g.simulation:
            component = replace(component, adapter_type="mock_whole_body", adapter_kwargs={})
        adapter = whole_body_adapter_registry.create(
            component.adapter_type,
            dof=len(component.joints),
            hardware_id=self.config.source,
            address=component.address,
            **component.adapter_kwargs,
        )
        if not isinstance(adapter, DamiaoAdapter):
            raise TypeError("Damiao connection requires an adapter with activation lifecycle")
        try:
            if not adapter.connect():
                raise ConnectionError(f"failed to connect {self.config.source}")
            adapter.read_motor_states()
            if component.auto_enable and not adapter.activate():
                raise ConnectionError(f"failed to activate {self.config.source}")
            self._session = DamiaoSession(ConnectedWholeBody(adapter, component), adapter)
        except Exception:
            adapter.disconnect()
            raise

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
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            super().stop()
        finally:
            with self._hardware_lock:
                session, self._session = self._session, None
                if session is not None:
                    try:
                        if not session.adapter.deactivate():
                            raise RuntimeError(f"failed to disable {self.config.source}")
                    finally:
                        session.hardware.disconnect()

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
        assert self._session is not None
        prefix = f"{self.config.source}/"
        commands = {
            key.removeprefix(prefix).removesuffix("/position"): value
            for key, value in values.items()
        }
        if not self._session.hardware.write_command(commands, ControlMode.POSITION):
            raise RuntimeError(f"{self.config.source} refused position command")

    def halt(self) -> None:
        if self._session is None or self._fault is not None:
            return
        positions = {
            name: state.position for name, state in self._session.hardware.read_state().items()
        }
        if not self._session.hardware.write_command(positions, ControlMode.POSITION):
            raise RuntimeError(f"{self.config.source} refused hold")

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._session is None:
                        return
                    if self._session.hardware.ready_for_control():
                        states = self._session.hardware.read_state()
                    else:
                        states = {}
                if states:
                    self.joint_state.publish(
                        JointState(
                            ts=time.time(),
                            frame_id=self._state_frame,
                            name=[f"{self.config.source}/{name}" for name in states],
                            position=[state.position for state in states.values()],
                            velocity=[state.velocity for state in states.values()],
                            effort=[state.effort for state in states.values()],
                        )
                    )
            except Exception as error:
                with self._hardware_lock:
                    self._fault = f"feedback failed: {type(error).__name__}: {error}"
                    self._report(self._fault)
                    if self._session is not None and not self._session.adapter.deactivate():
                        self._report("failed to disable after feedback failure")
                return
            self._stop_reading.wait(max(0.0, period - (time.monotonic() - started)))
