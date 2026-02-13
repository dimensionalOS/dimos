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

"""Simulator-agnostic manipulator simulation module."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING, Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.simulation.engines import EngineType, get_engine
from dimos.simulation.manipulators.sim_manip_interface import SimManipInterface

if TYPE_CHECKING:
    from collections.abc import Callable
    from pathlib import Path


@dataclass(kw_only=True)
class SimulationModuleConfig(ModuleConfig):
    engine: EngineType
    config_path: Path | Callable[[], Path]
    headless: bool = False


class SimulationModule(Module[SimulationModuleConfig]):
    """Module wrapper for manipulator simulation across engines."""

    default_config = SimulationModuleConfig
    config: SimulationModuleConfig

    joint_state: Out[JointState]
    robot_state: Out[RobotState]
    joint_position_command: In[JointCommand]
    joint_velocity_command: In[JointCommand]

    MIN_CONTROL_RATE = 1.0

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._backend: SimManipInterface | None = None
        self._control_rate = 100.0
        self._monitor_rate = 100.0
        self._joint_prefix = "joint"
        self._stop_event = threading.Event()
        self._control_thread: threading.Thread | None = None
        self._monitor_thread: threading.Thread | None = None
        self._command_lock = threading.Lock()
        self._pending_positions: list[float] | None = None
        self._pending_velocities: list[float] | None = None

    def _create_backend(self) -> SimManipInterface:
        engine_cls = get_engine(self.config.engine)
        config_path = (
            self.config.config_path()
            if callable(self.config.config_path)
            else self.config.config_path
        )
        engine = engine_cls(
            config_path=config_path,
            headless=self.config.headless,
        )
        return SimManipInterface(engine=engine)

    @rpc
    def start(self) -> None:
        super().start()
        if self._backend is None:
            self._backend = self._create_backend()
        if not self._backend.connect():
            raise RuntimeError("Failed to connect to simulation backend")
        self._backend.write_enable(True)

        self._disposables.add(
            Disposable(self.joint_position_command.subscribe(self._on_joint_position_command))
        )
        self._disposables.add(
            Disposable(self.joint_velocity_command.subscribe(self._on_joint_velocity_command))
        )

        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name=f"{self.__class__.__name__}-control",
        )
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name=f"{self.__class__.__name__}-monitor",
        )
        self._control_thread.start()
        self._monitor_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
        if self._backend:
            self._backend.disconnect()
        super().stop()

    @rpc
    def enable_servos(self) -> bool:
        if not self._backend:
            return False
        return self._backend.write_enable(True)

    @rpc
    def disable_servos(self) -> bool:
        if not self._backend:
            return False
        return self._backend.write_enable(False)

    @rpc
    def clear_errors(self) -> bool:
        if not self._backend:
            return False
        return self._backend.write_clear_errors()

    @rpc
    def emergency_stop(self) -> bool:
        if not self._backend:
            return False
        return self._backend.write_stop()

    def _on_joint_position_command(self, msg: JointCommand) -> None:
        with self._command_lock:
            self._pending_positions = list(msg.positions)
            self._pending_velocities = None

    def _on_joint_velocity_command(self, msg: JointCommand) -> None:
        with self._command_lock:
            self._pending_velocities = list(msg.positions)
            self._pending_positions = None

    def _control_loop(self) -> None:
        period = 1.0 / max(self._control_rate, self.MIN_CONTROL_RATE)
        next_tick = time.monotonic()  # monotonic time used to avoid time drift
        while not self._stop_event.is_set():
            with self._command_lock:
                positions = (
                    None if self._pending_positions is None else list(self._pending_positions)
                )
                velocities = (
                    None if self._pending_velocities is None else list(self._pending_velocities)
                )

            if self._backend:
                if positions is not None:
                    self._backend.write_joint_positions(positions)
                elif velocities is not None:
                    self._backend.write_joint_velocities(velocities)
                dof = self._backend.get_dof()
                names = self._resolve_joint_names(dof)
                positions = self._backend.read_joint_positions()
                velocities = self._backend.read_joint_velocities()
                efforts = self._backend.read_joint_efforts()
                self.joint_state.publish(
                    JointState(
                        frame_id=self.frame_id,
                        name=names,
                        position=positions,
                        velocity=velocities,
                        effort=efforts,
                    )
                )
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                if self._stop_event.wait(sleep_for):
                    break
            else:
                next_tick = time.monotonic()

    def _monitor_loop(self) -> None:
        period = 1.0 / max(self._monitor_rate, self.MIN_CONTROL_RATE)
        next_tick = time.monotonic()  # monotonic time used to avoid time drift
        while not self._stop_event.is_set():
            if not self._backend:
                pass
            else:
                dof = self._backend.get_dof()
                self._resolve_joint_names(dof)
                positions = self._backend.read_joint_positions()
                self._backend.read_joint_velocities()
                self._backend.read_joint_efforts()
                state = self._backend.read_state()
                error_code, _ = self._backend.read_error()
                self.robot_state.publish(
                    RobotState(
                        state=state.get("state", 0),
                        mode=state.get("mode", 0),
                        error_code=error_code,
                        warn_code=0,
                        cmdnum=0,
                        mt_brake=0,
                        mt_able=1 if self._backend.read_enabled() else 0,
                        tcp_pose=[],
                        tcp_offset=[],
                        joints=[float(p) for p in positions],
                    )
                )
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                if self._stop_event.wait(sleep_for):
                    break
            else:
                next_tick = time.monotonic()

    def _resolve_joint_names(self, dof: int) -> list[str]:
        if self._backend:
            names = self._backend.get_joint_names()
            if len(names) >= dof:
                return list(names[:dof])
        return [f"{self._joint_prefix}{i + 1}" for i in range(dof)]


simulation = SimulationModule.blueprint

__all__ = [
    "SimulationModule",
    "SimulationModuleConfig",
    "simulation",
]
