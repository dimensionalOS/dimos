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

"""MuJoCo simulation engine implementation."""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import mujoco
import mujoco.viewer as viewer  # type: ignore[import-untyped,import-not-found]

from dimos.simulation.engines.base import SimulationEngine
from dimos.simulation.utils.xml_parser import JointMapping, build_joint_mappings
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.msgs.sensor_msgs import JointState

logger = setup_logger()


class MujocoEngine(SimulationEngine):
    """
    MuJoCo simulation engine.

    - starts MuJoCo simulation engine
    - loads robot/environment into simulation
    - applies control commands
    """

    def __init__(self, config_path: Path, headless: bool) -> None:
        super().__init__(config_path=config_path, headless=headless)

        xml_path = self._resolve_xml_path(config_path)
        self._model = mujoco.MjModel.from_xml_path(str(xml_path))
        self._xml_path = xml_path

        self._data = mujoco.MjData(self._model)
        self._joint_mappings = build_joint_mappings(self._xml_path, self._model)
        self._joint_names = [mapping.name for mapping in self._joint_mappings]
        self._num_joints = len(self._joint_names)
        timestep = float(self._model.opt.timestep)
        self._control_frequency = 1.0 / timestep if timestep > 0.0 else 100.0

        self._connected = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None

        self._joint_positions = [0.0] * self._num_joints
        self._joint_velocities = [0.0] * self._num_joints
        self._joint_efforts = [0.0] * self._num_joints

        self._joint_position_targets = [0.0] * self._num_joints
        self._joint_velocity_targets = [0.0] * self._num_joints
        self._joint_effort_targets = [0.0] * self._num_joints
        self._command_mode = "position"
        for i, mapping in enumerate(self._joint_mappings):
            current_pos = self._current_position(mapping)
            self._joint_position_targets[i] = current_pos
            self._joint_positions[i] = current_pos

    def _resolve_xml_path(self, config_path: Path) -> Path:
        if config_path is None:
            raise ValueError("config_path is required for MuJoCo simulation loading")
        resolved = config_path.expanduser()
        xml_path = resolved / "scene.xml" if resolved.is_dir() else resolved
        if not xml_path.exists():
            raise FileNotFoundError(f"MuJoCo XML not found: {xml_path}")
        return xml_path

    def _current_position(self, mapping: JointMapping) -> float:
        if mapping.joint_id is not None and mapping.qpos_adr is not None:
            return float(self._data.qpos[mapping.qpos_adr])
        if mapping.tendon_qpos_adrs:
            return float(
                sum(self._data.qpos[adr] for adr in mapping.tendon_qpos_adrs)
                / len(mapping.tendon_qpos_adrs)
            )
        if mapping.actuator_id is not None:
            return float(self._data.actuator_length[mapping.actuator_id])
        return 0.0

    def _apply_control(self) -> None:
        with self._lock:
            if self._command_mode == "effort":
                targets = list(self._joint_effort_targets)
            elif self._command_mode == "velocity":
                targets = list(self._joint_velocity_targets)
            elif self._command_mode == "position":
                targets = list(self._joint_position_targets)
            for i, mapping in enumerate(self._joint_mappings):
                if mapping.actuator_id is None:
                    continue
                if i < len(targets):
                    self._data.ctrl[mapping.actuator_id] = targets[i]

    def _update_joint_state(self) -> None:
        with self._lock:
            for i, mapping in enumerate(self._joint_mappings):
                if mapping.joint_id is not None:
                    if mapping.qpos_adr is not None:
                        self._joint_positions[i] = float(self._data.qpos[mapping.qpos_adr])
                    if mapping.dof_adr is not None:
                        self._joint_velocities[i] = float(self._data.qvel[mapping.dof_adr])
                        self._joint_efforts[i] = float(self._data.qfrc_actuator[mapping.dof_adr])
                    continue

                if mapping.tendon_qpos_adrs:
                    pos_sum = sum(self._data.qpos[adr] for adr in mapping.tendon_qpos_adrs)
                    count = len(mapping.tendon_qpos_adrs)
                    self._joint_positions[i] = float(pos_sum / count)
                    if mapping.tendon_dof_adrs:
                        vel_sum = sum(self._data.qvel[adr] for adr in mapping.tendon_dof_adrs)
                        self._joint_velocities[i] = float(vel_sum / len(mapping.tendon_dof_adrs))
                    else:
                        self._joint_velocities[i] = 0.0
                elif mapping.actuator_id is not None:
                    self._joint_positions[i] = float(
                        self._data.actuator_length[mapping.actuator_id]
                    )
                    self._joint_velocities[i] = 0.0

                if mapping.actuator_id is not None:
                    self._joint_efforts[i] = float(self._data.actuator_force[mapping.actuator_id])

    def connect(self) -> bool:
        try:
            logger.info(f"{self.__class__.__name__}: connect()")
            with self._lock:
                self._connected = True
                self._stop_event.clear()

            if self._sim_thread is None or not self._sim_thread.is_alive():
                self._sim_thread = threading.Thread(
                    target=self._sim_loop,
                    name=f"{self.__class__.__name__}Sim",
                    daemon=True,
                )
                self._sim_thread.start()
            return True
        except Exception as e:
            logger.error(f"{self.__class__.__name__}: connect() failed: {e}")
            return False

    def disconnect(self) -> bool:
        try:
            logger.info(f"{self.__class__.__name__}: disconnect()")
            with self._lock:
                self._connected = False
            self._stop_event.set()
            if self._sim_thread and self._sim_thread.is_alive():
                self._sim_thread.join(timeout=2.0)
            self._sim_thread = None
            return True
        except Exception as e:
            logger.error(f"{self.__class__.__name__}: disconnect() failed: {e}")
            return False

    def _sim_loop(self) -> None:
        logger.info(f"{self.__class__.__name__}: sim loop started")
        dt = 1.0 / self._control_frequency

        def _step_once(sync_viewer: bool) -> None:
            loop_start = time.time()
            self._apply_control()
            mujoco.mj_step(self._model, self._data)
            if sync_viewer:
                m_viewer.sync()
            self._update_joint_state()

            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if self._headless:
            while not self._stop_event.is_set():
                _step_once(sync_viewer=False)
        else:
            with viewer.launch_passive(
                self._model, self._data, show_left_ui=False, show_right_ui=False
            ) as m_viewer:
                while m_viewer.is_running() and not self._stop_event.is_set():
                    _step_once(sync_viewer=True)

        logger.info(f"{self.__class__.__name__}: sim loop stopped")

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def joint_positions(self) -> list[float]:
        with self._lock:
            return list(self._joint_positions)

    @property
    def joint_velocities(self) -> list[float]:
        with self._lock:
            return list(self._joint_velocities)

    @property
    def joint_efforts(self) -> list[float]:
        with self._lock:
            return list(self._joint_efforts)

    @property
    def control_frequency(self) -> float:
        return self._control_frequency

    def read_joint_positions(self) -> list[float]:
        return self.joint_positions

    def read_joint_velocities(self) -> list[float]:
        return self.joint_velocities

    def read_joint_efforts(self) -> list[float]:
        return self.joint_efforts

    def write_joint_command(self, command: JointState) -> None:
        if command.position:
            self._command_mode = "position"
            self._set_position_targets(command.position)
            return
        if command.velocity:
            self._command_mode = "velocity"
            self._set_velocity_targets(command.velocity)
            return
        if command.effort:
            self._command_mode = "effort"
            self._set_effort_targets(command.effort)
            return

    def _set_position_targets(self, positions: list[float]) -> None:
        if len(positions) > self._num_joints:
            raise ValueError(
                f"Position command has {len(positions)} joints, expected at most {self._num_joints}"
            )
        with self._lock:
            for i in range(len(positions)):
                self._joint_position_targets[i] = float(positions[i])

    def _set_velocity_targets(self, velocities: list[float]) -> None:
        if len(velocities) > self._num_joints:
            raise ValueError(
                f"Velocity command has {len(velocities)} joints, expected at most {self._num_joints}"
            )
        with self._lock:
            for i in range(len(velocities)):
                self._joint_velocity_targets[i] = float(velocities[i])

    def _set_effort_targets(self, efforts: list[float]) -> None:
        if len(efforts) > self._num_joints:
            raise ValueError(
                f"Effort command has {len(efforts)} joints, expected at most {self._num_joints}"
            )
        with self._lock:
            for i in range(len(efforts)):
                self._joint_effort_targets[i] = float(efforts[i])

    def hold_current_position(self) -> None:
        with self._lock:
            self._command_mode = "position"
            for i, mapping in enumerate(self._joint_mappings):
                self._joint_position_targets[i] = self._current_position(mapping)


__all__ = [
    "MujocoEngine",
]
