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


from __future__ import annotations

from pathlib import Path
import threading
import time

import mujoco
import mujoco.viewer as viewer
from robot_descriptions.loaders.mujoco import load_robot_description

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MujocoSimBackend:
    """
    Base class for MuJoCo simulation backend.
    """

    def __init__(
        self,
        robot: str,
        config_path: str | None = None,
        headless: bool = False,
    ):
        """
        Initialize the MuJoCo simulation backend.

        Args:
            robot: Robot description (from robot_descriptions library) name (e.g., "piper", "xarm7_mj_description").
            config_path: Path to a MuJoCo XML or a folder containing scene.xml.
            headless: Run without launching the MuJoCo GUI viewer.
        """
        self._robot_name = robot
        self._headless = headless

        if config_path:
            resolved = Path(config_path).expanduser()
            xml_path = resolved / "scene.xml" if resolved.is_dir() else resolved
            if not xml_path.exists():
                raise FileNotFoundError(f"MuJoCo XML not found: {xml_path}")
            self._model = mujoco.MjModel.from_xml_path(str(xml_path))
        else:
            self._model = load_robot_description(robot)
        self._data = mujoco.MjData(self._model)
        self._num_joints = self._model.nq
        timestep = float(self._model.opt.timestep)
        self._control_frequency = 1.0 / timestep if timestep > 0.0 else 100.0

        self._connected: bool = False

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None

        self._joint_positions = [0.0] * self._num_joints
        self._joint_velocities = [0.0] * self._num_joints
        self._joint_efforts = [0.0] * self._num_joints

        self._joint_position_targets = [0.0] * self._num_joints

        for i in range(min(self._num_joints, self._model.nq)):
            current_pos = float(self._data.qpos[i])
            self._joint_position_targets[i] = current_pos
            self._joint_positions[i] = current_pos

    def _apply_control(self) -> None:
        """
        Apply control commands to MuJoCo actuators.

        Default implementation applies joint position targets directly to actuators.
        """
        with self._lock:
            pos_targets = list(self._joint_position_targets)
        n_act = min(self._num_joints, self._model.nu)
        with self._lock:
            for i in range(n_act):
                self._data.ctrl[i] = pos_targets[i]

    def _update_joint_state(self) -> None:
        """
        Update internal joint state from MuJoCo simulation.

        Default implementation mirrors MuJoCo state directly.
        """
        with self._lock:
            n_q = min(self._num_joints, self._model.nq)
            n_v = min(self._num_joints, self._model.nv)
            n_act = min(self._num_joints, self._model.nu)
            for i in range(n_q):
                self._joint_positions[i] = float(self._data.qpos[i])
            for i in range(n_v):
                self._joint_velocities[i] = float(self._data.qvel[i])
            for i in range(n_act):
                self._joint_efforts[i] = float(self._data.qfrc_actuator[i])

    def connect(self) -> None:
        """Connect to simulation and start the simulation loop."""
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

    def disconnect(self) -> None:
        """Disconnect from simulation and stop the simulation loop."""
        logger.info(f"{self.__class__.__name__}: disconnect()")
        with self._lock:
            self._connected = False

        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
        self._sim_thread = None

    def _sim_loop(self) -> None:
        """
        Main simulation loop running MuJoCo.

        This method:
        1. Launches the MuJoCo viewer
        2. Runs the simulation at the specified control frequency
        3. Calls `_apply_control()` to apply control commands
        4. Steps the simulation
        5. Calls `_update_joint_state()` to update internal state
        """
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
        """Whether the backend is connected to simulation."""
        with self._lock:
            return self._connected

    @property
    def num_joints(self) -> int:
        """Number of joints in the robot."""
        return self._num_joints

    @property
    def model(self) -> mujoco.MjModel:
        """MuJoCo model (read-only)."""
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """MuJoCo data (read-only)."""
        return self._data

    @property
    def joint_positions(self) -> list[float]:
        """Current joint positions in radians (thread-safe copy)."""
        with self._lock:
            return list(self._joint_positions)

    @property
    def joint_velocities(self) -> list[float]:
        """Current joint velocities in rad/s (thread-safe copy)."""
        with self._lock:
            return list(self._joint_velocities)

    @property
    def joint_efforts(self) -> list[float]:
        """Current joint efforts/torques (thread-safe copy)."""
        with self._lock:
            return list(self._joint_efforts)

    @property
    def control_frequency(self) -> float:
        """Control loop frequency derived from the MuJoCo model."""
        return self._control_frequency

    def set_joint_position_targets(self, positions: list[float]) -> None:
        """Set joint position targets in radians."""
        with self._lock:
            limit = min(len(positions), self._num_joints)
            for i in range(limit):
                self._joint_position_targets[i] = float(positions[i])

    def hold_current_position(self) -> None:
        """Lock joints at their current positions."""
        with self._lock:
            for i in range(min(self._num_joints, self._model.nq)):
                current_pos = float(self._data.qpos[i])
                self._joint_position_targets[i] = current_pos
