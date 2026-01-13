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

"""
Base class for MuJoCo simulation bridges.

This base class provides common infrastructure for connecting MuJoCo simulation
with robot arm drivers, allowing the same driver code to work with both hardware
and simulation.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import threading
import time
from typing import TYPE_CHECKING

import mujoco
import mujoco.viewer as viewer

from dimos.simulation.manipulators.mujoco_sim.constants import (
    DEFAULT_CONTROL_FREQUENCY,
    MIN_CONTROL_FREQUENCY,
    POSITION_ZERO_THRESHOLD,
    THREAD_JOIN_TIMEOUT,
    VELOCITY_STOP_THRESHOLD,
)
from dimos.simulation.manipulators.mujoco_sim.model_utils import load_manipulator_model
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    pass

logger = setup_logger()


class MujocoSimBridgeBase(ABC):
    """
    Base class for MuJoCo simulation bridges that connect simulation with robot drivers.

    This class handles:
    - MuJoCo model loading
    - Threading infrastructure for simulation loop
    - Basic state management (joint positions, velocities, efforts)
    - Simulation stepping and viewer integration
    - Connection management

    Subclasses should implement robot-specific SDK interface methods.
    """

    def __init__(
        self,
        robot_name: str,
        num_joints: int,
        control_frequency: float = DEFAULT_CONTROL_FREQUENCY,
        model_path: Path | str | None = None,
    ):
        """
        Initialize the MuJoCo simulation bridge.

        Args:
            robot_name: Name of the robot (e.g., "piper", "xarm")
            num_joints: Number of joints in the robot arm
            control_frequency: Control frequency in Hz
            model_path: Optional explicit path to MuJoCo model XML file.
                       If None, will be found automatically.
        """
        self._robot_name = robot_name
        self._num_joints = num_joints
        self._control_frequency = (
            control_frequency
            if control_frequency > MIN_CONTROL_FREQUENCY
            else DEFAULT_CONTROL_FREQUENCY
        )

        # Load MuJoCo model
        self._model, self._data = load_manipulator_model(
            robot_name=robot_name,
            num_joints=num_joints,
            model_path=model_path,
        )

        # --- State variables --- #
        self._connected: bool = False

        # --- Threading infrastructure --- #
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None

        # --- Joint state (in radians, MuJoCo's native unit) --- #
        self._joint_positions = [0.0] * self._num_joints
        self._joint_velocities = [0.0] * self._num_joints
        self._joint_efforts = [0.0] * self._num_joints

        # --- Joint targets (in radians) --- #
        self._joint_position_targets = [0.0] * self._num_joints

        # Initialize position targets to current joint positions at startup
        # This prevents the arm from moving to zero when motion is first enabled
        for i in range(min(self._num_joints, self._model.nq)):
            current_pos = float(self._data.qpos[i])
            self._joint_position_targets[i] = current_pos
            self._joint_positions[i] = current_pos

    # ============= Abstract Methods (must be implemented by subclasses) =============

    @abstractmethod
    def _apply_control(self) -> None:
        """
        Apply control commands to MuJoCo actuators.

        This method is called during the simulation loop. Subclasses should:
        - Read joint position/velocity targets from `self._joint_position_targets` or similar
        - Apply them to `self._data.ctrl[i]` for each actuator
        - Handle any robot-specific control logic (e.g., velocity control, position control)
        """
        pass

    @abstractmethod
    def _update_joint_state(self) -> None:
        """
        Update internal joint state from MuJoCo simulation.

        This method is called after each simulation step. Subclasses should:
        - Read `self._data.qpos`, `self._data.qvel`, `self._data.qfrc_actuator`
        - Update `self._joint_positions`, `self._joint_velocities`, `self._joint_efforts`
        - Perform any unit conversions if needed (e.g., to robot-specific units)
        """
        pass

    # ============= Connection Management =============

    def connect(self) -> None:
        """Connect to simulation and start the simulation loop."""
        logger.info(f"{self.__class__.__name__}: connect()")
        with self._lock:
            self._connected = True
            self._stop_event.clear()

        # Start simulation thread
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
            self._sim_thread.join(timeout=THREAD_JOIN_TIMEOUT)
        self._sim_thread = None

    # ============= Simulation Loop =============

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

        with viewer.launch_passive(
            self._model, self._data, show_left_ui=False, show_right_ui=False
        ) as m_viewer:
            while m_viewer.is_running() and not self._stop_event.is_set():
                loop_start = time.time()

                # Apply control (subclass implements this)
                self._apply_control()

                # Step simulation
                mujoco.mj_step(self._model, self._data)
                m_viewer.sync()

                # Update joint state (subclass implements this)
                self._update_joint_state()

                # Maintain accurate control frequency by accounting for execution time
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        logger.info(f"{self.__class__.__name__}: sim loop stopped")

    # ============= Properties =============

    @property
    def connected(self) -> bool:
        """Whether the bridge is connected to simulation."""
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

    # ============= Helper Methods =============

    def hold_current_position(self) -> None:
        """Lock joints at their current positions (useful for emergency stop, enable, etc.)."""
        with self._lock:
            for i in range(min(self._num_joints, self._model.nq)):
                current_pos = float(self._data.qpos[i])
                self._joint_position_targets[i] = current_pos
