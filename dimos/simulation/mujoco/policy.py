#!/usr/bin/env python3

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


from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.simulation.mujoco.input_controller import InputController
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

POLICY_CONTROL_DT = 0.02
POLICY_ACTION_SCALE = 0.5
G1_PHYSICS_DT = 0.002
G1_DRIFT_COMPENSATION = (-0.18, 0.0, -0.09)


class OnnxController(ABC):
    def __init__(
        self,
        policy_path: str,
        default_angles: np.ndarray[Any, Any],
        n_substeps: int,
        action_scale: float,
        input_controller: InputController,
        ctrl_dt: float | None = None,
        drift_compensation: Sequence[float] | None = None,
    ) -> None:
        self._output_names = ["continuous_actions"]
        # The policy is a small MLP: CPU inference beats CUDA, TensorRT and CoreML
        # for it, and the GPU providers spend seconds building a session. One
        # thread, because the default pool spins every core between inferences.
        options = ort.SessionOptions()  # type: ignore[attr-defined]
        options.intra_op_num_threads = 1
        self._policy = ort.InferenceSession(
            policy_path, sess_options=options, providers=["CPUExecutionProvider"]
        )
        logger.info(f"Loaded policy: {policy_path} with providers: {self._policy.get_providers()}")

        self._action_scale = action_scale
        self._default_angles = default_angles
        self._last_action = np.zeros_like(default_angles, dtype=np.float32)

        self._targets = default_angles.copy()
        self._counter = 0
        self._n_substeps = n_substeps
        self._input_controller = input_controller

        self._drift_compensation = np.array(drift_compensation or [0.0, 0.0, 0.0], dtype=np.float32)

    @abstractmethod
    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray[Any, Any]:
        pass

    def reset(self) -> None:
        self._last_action.fill(0)
        self._targets = self._default_angles.copy()
        self._counter = 0

    def get_action(self, model: mujoco.MjModel, data: mujoco.MjData) -> NDArray[np.float64]:
        """Compute this policy's targets without writing any world actuator."""
        self._counter += 1
        if self._counter % self._n_substeps == 0:
            obs = self.get_obs(model, data)
            onnx_input = {"obs": obs.reshape(1, -1)}
            onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0]
            self._last_action = onnx_pred.copy()
            self._targets = onnx_pred * self._action_scale + self._default_angles
            self._post_control_update()
        return np.asarray(self._targets, dtype=np.float64).copy()

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Legacy single-world callback; shared worlds consume get_action instead."""
        data.ctrl[:] = self.get_action(model, data)

    def _post_control_update(self) -> None:  # noqa: B027
        pass


class Go1OnnxController(OnnxController):
    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray[Any, Any]:
        linvel = data.sensor("local_linvel").data
        gyro = data.sensor("gyro").data
        imu_xmat = data.site_xmat[model.site("imu").id].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = data.qpos[7:] - self._default_angles
        joint_velocities = data.qvel[6:]
        obs = np.hstack(
            [
                linvel,
                gyro,
                gravity,
                joint_angles,
                joint_velocities,
                self._last_action,
                self._input_controller.get_command(),
            ]
        )
        return obs.astype(np.float32)


class G1OnnxController(OnnxController):
    def __init__(
        self,
        policy_path: str,
        default_angles: np.ndarray[Any, Any],
        ctrl_dt: float,
        n_substeps: int,
        action_scale: float,
        input_controller: InputController,
        drift_compensation: Sequence[float] | None = None,
        *,
        naming_prefix: str = "",
        qpos_indices: NDArray[np.int64] | None = None,
        dof_indices: NDArray[np.int64] | None = None,
    ) -> None:
        super().__init__(
            policy_path,
            default_angles,
            n_substeps,
            action_scale,
            input_controller,
            ctrl_dt,
            drift_compensation,
        )

        self._naming_prefix = naming_prefix
        self._qpos_indices = qpos_indices
        self._dof_indices = dof_indices
        self._phase = np.array([0.0, np.pi])
        self._gait_freq = 1.5
        self._phase_dt = 2 * np.pi * self._gait_freq * ctrl_dt

    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray[Any, Any]:
        linvel = data.sensor(self._naming_prefix + "local_linvel_pelvis").data
        gyro = data.sensor(self._naming_prefix + "gyro_pelvis").data
        imu_xmat = data.site_xmat[model.site(self._naming_prefix + "imu_in_pelvis").id].reshape(
            3, 3
        )
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = (
            data.qpos[7:] if self._qpos_indices is None else data.qpos[self._qpos_indices]
        )
        joint_angles = joint_angles - self._default_angles
        joint_velocities = (
            data.qvel[6:] if self._dof_indices is None else data.qvel[self._dof_indices]
        )
        phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
        command = self._input_controller.get_command().copy()
        command[0] = command[0] * 2
        command[1] = command[1] * 2
        command[0] += self._drift_compensation[0]
        command[1] += self._drift_compensation[1]
        command[2] += self._drift_compensation[2]
        obs = np.hstack(
            [
                linvel,
                gyro,
                gravity,
                command,
                joint_angles,
                joint_velocities,
                self._last_action,
                phase,
            ]
        )
        return obs.astype(np.float32)

    def reset(self) -> None:
        super().reset()
        self._phase = np.array([0.0, np.pi])

    def _post_control_update(self) -> None:
        phase_tp1 = self._phase + self._phase_dt
        self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi


class G1VelocityController:
    """The existing G1 velocity-policy surrogate, bound to one robot in a world.

    This emulates a velocity interface; it does not execute Unitree firmware or
    the GR00T policy. All actuator writes remain with the shared simulator.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        prefix: str,
        joint_names: tuple[str, ...],
        dt: float,
        *,
        policy_path: str,
    ) -> None:
        self._model, self._data = model, data
        self._command = np.zeros(3, dtype=np.float32)
        actuators = np.asarray(
            [index for index in range(model.nu) if model.actuator(index).name.startswith(prefix)],
            dtype=np.int64,
        )
        if len(actuators) != len(joint_names):
            raise ValueError("velocity policy motor mapping must match its robot's actuators")
        joints = model.actuator_trnid[actuators, 0]
        qpos = np.asarray(model.jnt_qposadr[joints], dtype=np.int64)
        dofs = np.asarray(model.jnt_dofadr[joints], dtype=np.int64)
        ctrl_dt = POLICY_CONTROL_DT
        n_substeps = round(ctrl_dt / dt)
        if n_substeps < 1 or not np.isclose(n_substeps * dt, ctrl_dt):
            raise ValueError("G1 velocity policy requires a physics timestep dividing 0.02 seconds")
        self._policy = G1OnnxController(
            policy_path=policy_path,
            default_angles=np.asarray(data.qpos[qpos]).copy(),
            ctrl_dt=ctrl_dt,
            n_substeps=n_substeps,
            action_scale=POLICY_ACTION_SCALE,
            input_controller=self,
            drift_compensation=G1_DRIFT_COMPENSATION,
            naming_prefix=prefix,
            qpos_indices=qpos,
            dof_indices=dofs,
        )

    def get_command(self) -> NDArray[np.float32]:
        return self._command.copy()

    def stop(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._command.fill(0)
        self._policy.reset()

    def step(self, command: Twist) -> MotorCommandArray:
        self._command[:] = command.linear.x, command.linear.y, command.angular.z
        targets = self._policy.get_action(self._model, self._data)
        return MotorCommandArray(q=targets.tolist())
