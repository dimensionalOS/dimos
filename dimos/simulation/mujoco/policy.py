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
from typing import Any

import mujoco
import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.simulation.mujoco.input_controller import InputController
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OnnxController(ABC):
    def __init__(
        self,
        policy_path: str,
        default_angles: np.ndarray[Any, Any],
        n_substeps: int,
        action_scale: float,
        input_controller: InputController,
        ctrl_dt: float | None = None,
        drift_compensation: list[float] | None = None,
    ) -> None:
        self._output_names = ["continuous_actions"]
        providers = ort.get_available_providers()
        try:
            self._policy = ort.InferenceSession(policy_path, providers=providers)
        except RuntimeError:
            logger.warning("GPU providers failed, falling back to CPUExecutionProvider")
            self._policy = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
        logger.info(f"Loaded policy: {policy_path} with providers: {self._policy.get_providers()}")

        self._action_scale = action_scale
        self._default_angles = default_angles
        self._last_action = np.zeros_like(default_angles, dtype=np.float32)

        self._counter = 0
        self._n_substeps = n_substeps
        self._input_controller = input_controller

        self._drift_compensation = np.array(drift_compensation or [0.0, 0.0, 0.0], dtype=np.float32)

    @abstractmethod
    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray[Any, Any]:
        pass

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._counter += 1
        if self._counter % self._n_substeps == 0:
            obs = self.get_obs(model, data)
            onnx_input = {"obs": obs.reshape(1, -1)}
            onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0]
            self._last_action = onnx_pred.copy()
            data.ctrl[:] = onnx_pred * self._action_scale + self._default_angles
            self._post_control_update()

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
        drift_compensation: list[float] | None = None,
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

        self._phase = np.array([0.0, np.pi])
        self._gait_freq = 1.5
        self._phase_dt = 2 * np.pi * self._gait_freq * ctrl_dt

    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray[Any, Any]:
        linvel = data.sensor("local_linvel_pelvis").data
        gyro = data.sensor("gyro_pelvis").data
        imu_xmat = data.site_xmat[model.site("imu_in_pelvis").id].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = data.qpos[7:] - self._default_angles
        joint_velocities = data.qvel[6:]
        phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
        command = self._input_controller.get_command()
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

    def _post_control_update(self) -> None:
        phase_tp1 = self._phase + self._phase_dt
        self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi

class DroneController:
    def __init__(
            self,
            input_controller: InputController,
            drone_hover_thrust: float = 0.26487,
            attitude_p: float = 0.4,
            attitude_d: float = 0.5,
            yaw_p: float = 1,
            yaw_d: float = 0.05,
            max_tilt_angle: float = 0.1,
            max_yaw_rate: float = 2.0,
            velocity_damping: float = 0.0,
            **kwargs: Any,
    ) -> None:
        self._input_controller = input_controller
        self._drone_hover_thrust = drone_hover_thrust
        self._attitude_p = attitude_p
        self._attitude_d = attitude_d
        self._yaw_p = yaw_p
        self._yaw_d = yaw_d
        self._max_tilt_angle = max_tilt_angle
        self._max_yaw_rate = max_yaw_rate
        self._velocity_damping = velocity_damping

    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        return self._input_controller.get_command().astype(np.float32)
    
    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        command = self._input_controller.get_command()

        pitch = float(command[0]) * self._max_tilt_angle
        roll = -float(command[1]) * self._max_tilt_angle
        yaw_rate_desired = float(command[2]) * self._max_yaw_rate

        qw, qx, qy, qz = data.qpos[3:7]

        # Quaternion to Euler 
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        current_roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        current_pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        current_yaw = np.arctan2(siny_cosp, cosy_cosp)

        roll_rate = data.qvel[3]
        pitch_rate = data.qvel[4]
        yaw_rate = data.qvel[5]

        vx_w, vy_w = data.qvel[0], data.qvel[1]
        cos_y = np.cos(current_yaw)
        sin_y = np.sin(current_yaw)
        vx_body = vx_w * cos_y + vy_w * sin_y
        vy_body = -vx_w * sin_y + vy_w * cos_y
        
        desired_pitch = np.clip(pitch - self._velocity_damping * vx_body, -self._max_tilt_angle, self._max_tilt_angle)
        desired_roll = np.clip(roll + self._velocity_damping * vy_body, -self._max_tilt_angle, self._max_tilt_angle)

        yaw_rate_error = yaw_rate - yaw_rate_desired

        cos_tilt = max(1.0 - 2.0 * (qx * qx + qy * qy), 0.5)

        data.ctrl[0] = self._drone_hover_thrust / cos_tilt
        data.ctrl[1] = self._attitude_p * (current_roll - desired_roll) + self._attitude_d * roll_rate
        data.ctrl[2] = self._attitude_p * (current_pitch - desired_pitch) + self._attitude_d * pitch_rate
        data.ctrl[3] = self._yaw_d * yaw_rate_error