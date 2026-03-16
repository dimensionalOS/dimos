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


import math
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
        self._policy = ort.InferenceSession(policy_path, providers=ort.get_available_providers())
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
    """PD attitude controller for the Skydio X2 quadrotor.

    Converts velocity commands into 4 motor thrusts via:
      1. Vertical velocity PD -> collective thrust
      2. Horizontal velocity error -> target tilt angles (clamped)
      3. Attitude PD on body-frame pitch/roll (gravity-vector based, yaw-independent)
      4. Yaw heading-lock (angle PD) or rate-tracking (rate P)
      5. Yaw feedforward to cancel parasitic torque from roll commands
      6. X-config motor mixing -> clipped to [0, 13] per motor
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "x2")

        self.hover_thrust = 3.2495625

        self.kp_vxy = 1.2
        self.kp_vz = 3.0
        self.kp_att = 8.0
        self.kd_att = 4.0
        self.kp_yaw_rate = 1.5
        self.kp_yaw_angle = 3.0
        self.kd_yaw = 1.5

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw_rate = 0.0

        quat = self.data.xquat[self.body_id]
        self._target_yaw = math.atan2(
            2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
            1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2),
        )
        self._yaw_locked = True

    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_vz = vz
        self.cmd_yaw_rate = yaw_rate
        if yaw_rate != 0.0:
            self._yaw_locked = False
        elif not self._yaw_locked:
            q = self.data.xquat[self.body_id]
            self._target_yaw = math.atan2(
                2.0 * (q[0] * q[3] + q[1] * q[2]),
                1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2),
            )
            self._yaw_locked = True

    def compute_control(self) -> np.ndarray[Any, Any]:
        quat = self.data.xquat[self.body_id].copy()
        vel = self.data.cvel[self.body_id]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        v_body = R.T @ vel[3:]
        omega_body = R.T @ vel[:3]
        w, qx, qy, qz = quat

        gz = R[2, :]
        pitch = math.atan2(-gz[0], gz[2])
        roll = math.atan2(gz[1], gz[2])

        base = self.hover_thrust + self.kp_vz * (self.cmd_vz - vel[3 + 2])
        target_pitch = float(np.clip(self.kp_vxy * (self.cmd_vx - v_body[0]), -0.4, 0.4))
        target_roll = float(np.clip(self.kp_vxy * (self.cmd_vy - v_body[1]), -0.4, 0.4))

        pitch_cmd = self.kp_att * (target_pitch - pitch) - self.kd_att * omega_body[1]
        roll_cmd = self.kp_att * (target_roll - roll) - self.kd_att * omega_body[0]

        if self._yaw_locked:
            cur_yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
            angle_err = math.atan2(
                math.sin(self._target_yaw - cur_yaw),
                math.cos(self._target_yaw - cur_yaw),
            )
            yaw_cmd = self.kp_yaw_angle * angle_err - self.kd_yaw * omega_body[2]
        else:
            yaw_cmd = self.kp_yaw_rate * (self.cmd_yaw_rate - omega_body[2])

        yaw_cmd += roll_cmd

        t1 = base + pitch_cmd + roll_cmd - yaw_cmd
        t2 = base + pitch_cmd - roll_cmd + yaw_cmd
        t3 = base - pitch_cmd - roll_cmd + yaw_cmd
        t4 = base - pitch_cmd + roll_cmd - yaw_cmd
        return np.clip(np.array([t1, t2, t3, t4]), 0.0, 13.0)

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        data.ctrl[:] = self.compute_control()
