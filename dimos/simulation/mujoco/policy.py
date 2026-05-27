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
import collections
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.simulation.mujoco.input_controller import InputController
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


G1_GROOT_BALANCE_ONNX = "GR00T-WholeBodyControl-Balance.onnx"
G1_GROOT_WALK_ONNX = "GR00T-WholeBodyControl-Walk.onnx"
G1_GROOT_DEFAULT_ANGLES = np.array(
    [
        -0.1,
        0.0,
        0.0,
        0.3,
        -0.2,
        0.0,
        -0.1,
        0.0,
        0.0,
        0.3,
        -0.2,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)
G1_GROOT_CMD_SCALE = np.array([2.0, 2.0, 0.5], dtype=np.float32)
G1_GROOT_KPS = np.array(
    [150, 150, 150, 200, 40, 40, 150, 150, 150, 200, 40, 40, 250, 250, 250],
    dtype=np.float32,
)
G1_GROOT_KDS = np.array(
    [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 5, 5, 5],
    dtype=np.float32,
)


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


class G1GrootOnnxController:
    """GR00T G1 ONNX locomotion policy adapter for the current MuJoCo stack."""

    _single_obs_dim = 86
    _history_len = 6
    _num_actions = 15
    _height_cmd = 0.74
    _ang_vel_scale = 0.5
    _dof_pos_scale = 1.0
    _dof_vel_scale = 0.05
    _action_scale = 0.25

    def __init__(
        self,
        policy_dir: str,
        default_angles: np.ndarray[Any, Any],
        n_substeps: int,
        input_controller: InputController,
    ) -> None:
        self._default_angles = default_angles.astype(np.float32)
        self._target_angles = np.zeros_like(self._default_angles)
        self._target_angles[: self._num_actions] = G1_GROOT_DEFAULT_ANGLES
        self._last_action = np.zeros(self._num_actions, dtype=np.float32)
        self._counter = 0
        self._n_substeps = n_substeps
        self._input_controller = input_controller

        policy_root = resolve_named_path(Path(policy_dir).expanduser())
        self._balance_policy = self._load_policy(policy_root / G1_GROOT_BALANCE_ONNX)
        self._walk_policy = self._load_policy(policy_root / G1_GROOT_WALK_ONNX)
        self._obs_history: collections.deque[np.ndarray[Any, Any]] = collections.deque(
            [np.zeros(self._single_obs_dim, dtype=np.float32)] * self._history_len,
            maxlen=self._history_len,
        )
        self._obs = np.zeros(self._single_obs_dim * self._history_len, dtype=np.float32)
        self._torque_actuators_configured = False

    def _load_policy(self, policy_path: Path) -> ort.InferenceSession:
        if not policy_path.exists():
            raise FileNotFoundError(
                f"Missing GR00T policy {policy_path}. Download it from "
                "https://huggingface.co/nepyope/GR00T-WholeBodyControl_g1"
            )
        providers = ort.get_available_providers()
        try:
            session = ort.InferenceSession(str(policy_path), providers=providers)
        except RuntimeError:
            logger.warning("GPU providers failed, falling back to CPUExecutionProvider")
            session = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
        logger.info(f"Loaded GR00T policy: {policy_path} with providers: {session.get_providers()}")
        return session

    @staticmethod
    def _quat_rotate_inverse(
        q: np.ndarray[Any, Any], v: np.ndarray[Any, Any]
    ) -> np.ndarray[Any, Any]:
        w, x, y, z = q
        q_conj = np.array([w, -x, -y, -z])
        return np.array(
            [
                v[0] * (q_conj[0] ** 2 + q_conj[1] ** 2 - q_conj[2] ** 2 - q_conj[3] ** 2)
                + v[1] * 2 * (q_conj[1] * q_conj[2] - q_conj[0] * q_conj[3])
                + v[2] * 2 * (q_conj[1] * q_conj[3] + q_conj[0] * q_conj[2]),
                v[0] * 2 * (q_conj[1] * q_conj[2] + q_conj[0] * q_conj[3])
                + v[1] * (q_conj[0] ** 2 - q_conj[1] ** 2 + q_conj[2] ** 2 - q_conj[3] ** 2)
                + v[2] * 2 * (q_conj[2] * q_conj[3] - q_conj[0] * q_conj[1]),
                v[0] * 2 * (q_conj[1] * q_conj[3] - q_conj[0] * q_conj[2])
                + v[1] * 2 * (q_conj[2] * q_conj[3] + q_conj[0] * q_conj[1])
                + v[2] * (q_conj[0] ** 2 - q_conj[1] ** 2 - q_conj[2] ** 2 + q_conj[3] ** 2),
            ],
            dtype=np.float32,
        )

    def _get_obs(self, data: mujoco.MjData) -> tuple[np.ndarray[Any, Any], np.ndarray[Any, Any]]:
        n_joints = len(self._default_angles)
        qj = data.qpos[7 : 7 + n_joints].copy()
        dqj = data.qvel[6 : 6 + n_joints].copy()
        command = self._input_controller.get_command().astype(np.float32)

        padded_defaults = np.zeros(n_joints, dtype=np.float32)
        action_joints = min(self._num_actions, n_joints)
        padded_defaults[:action_joints] = G1_GROOT_DEFAULT_ANGLES[:action_joints]

        single_obs = np.zeros(self._single_obs_dim, dtype=np.float32)
        single_obs[0:3] = command[:3] * G1_GROOT_CMD_SCALE
        single_obs[3] = self._height_cmd
        single_obs[7:10] = data.qvel[3:6] * self._ang_vel_scale
        single_obs[10:13] = self._quat_rotate_inverse(
            data.qpos[3:7].copy(), np.array([0.0, 0.0, -1.0], dtype=np.float32)
        )
        single_obs[13 : 13 + n_joints] = (qj - padded_defaults) * self._dof_pos_scale
        single_obs[13 + n_joints : 13 + 2 * n_joints] = dqj * self._dof_vel_scale
        single_obs[13 + 2 * n_joints : 13 + 2 * n_joints + self._num_actions] = self._last_action
        return single_obs, command

    def _infer(
        self, session: ort.InferenceSession, obs: np.ndarray[Any, Any]
    ) -> np.ndarray[Any, Any]:
        input_name = session.get_inputs()[0].name
        output = session.run(None, {input_name: obs.reshape(1, -1)})[0][0]
        return output.astype(np.float32)

    def _configure_torque_actuators(self, model: mujoco.MjModel) -> None:
        if self._torque_actuators_configured:
            return

        model.actuator_ctrllimited[:] = 0
        model.actuator_gaintype[:] = int(mujoco.mjtGain.mjGAIN_FIXED)
        model.actuator_biastype[:] = int(mujoco.mjtBias.mjBIAS_NONE)
        model.actuator_gainprm[:] = 0
        model.actuator_gainprm[:, 0] = 1
        model.actuator_biasprm[:] = 0
        self._torque_actuators_configured = True

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._configure_torque_actuators(model)
        self._counter += 1

        if self._counter % self._n_substeps == 0:
            single_obs, command = self._get_obs(data)
            self._obs_history.append(single_obs)
            for i, hist_obs in enumerate(self._obs_history):
                start = i * self._single_obs_dim
                self._obs[start : start + self._single_obs_dim] = hist_obs

            policy = (
                self._balance_policy if np.linalg.norm(command[:3]) <= 0.05 else self._walk_policy
            )
            self._last_action = self._infer(policy, self._obs)
            self._target_angles[:] = 0.0
            self._target_angles[: self._num_actions] = (
                self._last_action * self._action_scale + G1_GROOT_DEFAULT_ANGLES
            )

        q = data.qpos[7 : 7 + model.nu]
        dq = data.qvel[6 : 6 + model.nu]
        kps = np.full(model.nu, 100.0, dtype=np.float32)
        kds = np.full(model.nu, 0.5, dtype=np.float32)
        kps[: self._num_actions] = G1_GROOT_KPS
        kds[: self._num_actions] = G1_GROOT_KDS
        data.ctrl[:] = (self._target_angles - q) * kps + (0.0 - dq) * kds
