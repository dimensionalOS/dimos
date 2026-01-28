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
import time
from typing import Any

import mujoco
import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.simulation.mujoco.input_controller import InputController
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_MUJOCO_PROFILER_ENABLED = False
_MUJOCO_PROF: dict[str, float | int] = {
    "control_calls": 0,
    "control_total_s": 0.0,
    "obs_total_s": 0.0,
    "onnx_total_s": 0.0,
}


def set_mujoco_profiler_enabled(enabled: bool) -> None:
    global _MUJOCO_PROFILER_ENABLED
    _MUJOCO_PROFILER_ENABLED = enabled


def get_mujoco_profiler_and_reset() -> dict[str, float | int]:
    global _MUJOCO_PROF
    out = dict(_MUJOCO_PROF)
    _MUJOCO_PROF = {
        "control_calls": 0,
        "control_total_s": 0.0,
        "obs_total_s": 0.0,
        "onnx_total_s": 0.0,
    }
    return out


def _parse_csv_floats(s: str) -> np.ndarray[Any, Any]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    return np.array([float(p) for p in parts], dtype=np.float32)


def _parse_csv_strings(s: str) -> list[str]:
    # Metadata is written as comma-separated values. Joint names do not contain commas.
    return [p.strip() for p in s.split(",") if p.strip()]


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
        # Load policy. Prefer available providers (CPU-only by default, GPU if installed).
        self._policy = ort.InferenceSession(policy_path, providers=ort.get_available_providers())

        # Support multiple exporter conventions. Prefer "continuous_actions" (older exports),
        # then "actions" (RSL-RL/IsaacLab default), and fall back to the first output.
        outputs = [o.name for o in self._policy.get_outputs()]
        if "continuous_actions" in outputs:
            self._output_names = ["continuous_actions"]
        elif "actions" in outputs:
            self._output_names = ["actions"]
        elif outputs:
            self._output_names = [outputs[0]]
        else:
            raise ValueError(f"ONNX policy has no outputs: {policy_path}")

        logger.info(
            "Loaded policy",
            policy_path=policy_path,
            providers=self._policy.get_providers(),
            outputs=self._output_names,
        )

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
        if self._counter % self._n_substeps != 0:
            return

        if _MUJOCO_PROFILER_ENABLED:
            t0 = time.perf_counter()
            obs = self.get_obs(model, data)
            t_obs = time.perf_counter()
            onnx_input = {"obs": obs.reshape(1, -1)}
            onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0]
            t_onnx = time.perf_counter()
            self._last_action = onnx_pred.copy()
            data.ctrl[:] = onnx_pred * self._action_scale + self._default_angles
            self._post_control_update()
            t1 = time.perf_counter()

            _MUJOCO_PROF["control_calls"] = int(_MUJOCO_PROF["control_calls"]) + 1
            _MUJOCO_PROF["control_total_s"] = float(_MUJOCO_PROF["control_total_s"]) + (t1 - t0)
            _MUJOCO_PROF["obs_total_s"] = float(_MUJOCO_PROF["obs_total_s"]) + (t_obs - t0)
            _MUJOCO_PROF["onnx_total_s"] = float(_MUJOCO_PROF["onnx_total_s"]) + (t_onnx - t_obs)
        else:
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


class MjlabVelocityOnnxController(OnnxController):
    """MJLab velocity-policy interface for G1.

    Matches MJLab `Velocity` policy observation layout (99 dims):
      [imu_lin_vel(3), imu_ang_vel(3), projected_gravity(3),
       joint_pos_rel(N), joint_vel_rel(N), last_action(N), command(3)]
    """

    def __init__(
        self,
        policy_path: str,
        default_angles: np.ndarray[Any, Any],
        n_substeps: int,
        action_scale: float,
        input_controller: InputController,
        ctrl_dt: float | None = None,
        *,
        imu_site_name: str = "imu_in_pelvis",
        gravity_vec_w: np.ndarray[Any, Any] | None = None,
    ) -> None:
        del ctrl_dt  # Unused (kept for compatibility with existing load_model params).
        super().__init__(
            policy_path=policy_path,
            default_angles=default_angles,
            n_substeps=n_substeps,
            action_scale=action_scale,
            input_controller=input_controller,
        )
        self._imu_site_name = imu_site_name
        self._gravity_w = (
            gravity_vec_w
            if gravity_vec_w is not None
            else np.array([0.0, 0.0, -1.0], dtype=np.float32)
        )

        # MJLab exporter can embed metadata required to reproduce the exact action contract.
        meta = self._policy.get_modelmeta().custom_metadata_map
        self._policy_joint_names: list[str] | None = None
        self._default_joint_pos_policy: np.ndarray[Any, Any] | None = None
        self._action_scale_policy: np.ndarray[Any, Any] | None = None
        if "joint_names" in meta and "default_joint_pos" in meta and "action_scale" in meta:
            self._policy_joint_names = _parse_csv_strings(meta["joint_names"])
            self._default_joint_pos_policy = _parse_csv_floats(meta["default_joint_pos"])
            self._action_scale_policy = _parse_csv_floats(meta["action_scale"])

        # Lazy-built mappings (depend on the loaded MuJoCo model).
        self._policy_qpos_adr: np.ndarray[Any, Any] | None = None
        self._policy_qvel_adr: np.ndarray[Any, Any] | None = None
        self._ctrl_policy_idx: np.ndarray[Any, Any] | None = None

    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray[Any, Any]:
        self._ensure_mappings(model)
        # Linear/angular velocity sensors (names match MJLab G1 XML).
        linvel = data.sensor("imu_lin_vel").data
        gyro = data.sensor("imu_ang_vel").data

        # Projected gravity in body frame using IMU site orientation.
        imu_xmat = data.site_xmat[model.site(self._imu_site_name).id].reshape(3, 3)
        gravity_b = imu_xmat.T @ self._gravity_w

        assert self._default_joint_pos_policy is not None
        assert self._policy_qpos_adr is not None
        assert self._policy_qvel_adr is not None

        # Joint state relative to MJLab default, in MJLab joint order.
        joint_pos = data.qpos[self._policy_qpos_adr]
        joint_angles = joint_pos - self._default_joint_pos_policy
        joint_velocities = data.qvel[self._policy_qvel_adr]

        obs = np.hstack(
            [
                linvel,
                gyro,
                gravity_b,
                joint_angles,
                joint_velocities,
                self._last_action,
                self._input_controller.get_command(),
            ]
        )
        return obs.astype(np.float32)

    def _ensure_mappings(self, model: mujoco.MjModel) -> None:
        """Build joint/actuator mappings to exactly match MJLab joint ordering."""
        if self._policy_joint_names is None:
            raise ValueError(
                "MJLab profile requires ONNX metadata keys: joint_names, default_joint_pos, action_scale."
            )
        if self._policy_qpos_adr is not None:
            return

        names = self._policy_joint_names
        default = self._default_joint_pos_policy
        scale = self._action_scale_policy
        assert default is not None and scale is not None

        if len(names) != len(default) or len(names) != len(scale):
            raise ValueError(
                f"ONNX metadata size mismatch: len(joint_names)={len(names)}, "
                f"len(default_joint_pos)={len(default)}, len(action_scale)={len(scale)}"
            )

        # Build qpos/qvel address arrays in policy joint order.
        qpos_adr: list[int] = []
        qvel_adr: list[int] = []
        for jname in names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise ValueError(f"Policy joint '{jname}' not found in MuJoCo model joints.")
            qpos_adr.append(int(model.jnt_qposadr[jid]))
            qvel_adr.append(int(model.jnt_dofadr[jid]))
        self._policy_qpos_adr = np.array(qpos_adr, dtype=np.int32)
        self._policy_qvel_adr = np.array(qvel_adr, dtype=np.int32)

        # Build actuator ctrl->policy index mapping so actions are applied to the correct joints.
        index_by_name = {n: i for i, n in enumerate(names)}
        ctrl_policy_idx: list[int] = []
        for act_id in range(model.nu):
            # For joint actuators, actuator_trnid[act_id, 0] is the joint id.
            jid = int(model.actuator_trnid[act_id, 0])
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname is None or jname not in index_by_name:
                raise ValueError(
                    f"Actuator {act_id} joint '{jname}' not present in policy joint_names metadata."
                )
            ctrl_policy_idx.append(index_by_name[jname])
        self._ctrl_policy_idx = np.array(ctrl_policy_idx, dtype=np.int32)

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Override: apply MJLab action contract (name-based remap + per-joint scaling)."""
        self._counter += 1
        if self._counter % self._n_substeps != 0:
            return

        if _MUJOCO_PROFILER_ENABLED:
            t0 = time.perf_counter()

            self._ensure_mappings(model)
            assert self._ctrl_policy_idx is not None
            assert self._default_joint_pos_policy is not None
            assert self._action_scale_policy is not None

            obs = self.get_obs(model, data)
            t_obs = time.perf_counter()
            onnx_input = {"obs": obs.reshape(1, -1)}
            onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0].astype(np.float32)
            t_onnx = time.perf_counter()

            # Store last action in MJLab policy order (for next observation).
            self._last_action = onnx_pred.copy()

            # Apply control in MuJoCo actuator order.
            idx = self._ctrl_policy_idx
            targets = (
                self._default_joint_pos_policy[idx]
                + onnx_pred[idx] * self._action_scale_policy[idx]
            )
            data.ctrl[:] = targets
            t1 = time.perf_counter()

            _MUJOCO_PROF["control_calls"] = int(_MUJOCO_PROF["control_calls"]) + 1
            _MUJOCO_PROF["control_total_s"] = float(_MUJOCO_PROF["control_total_s"]) + (t1 - t0)
            _MUJOCO_PROF["obs_total_s"] = float(_MUJOCO_PROF["obs_total_s"]) + (t_obs - t0)
            _MUJOCO_PROF["onnx_total_s"] = float(_MUJOCO_PROF["onnx_total_s"]) + (t_onnx - t_obs)
        else:
            self._ensure_mappings(model)
            assert self._ctrl_policy_idx is not None
            assert self._default_joint_pos_policy is not None
            assert self._action_scale_policy is not None

            obs = self.get_obs(model, data)
            onnx_input = {"obs": obs.reshape(1, -1)}
            onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0].astype(np.float32)

            # Store last action in MJLab policy order (for next observation).
            self._last_action = onnx_pred.copy()

            # Apply control in MuJoCo actuator order.
            idx = self._ctrl_policy_idx
            targets = (
                self._default_joint_pos_policy[idx]
                + onnx_pred[idx] * self._action_scale_policy[idx]
            )
            data.ctrl[:] = targets
