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


from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.g1.velocity_policy import G1VelocityController
from dimos.sim2.physics import BodyState, InertialState, RobotState
from dimos.simulation.mujoco.input_controller import InputController
from dimos.simulation.policy import OnnxPolicy

if TYPE_CHECKING:
    import mujoco


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
        self._policy = OnnxPolicy(policy_path)
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
            onnx_pred = self._policy.predict(obs)
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


class G1OnnxController:
    """Legacy MuJoCo observations adapted to the same neutral G1 policy."""

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
        self._input_controller = input_controller
        self._prefix = naming_prefix
        self._qpos = qpos_indices
        self._dofs = dof_indices
        self._controller = G1VelocityController(
            ctrl_dt / n_substeps,
            policy_path=policy_path,
            default_angles=default_angles.tolist(),
            control_dt=ctrl_dt,
            action_scale=action_scale,
            drift_compensation=drift_compensation or (0.0, 0.0, 0.0),
        )

    def _state(self, model: mujoco.MjModel, data: mujoco.MjData) -> RobotState:
        qpos = np.arange(7, model.nq) if self._qpos is None else self._qpos
        dofs = np.arange(6, model.nv) if self._dofs is None else self._dofs
        site = model.site(self._prefix + "imu_in_pelvis").id
        rotation = data.site_xmat[site].reshape(3, 3)
        orientation = tuple(Rotation.from_matrix(rotation).as_quat())
        local_velocity = data.sensor(self._prefix + "local_linvel_pelvis").data
        local_gyro = data.sensor(self._prefix + "gyro_pelvis").data
        local_accel = data.sensor(self._prefix + "accelerometer_pelvis").data
        body = data.body(self._prefix + "pelvis")
        world_gyro = rotation @ local_gyro
        world_velocity = rotation @ local_velocity - np.cross(
            world_gyro, data.site_xpos[site] - body.xpos
        )
        return RobotState(
            joint_names=tuple(model.joint(int(model.dof_jntid[i])).name for i in dofs),
            position=tuple(data.qpos[qpos]),
            velocity=tuple(data.qvel[dofs]),
            effort=tuple(data.qfrc_actuator[dofs]),
            base=BodyState(
                position=tuple(body.xpos),
                orientation=tuple(np.roll(body.xquat, -1)),
                linear_velocity=tuple(world_velocity),
                angular_velocity=tuple(world_gyro),
            ),
            imu=InertialState(
                orientation=orientation,
                angular_velocity=tuple(local_gyro),
                linear_velocity=tuple(local_velocity),
                linear_acceleration=tuple(local_accel),
            ),
        )

    def _command(self) -> Twist:
        x, y, yaw = self._input_controller.get_command()
        return Twist(linear=(x, y, 0.0), angular=(0.0, 0.0, yaw))

    def get_obs(self, model: mujoco.MjModel, data: mujoco.MjData) -> NDArray[np.float32]:
        return self._controller.observation(self._state(model, data), self._command())

    def get_action(self, model: mujoco.MjModel, data: mujoco.MjData) -> NDArray[np.float64]:
        command = self._controller.step(self._state(model, data), self._command())
        return np.asarray(command.q, dtype=np.float64)

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        data.ctrl[:] = self.get_action(model, data)

    def reset(self) -> None:
        self._controller.reset()
