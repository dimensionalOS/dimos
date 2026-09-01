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

"""Microduck ONNX gait policy: observation building and inference.

The alpha policies from pollen-robotics/microduck share one observation
contract (61 floats), documented in that repo's duck-control/src/obs.rs::

    0..3    gyro, trunk frame, rad/s
    3..6    projected gravity, trunk frame, unit vector
    6..20   joint position minus home pose (14, policy order)
    20..34  joint velocity (14)
    34..48  previous raw policy action (14)
    48..61  command: [vx, vy, vyaw, head(4), body_x, body_y, body_z,
                      body_roll, body_pitch, body_yaw]

Actions are position offsets from the home pose, applied at 50 Hz. Joint
order, home pose and action scale are read from the ONNX metadata rather
than hardcoded, so a retrained/re-exported policy keeps working as long as
it declares them.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

OBS_LEN = 61
COMMAND_LEN = 13
CONTROL_DT = 0.02  # 50 Hz, the rate every alpha policy was trained at

# Command clipping, matching the velocity ranges the walking policy was
# trained on (microduck_rl velocity task).
VX_RANGE = (-0.25, 0.3)
VY_RANGE = (-0.2, 0.2)
WZ_RANGE = (-1.5, 1.5)


class MicroduckGaitPolicy:
    """Runs an alpha-family ONNX policy against a composed MuJoCo model.

    The model may embed the robot in an arbitrary scene; joints and sensors
    are resolved by name (policy order from ONNX metadata).
    """

    def __init__(self, onnx_path: str | Path, model: mujoco.MjModel) -> None:
        import onnxruntime as ort

        self._session = ort.InferenceSession(str(onnx_path))
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name

        meta = self._session.get_modelmeta().custom_metadata_map
        try:
            self.joint_names: list[str] = meta["joint_names"].split(",")
            self.default_pose = np.array(
                [float(v) for v in meta["default_joint_pos"].split(",")], dtype=np.float32
            )
            self.action_scale = float(meta.get("action_scale", "1.0"))
        except KeyError as exc:
            raise RuntimeError(
                f"Microduck policy {onnx_path} is missing ONNX metadata {exc}; "
                "use a policy exported by microduck_rl's scripts/export.py"
            ) from exc

        n = len(self.joint_names)
        obs_dim = self._session.get_inputs()[0].shape[-1]
        if obs_dim != OBS_LEN:
            raise RuntimeError(
                f"Microduck policy expects obs dim {obs_dim}, this runner builds {OBS_LEN}; "
                "only unified-61D alpha policies are supported"
            )

        # Resolve joint addresses in policy order.
        self._qpos_adr = np.empty(n, dtype=np.int64)
        self._qvel_adr = np.empty(n, dtype=np.int64)
        for i, name in enumerate(self.joint_names):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"Microduck joint '{name}' not found in composed model")
            self._qpos_adr[i] = model.jnt_qposadr[jid]
            self._qvel_adr[i] = model.jnt_dofadr[jid]

        gyro_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")
        if gyro_sid < 0:
            raise RuntimeError("Microduck model has no 'imu_ang_vel' sensor")
        adr = int(model.sensor_adr[gyro_sid])
        self._gyro_slice = slice(adr, adr + 3)

        trunk_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
        if trunk_jid < 0:
            raise RuntimeError("Microduck model has no 'trunk_base_freejoint'")
        root_adr = int(model.jnt_qposadr[trunk_jid])
        self.root_qpos_adr = root_adr
        self._root_quat_slice = slice(root_adr + 3, root_adr + 7)

        self.last_action = np.zeros(n, dtype=np.float32)
        self._command = np.zeros(COMMAND_LEN, dtype=np.float32)

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    def set_twist(self, vx: float, vy: float, wz: float) -> None:
        self._command[0] = float(np.clip(vx, *VX_RANGE))
        self._command[1] = float(np.clip(vy, *VY_RANGE))
        self._command[2] = float(np.clip(wz, *WZ_RANGE))

    def reset(self) -> None:
        self.last_action[:] = 0.0
        self._command[:] = 0.0

    def initial_qpos(self, data: mujoco.MjData) -> None:
        """Pose the robot at the standing home pose, keeping its base x/y."""
        adr = self.root_qpos_adr
        data.qpos[adr + 2] = 0.125
        data.qpos[adr + 3 : adr + 7] = (1.0, 0.0, 0.0, 0.0)
        data.qpos[self._qpos_adr] = self.default_pose
        data.qvel[:] = 0.0

    def projected_gravity(self, data: mujoco.MjData) -> NDArray[np.float32]:
        w, x, y, z = data.qpos[self._root_quat_slice]
        quat = np.array([w, x, y, z], dtype=np.float32)
        down = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        # v rotated by quat^-1
        xyz = quat[1:4]
        t = np.cross(xyz, down) * 2.0
        return down - quat[0] * t + np.cross(xyz, t)

    def build_observation(self, data: mujoco.MjData) -> NDArray[np.float32]:
        gyro = data.sensordata[self._gyro_slice].astype(np.float32)
        gravity = self.projected_gravity(data)
        joint_pos = data.qpos[self._qpos_adr].astype(np.float32) - self.default_pose
        joint_vel = data.qvel[self._qvel_adr].astype(np.float32)
        return np.concatenate(
            [gyro, gravity, joint_pos, joint_vel, self.last_action, self._command]
        )

    def step(self, data: mujoco.MjData) -> NDArray[np.float32]:
        """One 50 Hz control step: returns position targets in policy order."""
        obs = self.build_observation(data).reshape(1, -1)
        action = self._session.run([self._output_name], {self._input_name: obs})[0]
        action = action.squeeze(0).astype(np.float32)
        self.last_action = action.copy()
        return self.default_pose + action * self.action_scale
