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

Every published Microduck policy (walk, stand, sitstand, kicks, roulade,
ground pick, roller, roller crouch) declares the same joint order and home
pose, so the model-bound half of the contract (``MicroduckObserver``) and the
``last_action`` slot can be shared by several loaded policies; only the
13-float command differs per policy (see ``policies.py``).
``MicroduckGaitPolicy`` is the single-policy (walking) wrapper the plain
``microduck-sim`` blueprint runs.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    import mujoco

OBS_LEN = 61
COMMAND_LEN = 13
CONTROL_DT = 0.02  # 50 Hz, the rate every alpha policy was trained at

# Command clipping, matching the velocity ranges the walking policy was
# trained on (microduck_rl velocity task).
VX_RANGE = (-0.25, 0.3)
VY_RANGE = (-0.2, 0.2)
WZ_RANGE = (-1.5, 1.5)

_TRUNK_FREEJOINT = "trunk_base_freejoint"
_GYRO_SENSOR = "imu_ang_vel"


@dataclass(frozen=True)
class PolicySession:
    """One loaded ONNX policy plus the metadata the runner needs from it."""

    path: Path
    session: Any  # onnxruntime.InferenceSession (kept untyped: optional dep)
    input_name: str
    output_name: str
    joint_names: tuple[str, ...]
    default_pose: NDArray[np.float32]
    action_scale: float

    def run(self, obs: NDArray[np.float32]) -> NDArray[np.float32]:
        """Raw action (position offsets, policy joint order) for one observation."""
        action = self.session.run([self.output_name], {self.input_name: obs.reshape(1, -1)})[0]
        return np.asarray(action, dtype=np.float32).reshape(-1)


def load_policy_session(onnx_path: str | Path) -> PolicySession:
    """Load an alpha-family ONNX policy, validating its metadata and obs width."""
    import onnxruntime as ort

    path = Path(onnx_path)
    session = ort.InferenceSession(str(path))
    meta = session.get_modelmeta().custom_metadata_map
    try:
        joint_names = tuple(meta["joint_names"].split(","))
        default_pose = np.array(
            [float(v) for v in meta["default_joint_pos"].split(",")], dtype=np.float32
        )
        action_scale = float(meta.get("action_scale", "1.0"))
    except KeyError as exc:
        raise RuntimeError(
            f"Microduck policy {path} is missing ONNX metadata {exc}; "
            "use a policy exported by microduck_rl's scripts/export.py"
        ) from exc

    obs_dim = session.get_inputs()[0].shape[-1]
    if obs_dim != OBS_LEN:
        raise RuntimeError(
            f"Microduck policy {path} expects obs dim {obs_dim}, this runner builds {OBS_LEN}; "
            "only unified-61D alpha policies are supported"
        )
    if len(default_pose) != len(joint_names):
        raise RuntimeError(
            f"Microduck policy {path} declares {len(joint_names)} joints but a "
            f"{len(default_pose)}-long default pose"
        )
    return PolicySession(
        path=path,
        session=session,
        input_name=session.get_inputs()[0].name,
        output_name=session.get_outputs()[0].name,
        joint_names=joint_names,
        default_pose=default_pose,
        action_scale=action_scale,
    )


class MicroduckObserver:
    """Model-bound half of the observation contract.

    Resolves the policy's joints and sensors by name in a composed MuJoCo
    model (the robot may be embedded in an arbitrary scene) and builds the
    61-float observation from ``MjData``. Holds no per-policy state, so one
    observer serves every loaded policy.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        joint_names: Sequence[str],
        default_pose: NDArray[np.float32],
    ) -> None:
        import mujoco

        self.joint_names: list[str] = list(joint_names)
        self.default_pose = np.asarray(default_pose, dtype=np.float32)
        n = len(self.joint_names)
        if len(self.default_pose) != n:
            raise ValueError(f"default_pose has {len(self.default_pose)} entries for {n} joints")

        # Resolve joint addresses in policy order.
        self._qpos_adr = np.empty(n, dtype=np.int64)
        self._qvel_adr = np.empty(n, dtype=np.int64)
        for i, name in enumerate(self.joint_names):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"Microduck joint '{name}' not found in composed model")
            self._qpos_adr[i] = model.jnt_qposadr[jid]
            self._qvel_adr[i] = model.jnt_dofadr[jid]

        gyro_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, _GYRO_SENSOR)
        if gyro_sid < 0:
            raise RuntimeError(f"Microduck model has no '{_GYRO_SENSOR}' sensor")
        adr = int(model.sensor_adr[gyro_sid])
        self._gyro_slice = slice(adr, adr + 3)

        trunk_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, _TRUNK_FREEJOINT)
        if trunk_jid < 0:
            raise RuntimeError(f"Microduck model has no '{_TRUNK_FREEJOINT}'")
        root_adr = int(model.jnt_qposadr[trunk_jid])
        self.root_qpos_adr = root_adr
        self._root_quat_slice = slice(root_adr + 3, root_adr + 7)

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    def initial_qpos(self, data: mujoco.MjData) -> None:
        """Pose the robot at the standing home pose, keeping its base x/y."""
        adr = self.root_qpos_adr
        data.qpos[adr + 2] = 0.125
        data.qpos[adr + 3 : adr + 7] = (1.0, 0.0, 0.0, 0.0)
        data.qpos[self._qpos_adr] = self.default_pose
        data.qvel[:] = 0.0

    def root_yaw(self, data: mujoco.MjData) -> float:
        """Trunk yaw (rad) in the world frame."""
        w, x, y, z = data.qpos[self._root_quat_slice]
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def projected_gravity(self, data: mujoco.MjData) -> NDArray[np.float32]:
        w, x, y, z = data.qpos[self._root_quat_slice]
        quat = np.array([w, x, y, z], dtype=np.float32)
        down = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        # v rotated by quat^-1
        xyz = quat[1:4]
        t = np.cross(xyz, down) * 2.0
        return down - quat[0] * t + np.cross(xyz, t)

    def build(
        self,
        data: mujoco.MjData,
        last_action: NDArray[np.float32],
        command: NDArray[np.float32],
    ) -> NDArray[np.float32]:
        gyro = data.sensordata[self._gyro_slice].astype(np.float32)
        gravity = self.projected_gravity(data)
        joint_pos = data.qpos[self._qpos_adr].astype(np.float32) - self.default_pose
        joint_vel = data.qvel[self._qvel_adr].astype(np.float32)
        return np.concatenate([gyro, gravity, joint_pos, joint_vel, last_action, command])


class MicroduckGaitPolicy:
    """Runs an alpha-family ONNX policy against a composed MuJoCo model.

    The model may embed the robot in an arbitrary scene; joints and sensors
    are resolved by name (policy order from ONNX metadata).
    """

    def __init__(self, onnx_path: str | Path, model: mujoco.MjModel) -> None:
        self._policy = load_policy_session(onnx_path)
        self.joint_names: list[str] = list(self._policy.joint_names)
        self.default_pose = self._policy.default_pose
        self.action_scale = self._policy.action_scale
        self._observer = MicroduckObserver(model, self.joint_names, self.default_pose)
        self.root_qpos_adr = self._observer.root_qpos_adr

        n = len(self.joint_names)
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
        self._observer.initial_qpos(data)

    def projected_gravity(self, data: mujoco.MjData) -> NDArray[np.float32]:
        return self._observer.projected_gravity(data)

    def build_observation(self, data: mujoco.MjData) -> NDArray[np.float32]:
        return self._observer.build(data, self.last_action, self._command)

    def step(self, data: mujoco.MjData) -> NDArray[np.float32]:
        """One 50 Hz control step: returns position targets in policy order."""
        action = self._policy.run(self.build_observation(data))
        self.last_action = action.copy()
        return self.default_pose + action * self.action_scale
