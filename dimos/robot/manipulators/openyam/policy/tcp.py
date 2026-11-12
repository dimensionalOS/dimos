# Copyright 2026 Dimensional Inc.
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

"""The trained controller's TCP frame, from the plant it was trained in.

``openyam.py`` scores and records motion at the ``arm_tcp`` body of the trained
MuJoCo model. The deployed URDF has a different hand, so its ``gripper`` /
``gripper_tip`` frames are *not* the trained TCP: at the same joints they sit
about 0.1-0.19 m away. The policy goal must be expressed where the actor learned
it, so this module reads the TCP straight out of the frozen plant model.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.robot.manipulators.openyam.policy.contract import PolicyContractError

#: Body the trained controller records as the tool centre point.
TCP_BODY = "arm_tcp"
#: Plant joint names, in ``JOINT_ORDER``.
PLANT_JOINTS: tuple[str, ...] = tuple(f"arm_joint_{index}" for index in range(1, 7))


class MjcfTcpFk:
    """Forward kinematics of the trained plant's TCP body."""

    def __init__(
        self,
        model_path: str | Path,
        *,
        body: str = TCP_BODY,
        joint_names: tuple[str, ...] = PLANT_JOINTS,
    ) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover - depends on the install extra
            raise RuntimeError("MjcfTcpFk requires mujoco") from exc
        self._mujoco = mujoco
        self.path = Path(model_path)
        if not self.path.is_file():
            raise PolicyContractError(f"plant model not found: {self.path}")
        self.model = mujoco.MjModel.from_xml_path(str(self.path))
        self.data = mujoco.MjData(self.model)
        self._qpos = [int(self.model.joint(name).qposadr[0]) for name in joint_names]
        self._body = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body))
        if self._body < 0:
            raise PolicyContractError(f"plant model has no body {body!r}")

    def __call__(
        self, q: NDArray[np.float64] | NDArray[np.float32]
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Position and ``wxyz`` quaternion of the TCP body at ``q``."""
        values = np.asarray(q, dtype=np.float64).reshape(-1)
        if values.shape != (len(self._qpos),):
            raise PolicyContractError(f"expected {len(self._qpos)} joints, got {values.shape}")
        if not np.isfinite(values).all():
            raise PolicyContractError("non-finite joint angles")
        self.data.qpos[self._qpos] = values
        mujoco = self._mujoco
        mujoco.mj_kinematics(self.model, self.data)
        position = np.asarray(self.data.xpos[self._body], dtype=np.float64).copy()
        x, y, z, w = (float(value) for value in self.data.xquat[self._body])
        return position, np.array([w, x, y, z], dtype=np.float64)

    def body_names(self) -> list[str]:
        mujoco = self._mujoco
        return [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, index)
            for index in range(self.model.nbody)
        ]

    @property
    def timestep_s(self) -> float:
        return float(self.model.opt.timestep)

    def actuator_gains(self) -> dict[str, Any]:
        """Position-actuator ``kp``/``kv`` and force range the plant applies."""
        mujoco = self._mujoco
        return {
            "names": [
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
                for index in range(self.model.nu)
            ],
            "kp": self.model.actuator_gainprm[:, 0].tolist(),
            "kv": (-self.model.actuator_biasprm[:, 2]).tolist(),
            "forcerange": self.model.actuator_forcerange.tolist(),
            "ctrllimited": self.model.actuator_ctrllimited.tolist(),
        }
