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

"""Station-frame forward kinematics for the dual Metal policy.

The trained controller observes and is given goals for two TCP sites in the
station frame. Those sites are not the repo's planning tip: each sits 60 mm out
along the gripper-base frame's local +Z. The mount transform also differs from
this repo's example bench layout (training mounts the left arm at -Y). Both are
taken from :class:`StationGeometry` and verified against recorded station poses
in ``test_station.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    JOINT_ORDER,
    SIDES,
    StationGeometry,
)


def rpy_to_matrix(rpy: tuple[float, float, float]) -> NDArray[np.float64]:
    """URDF fixed-axis roll-pitch-yaw to a rotation matrix (Rz @ Ry @ Rx)."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


@dataclass
class MetalStationKinematics:
    """Pinocchio FK of the two station-mounted Metal arms.

    The URDF chain is this repo's pinned vendor Metal description; only the
    mount transform and TCP offset come from :class:`StationGeometry`, which
    keeps the geometry explicit instead of reusing the default bench layout.
    """

    geometry: StationGeometry = field(default_factory=StationGeometry)
    _model: object = field(default=None, init=False, repr=False)
    _data: object = field(default=None, init=False, repr=False)
    _q_indices: list[int] = field(default_factory=list, init=False, repr=False)
    _frame_ids: list[int] = field(default_factory=list, init=False, repr=False)
    _tcp_offset: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64), init=False, repr=False
    )

    def __post_init__(self) -> None:
        self._build()

    def _build(self) -> None:
        try:
            import pinocchio
        except ImportError as exc:  # pragma: no cover - depends on the install extra
            raise RuntimeError(
                "Metal station FK requires pinocchio; install the repository's planning extras"
            ) from exc

        from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
        from dimos.robot.manipulators.dual_makermods_metal.config import dual_metal_model_config

        config = dual_metal_model_config(
            left_xyz=self.geometry.left_mount_xyz,
            right_xyz=self.geometry.right_mount_xyz,
            left_rpy=self.geometry.left_mount_rpy,
            right_rpy=self.geometry.right_mount_rpy,
        )
        loaded = config.model.load()
        prepared = prepare_urdf_for_drake(loaded, convert_meshes=False)
        model = pinocchio.buildModelFromXML(prepared.xml)
        for name in JOINT_ORDER:
            joint_id = model.getJointId(name)
            if joint_id == 0:
                raise RuntimeError(f"station URDF has no joint {name!r}")
            self._q_indices.append(int(model.joints[joint_id].idx_q))
        for side in SIDES:
            frame_id = model.getFrameId(f"{side}_gripper_base")
            if frame_id >= len(model.frames):
                raise RuntimeError(f"station URDF has no frame {side}_gripper_base")
            self._frame_ids.append(int(frame_id))
        self._model = model
        self._data = model.createData()
        self._tcp_offset = np.asarray(self.geometry.tcp_offset_xyz, dtype=np.float64)

    def pose24(self, q: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]:
        """Return ``[left_xyz, right_xyz, left_R, right_R]`` in the station frame."""
        import pinocchio

        values = np.asarray(q, dtype=np.float64).reshape(-1)
        if values.shape != (len(JOINT_ORDER),):
            raise ValueError(f"expected {len(JOINT_ORDER)} joint angles, got {values.shape}")
        if not np.isfinite(values).all():
            raise ValueError("non-finite joint angles")
        q_model = np.zeros(self._model.nq)  # type: ignore[attr-defined]
        q_model[self._q_indices] = values
        pinocchio.forwardKinematics(self._model, self._data, q_model)  # type: ignore[arg-type]
        pinocchio.updateFramePlacements(self._model, self._data)  # type: ignore[arg-type]
        out = np.empty(24, dtype=np.float64)
        for index, frame_id in enumerate(self._frame_ids):
            placement = self._data.oMf[frame_id]  # type: ignore[attr-defined]
            rotation = np.asarray(placement.rotation, dtype=np.float64)
            position = (
                np.asarray(placement.translation, dtype=np.float64) + rotation @ self._tcp_offset
            )
            out[index * 3 : index * 3 + 3] = position
            start = 6 + index * 9
            out[start : start + 9] = rotation.reshape(-1)
        return out
