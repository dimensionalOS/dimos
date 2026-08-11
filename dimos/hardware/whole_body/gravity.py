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

"""Model-based gravity feedforward torque for whole-body position control."""

from __future__ import annotations

from typing import Any

import numpy as np

_GRAVITY = 9.81


class GravityFeedforward:
    """Gravity-holding torque for a subset of joints, from a robot model.

    Torques come from RNEA at the measured configuration (zero velocity and
    acceleration), plus ``J^T (0, 0, m g)`` per payload frame. The model root
    is assumed gravity-aligned; base tilt is not compensated. URDF and MJCF
    models are supported; in sim, use the sim's own MJCF so masses match.
    """

    def __init__(
        self,
        model_path: str,
        joint_map: dict[str, str],
        ff_joints: tuple[str, ...],
        payloads: tuple[tuple[str, float], ...] = (),
        scale: float = 1.0,
    ) -> None:
        self._model_path = model_path
        self._joint_map = dict(joint_map)
        self._ff_joints = tuple(ff_joints)
        self._payloads = tuple(payloads)
        self._scale = scale
        self._model: Any = None

    def _ensure_model(self) -> None:
        if self._model is not None:
            return
        import pinocchio

        self._pin = pinocchio
        if self._model_path.endswith(".xml"):
            model = pinocchio.buildModelFromMJCF(self._model_path)
        else:
            model = pinocchio.buildModelFromUrdf(self._model_path)
        self._model = model
        self._data = model.createData()
        self._idx_q: dict[str, int] = {}
        self._idx_v: dict[str, int] = {}
        for name, urdf_name in self._joint_map.items():
            if not model.existJointName(urdf_name):
                raise ValueError(f"Gravity FF joint '{urdf_name}' not in {self._model_path}")
            joint = model.joints[model.getJointId(urdf_name)]
            self._idx_q[name] = joint.idx_q
            self._idx_v[name] = joint.idx_v
        missing = [name for name in self._ff_joints if name not in self._joint_map]
        if missing:
            raise ValueError(f"Gravity FF joints missing from joint map: {missing}")
        self._payload_frames = []
        for frame, mass in self._payloads:
            if not model.existFrame(frame):
                raise ValueError(f"Gravity FF payload frame '{frame}' not in {self._model_path}")
            self._payload_frames.append((model.getFrameId(frame), mass))

    def tau(self, positions: dict[str, float]) -> dict[str, float]:
        """Holding torque for the feedforward joints at measured ``positions``."""
        self._ensure_model()
        pin = self._pin
        q = pin.neutral(self._model)
        for name, idx_q in self._idx_q.items():
            if name in positions:
                q[idx_q] = positions[name]
        torques = np.asarray(
            pin.rnea(self._model, self._data, q, np.zeros(self._model.nv), np.zeros(self._model.nv))
        )
        for frame_id, mass in self._payload_frames:
            jac = pin.computeFrameJacobian(
                self._model, self._data, q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            torques = torques + jac[:3].T @ np.array([0.0, 0.0, mass * _GRAVITY])
        return {name: self._scale * float(torques[self._idx_v[name]]) for name in self._ff_joints}
