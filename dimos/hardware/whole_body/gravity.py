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

import math
from typing import Any

import numpy as np

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_GRAVITY = 9.81


class GravityFeedforward:
    """Gravity-holding torque for a subset of joints, from a robot model.

    Torques come from RNEA at the measured configuration (zero velocity and
    acceleration), plus ``J^T (0, 0, m g)`` per payload frame. The model root
    is assumed gravity-aligned; base tilt is not compensated. URDF and MJCF
    models are supported; in sim, use the sim's own MJCF so masses match.

    Output is clamped to each joint's model effort limit. Feedforward torque
    is added open-loop on top of the PD term, so a scale calibrated against
    one payload can exceed a weaker joint's motor at another configuration --
    the G1's wrists take 5 Nm against the shoulders' 25.
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
        self._clamped: set[str] = set()

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
        # MJCF joints often carry no effort limit; those read 0 or inf here
        # and are left unclamped rather than pinned to zero torque.
        self._effort_limit: dict[str, float] = {}
        for name in self._ff_joints:
            limit = float(model.effortLimit[self._idx_v[name]])
            if np.isfinite(limit) and limit > 0.0:
                self._effort_limit[name] = limit

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
        return {
            name: self._clamp(name, self._scale * float(torques[self._idx_v[name]]))
            for name in self._ff_joints
        }

    def _clamp(self, name: str, tau: float) -> float:
        limit = self._effort_limit.get(name)
        if limit is None or abs(tau) <= limit:
            return tau
        if name not in self._clamped:
            self._clamped.add(name)
            logger.warning(
                "Gravity feedforward exceeds the joint's model effort limit; clamping. "
                "Lower gravity_ff_scale, or drop the payload.",
                joint=name,
                requested_nm=round(tau, 2),
                limit_nm=limit,
                scale=self._scale,
            )
        return math.copysign(limit, tau)
