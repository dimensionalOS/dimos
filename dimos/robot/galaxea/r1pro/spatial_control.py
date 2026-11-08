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

"""Torso height commands arbitrated by ControlCoordinator."""

import math
import threading

import numpy as np

from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.msgs.std_msgs.Float32 import Float32


def height_step(q: np.ndarray, axis: float, dt: float, limits: np.ndarray) -> np.ndarray:
    """Move the equal-link torso vertically, preserving reach, pitch and yaw."""
    if not math.isfinite(axis) or not -1 <= axis <= 1 or not axis:
        return q.copy()
    length = 0.35
    x = length * (math.sin(q[0]) + math.sin(q[0] + q[1]))
    z = length * (math.cos(q[0]) + math.cos(q[0] + q[1]))
    if (axis < 0 and z <= 0.49) or (axis > 0 and z >= 0.7):
        return q.copy()
    target_z = z + axis * 0.025 * dt
    reach = (x * x + target_z * target_z - 2 * length * length) / (2 * length * length)
    elbow = (-1 if q[1] <= 0 else 1) * math.acos(float(np.clip(reach, -1, 1)))
    hip = math.atan2(x, target_z) - math.atan2(math.sin(elbow), 1 + math.cos(elbow))
    pitch = q[0] + q[1] - q[2]
    target = np.array([hip, elbow, hip + elbow - pitch, q[3]])
    if np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
        return q.copy()
    delta = target - q
    scale = min(1.0, 0.15 * dt / max(float(np.max(np.abs(delta))), 1e-9))
    return np.asarray(q + scale * delta)


class R1TorsoHeightTask(BaseControlTask):
    """Steer height, with upright pitch compensation yielding to active bimanual IK."""

    def __init__(self, limits: np.ndarray) -> None:
        self._name = "r1pro_torso_height"
        self._lock = threading.Lock()
        self._axis = 0.0
        self._at = 0.0
        self._estopped = False
        self._limits = limits
        self._names = [f"r1pro/torso_joint{i}" for i in range(1, 5)]

    def claim(self) -> ResourceClaim:
        return ResourceClaim(frozenset(self._names[:3]), priority=19)

    def is_active(self) -> bool:
        return True

    def on_torso_height_command(self, message: Float32, t_now: float) -> bool:
        with self._lock:
            self._axis = (
                float(message.data)
                if math.isfinite(message.data) and abs(message.data) <= 1
                else 0.0
            )
            self._at = t_now
        return True

    def set_estop(self, estopped: bool) -> None:
        with self._lock:
            self._axis = 0.0
            self._estopped = estopped

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints.intersection(self._names[:2]):
            with self._lock:
                self._axis = 0.0

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if self._estopped or not self._axis or state.t_now - self._at > 0.3:
                return None
            positions = [state.joints.get_position(n) for n in self._names]
            if any(p is None for p in positions):
                return None
            target = height_step(
                np.asarray(positions, dtype=float), self._axis, min(state.dt, 0.02), self._limits
            )
            return JointCommandOutput(self._names[:3], positions=target[:3].tolist())
