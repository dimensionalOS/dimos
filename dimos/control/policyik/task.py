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

"""Coordinated Cartesian task that owns all posture joints as one controller."""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import threading
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.control.policyik.actor import PolicyIKActor
from dimos.control.policyik.controller import PERIOD_NS, CoordinatedPolicyIK
from dimos.control.policyik.kinematics import ToolKinematics
from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)


@dataclass(frozen=True)
class PolicyIKTaskConfig:
    bundle: Path
    joint_prefix: str = ""
    priority: int = 10
    feedback_timeout: float = 0.1


class PolicyIKTask(BaseControlTask):
    """Paired chassis-frame tool goals -> measured-feedback joint references.

    The caller schedules at least 30 Hz. Faster ticks hold the last reference.
    This task must not be combined with a second arm/torso command writer.
    """

    def __init__(self, name: str, config: PolicyIKTaskConfig) -> None:
        self._name = name
        self.config = config
        self._lock = threading.RLock()
        self._controller: CoordinatedPolicyIK | None = None
        self._manifest: dict[str, Any] = {}
        self._goals: NDArray[np.float32] | None = None
        self._active = False
        self._estopped = False
        self._last_time = -float("inf")
        self._last_sample = -float("inf")
        self._result: dict[str, Any] = {}
        self._names: list[str] = []

    def start(self) -> None:
        bundle = self.config.bundle
        manifest = json.loads((bundle / "manifest.json").read_text())
        for name, key in (("actor.npz", "actor_sha256"), ("robot.urdf", "urdf_sha256")):
            if hashlib.sha256((bundle / name).read_bytes()).hexdigest() != manifest[key]:
                raise ValueError(f"Policy bundle hash mismatch: {name}")
        if len(manifest["joints"]) != 18 or len(manifest["tools"]) != 2:
            raise ValueError("Expected coordinated upper-body policy bundle")
        kinematics = ToolKinematics(
            bundle / "robot.urdf", tuple(manifest["joints"]), tuple(manifest["tools"])
        )
        controller = CoordinatedPolicyIK(PolicyIKActor(bundle / "actor.npz"), kinematics)
        with self._lock:
            self._manifest = manifest
            self._controller = controller
            self._names = [self.config.joint_prefix + n for n in manifest["joints"]]
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._controller = None

    def claim(self) -> ResourceClaim:
        with self._lock:
            return ResourceClaim(joints=frozenset(self._names), priority=self.config.priority)

    def is_active(self) -> bool:
        with self._lock:
            return self._active and not self._estopped

    def set_goals(self, goals: list[list[float]], frame: str) -> None:
        values = np.asarray(goals, dtype=np.float32)
        with self._lock:
            if frame != self._manifest.get("frame"):
                raise ValueError("Paired goals must be in the policy's fixed chassis frame")
            if values.shape != (2, 7) or not np.isfinite(values).all():
                raise ValueError("Two finite [x,y,z,qw,qx,qy,qz] goals required, left then right")
            if not np.allclose(np.linalg.norm(values[:, 3:], axis=-1), 1, atol=1e-5):
                raise ValueError("Goal quaternions must be unit length")
            self._goals = values.copy()
            self._active = True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            controller = self._controller
            if controller is None or not self._active or self._estopped:
                return None
            age = state.t_now - state.joints.timestamp
            if not 0 <= age <= self.config.feedback_timeout:
                return None
            if state.t_now < self._last_time or state.joints.timestamp < self._last_sample:
                self._active = False
                raise ValueError("Policy time went backwards; reset the task for the new run")
            if state.joints.timestamp == self._last_sample:
                return None
            q = np.array([state.joints.joint_positions[n] for n in self._names], dtype=np.float32)
            qd = np.array([state.joints.joint_velocities[n] for n in self._names], dtype=np.float32)
            if self._goals is None:
                p, r = controller.kinematics.poses(q)
                self._goals = np.concatenate((p, r), axis=-1)
            # Header codecs may truncate a float timestamp by one nanosecond.
            # Compare integer ticks with the same two-ns tolerance as Sim2's
            # G1 host; floating subtraction otherwise skips valid 30 Hz ticks.
            elapsed_ns = (
                round((state.t_now - self._last_time) * 1e9)
                if np.isfinite(self._last_time)
                else PERIOD_NS
            )
            if elapsed_ns + 2 < PERIOD_NS:
                reference = controller.reference
                if reference is None:
                    return None
            else:
                result = controller.compute(q, qd, self._goals)
                reference = result.reference
                self._last_time = state.t_now
                self._last_sample = state.joints.timestamp
                self._result = {
                    "inferences": controller.step,
                    "position_error_m": result.position_error.tolist(),
                    "orientation_error_deg": np.degrees(result.orientation_error).tolist(),
                    "tool_position": result.tool_position.tolist(),
                    "tool_quaternion_wxyz": result.tool_quaternion.tolist(),
                    "goals": self._goals.tolist(),
                    "reference": reference.tolist(),
                    "action_max_abs": float(np.abs(result.action).max()),
                    "feedback_time": state.joints.timestamp,
                }
            return JointCommandOutput(joint_names=list(self._names), positions=reference.tolist())

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        with self._lock:
            if joints & set(self._names):
                self._active = False
                if self._controller is not None:
                    self._controller.reset()
                self._last_time = self._last_sample = -float("inf")

    def set_estop(self, estopped: bool) -> None:
        with self._lock:
            self._estopped = estopped
            if estopped:
                self.reset_runtime_state(reactivate=False)

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        with self._lock:
            if self._controller is not None:
                self._controller.reset()
            self._last_time = self._last_sample = -float("inf")
            self._goals = None
            self._result = {}
            if reactivate is not None:
                self._active = reactivate
            return True

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {**self._result, "active": self._active, "estopped": self._estopped}
