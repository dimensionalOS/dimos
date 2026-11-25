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

"""One connection-owned actor for OpenYAM arm goals."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()
MODEL_SHA256 = "8ac35d59dc94d04361286e7ac07679da2e623c60ea458d4ef6e0513482cea98f"
PERIOD = 0.020
Vector = tuple[float, ...]
Bounds = tuple[tuple[float, float], ...]
Actor = Callable[["NDArray[np.float32]"], "NDArray[np.float32]"]

# Fixed-base arm_tcp body chain from the browser scene, in metres.
_OFFSETS = (
    (0, 0, 0),
    (0.02, 2.51e-5, 0.0409),
    (-0.264, -2e-6, 0),
    (0.245, 2.2e-6, 0.0600003),
    (0.0739968, -2.37e-5, 0.0395003),
    (0.0353001, -1e-7, -0.0394999),
)
_AXES = np.array(
    (
        (0, 0, 1),
        (-7e-6, 1, -4e-6),
        (7e-6, -1, 6e-6),
        (7e-6, -1, 6e-6),
        (4e-6, -6e-6, -1),
        (1, 4e-6, 7e-6),
    )
)
_AXES /= np.linalg.norm(_AXES, axis=1, keepdims=True)


def tcp_pose(q: Vector) -> Vector:
    """Return world arm_tcp XYZ + WXYZ for the fixed browser plant."""
    xyz = np.zeros(3)
    rotation = Rotation.identity()
    for offset, axis, angle in zip(_OFFSETS, _AXES, q, strict=True):
        xyz += rotation.apply(offset)
        rotation = rotation * Rotation.from_rotvec(axis * angle)
    xyz += rotation.apply((0.13, 5e-7, 1e-6))
    return tuple(
        np.concatenate((xyz, rotation.as_quat(canonical=True, scalar_first=True))).tolist()
    )


def bounded(q: Vector, bounds: Bounds) -> bool:
    return len(q) == len(bounds) == 6 and all(
        lo <= value <= hi for value, (lo, hi) in zip(q, bounds, strict=True)
    )


def load_actor(path: Path) -> Actor:
    """Load only the pinned local graph, on CPU, without a model download."""
    import onnxruntime as ort  # Optional dependency of the opt-in path.

    content = path.read_bytes()
    if sha256(content).hexdigest() != MODEL_SHA256:
        raise ValueError("OpenYAM tri-mode model digest mismatch")
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(
        content, sess_options=options, providers=["CPUExecutionProvider"]
    )
    for nodes, name, width in (
        (session.get_inputs(), "obs", 318),
        (session.get_outputs(), "q_des", 6),
    ):
        if len(nodes) != 1 or (nodes[0].name, nodes[0].type, nodes[0].shape) != (
            name,
            "tensor(float)",
            [1, width],
        ):
            raise ValueError(f"Unexpected tri-mode {name} ABI")

    def infer(obs: NDArray[np.float32]) -> NDArray[np.float32]:
        return session.run(["q_des"], {"obs": obs})[0]

    return infer


# Same-host monotonic timestamps. Ingress must never refresh a reference's deadline.
REFERENCE_TIMEOUT = 0.10


@dataclass(frozen=True, slots=True)
class Goal:
    session: str
    issued_at: float
    deadline: float
    target: Vector


@dataclass(frozen=True, slots=True)
class PoseGoal(Goal):
    """OpenYAM base-frame arm_tcp XYZ + WXYZ."""


@dataclass(frozen=True, slots=True)
class PositionGoal(Goal):
    """Base-frame arm_tcp XYZ with the grip anchor orientation for conditioning."""


@dataclass(frozen=True, slots=True)
class JointGoal(Goal):
    """Full reference radians, ordered as OPENYAM_ARM_JOINTS."""

    terminal: bool = False


@dataclass(frozen=True, slots=True)
class ArmSample:
    """Host SDK poll; not a hardware acquisition timestamp or motor receipt."""

    at: float
    q: Vector
    dq: Vector
    session: str | None
    terminal_at: float | None
    fault: str | None = None


def goal_features(goal: Goal) -> tuple[Vector, Vector, int]:
    if isinstance(goal, JointGoal):
        return tcp_pose(goal.target), goal.target, 2
    if len(goal.target) != 7 or not np.isfinite(goal.target).all():
        raise ValueError("invalid Cartesian target")
    if not np.isclose(np.linalg.norm(goal.target[3:]), 1.0, rtol=0.0, atol=1e-5):
        raise ValueError("invalid Cartesian quaternion")
    return goal.target, (0.0,) * 6, int(isinstance(goal, PositionGoal))


class Tracker:
    """Latest-value mailbox; ONNX runs outside the connection's device lock."""

    def __init__(
        self,
        actor: Actor,
        bounds: Bounds,
        prepare: Callable[[Goal], ArmSample | None],
        submit: Callable[[Goal, ArmSample, Vector], bool],
        fail: Callable[[Goal], None],
    ) -> None:
        if len(bounds) != 6 or any(
            not np.isfinite((lo, hi)).all() or lo >= hi for lo, hi in bounds
        ):
            raise ValueError("tracker needs six finite ordered joint bounds")
        self._actor, self._bounds = actor, bounds
        self._prepare, self._submit, self._fail = prepare, submit, fail
        self._condition = threading.Condition()
        self._pending: Goal | None = None
        self._closed = False
        self._session: str | None = None
        self._history = np.empty((16, 19), dtype=np.float32)
        self._previous: Vector | None = None
        self._thread = threading.Thread(target=self._run, name="openyam-trimode", daemon=True)
        self._thread.start()

    def offer(self, goal: Goal) -> None:
        with self._condition:
            if not self._closed:
                self._pending = goal
                self._condition.notify()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise RuntimeError("tri-mode worker did not exit; authority remains revoked")

    def _run(self) -> None:
        next_tick = 0.0
        while True:
            with self._condition:
                while not self._closed:
                    remaining = next_tick - time.monotonic()
                    if self._pending is not None and remaining <= 0:
                        goal, self._pending = self._pending, None
                        break
                    self._condition.wait(remaining if remaining > 0 else None)
                else:
                    return
            next_tick = time.monotonic() + PERIOD
            try:
                self._step(goal)
            except Exception:
                logger.exception("OpenYAM tri-mode tick failed")
                self._fail(goal)

    def _step(self, goal: Goal) -> None:
        sample = self._prepare(goal)
        if sample is None:
            return
        q, dq = sample.q, sample.dq
        if not bounded(q, self._bounds) or len(dq) != 6 or not np.isfinite(dq).all():
            raise ValueError("invalid OpenYAM feedback")
        if isinstance(goal, JointGoal) and not bounded(goal.target, self._bounds):
            raise ValueError("out-of-range planned reference")
        tcp, ref, mode = goal_features(goal)
        if goal.session == self._session and self._previous is not None:
            history = self._history.copy()
            history[:-1] = self._history[1:]
            history[-1] = (*q, *dq, *self._previous, 1.0)
        else:
            history = np.tile(np.asarray((*q, *dq, *q, 0.0), dtype=np.float32), (16, 1))
            history[-1, -1] = 1.0
        obs = np.concatenate((history.ravel(), np.asarray((*tcp, *ref, mode), dtype=np.float32)))
        if not np.isfinite(obs).all():
            raise ValueError("non-finite actor input")
        output = self._actor(obs[None, :])
        if output.dtype != np.float32 or output.shape != (1, 6):
            raise ValueError("invalid q_des shape or dtype")
        q_des = tuple(output[0].tolist())
        if not bounded(q_des, self._bounds):
            raise ValueError("non-finite or out-of-range q_des")
        if self._submit(goal, sample, q_des):
            self._session, self._history, self._previous = goal.session, history, q_des
