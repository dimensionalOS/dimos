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

"""One local, lease-guarded actor for OpenYAM arm goals."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from hashlib import sha256
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.control.group_command import GroupCommandLease
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


@dataclass(eq=False)
class Session:
    """Task-side revocation fence independent of Layer A ownership."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    active: bool = True
    terminal_at: float | None = None
    fault: str | None = None

    def close(self, reason: str | None = None) -> None:
        with self.lock:
            if self.active:
                self.active = False
                self.fault = reason

    def submit(self, goal: Goal, sink: Callable[[Vector], None], q_des: Vector) -> bool:
        with self.lock:
            if not self.active or time.monotonic() >= goal.deadline:
                return False
            sink(q_des)
            if isinstance(goal, JointGoal) and goal.terminal and self.terminal_at is None:
                self.terminal_at = time.monotonic()
            return True


@dataclass(frozen=True, slots=True)
class Goal:
    session: Session
    q: Vector
    dq: Vector
    target: Vector
    deadline: float


@dataclass(frozen=True, slots=True)
class PoseGoal(Goal):
    """World XYZ + WXYZ."""


@dataclass(frozen=True, slots=True)
class PositionGoal(Goal):
    """World XYZ + non-authoritative anchor orientation."""


@dataclass(frozen=True, slots=True)
class JointGoal(Goal):
    """Ordered reference radians, converted to TCP only for conditioning."""

    terminal: bool = False


def goal_features(goal: Goal) -> tuple[Vector, Vector, int]:
    if isinstance(goal, JointGoal):
        return tcp_pose(goal.target), goal.target, 2
    if len(goal.target) != 7 or not np.isfinite(goal.target).all():
        raise ValueError("invalid Cartesian target")
    if not np.isclose(np.linalg.norm(goal.target[3:]), 1.0, rtol=0.0, atol=1e-5):
        raise ValueError("invalid Cartesian quaternion")
    return goal.target, (0.0,) * 6, int(isinstance(goal, PositionGoal))


class Tracker:
    """A latest-value mailbox, with at most one inference per 20 ms.

    Sink must submit synchronously to a local plant, not enqueue, publish, or
    call the coordinator. The sink raises if submission fails.
    """

    def __init__(self, actor: Actor, bounds: Bounds, sink: Callable[[Vector], None]) -> None:
        if len(bounds) != 6 or any(
            not np.isfinite((lo, hi)).all() or lo >= hi for lo, hi in bounds
        ):
            raise ValueError("tracker needs six finite ordered joint bounds")
        self._actor, self._bounds, self._sink = actor, bounds, sink
        self._condition = threading.Condition()
        self._pending: tuple[Goal, GroupCommandLease] | None = None
        self._closed = False
        self._session: Session | None = None
        self._history = np.empty((16, 19), dtype=np.float32)
        self._previous: Vector | None = None
        self._thread = threading.Thread(target=self._run, name="openyam-trimode", daemon=True)
        self._thread.start()

    def offer(self, payload: object, lease: GroupCommandLease) -> None:
        if not isinstance(payload, (PoseGoal, PositionGoal, JointGoal)):
            raise TypeError("expected an OpenYAM goal")
        with self._condition:
            if not self._closed:
                self._pending = payload, lease
                self._condition.notify()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise RuntimeError("tri-mode worker did not exit; leases must remain revoked")

    def _run(self) -> None:
        next_tick = 0.0
        while True:
            with self._condition:
                while not self._closed:
                    remaining = next_tick - time.monotonic()
                    if self._pending is not None and remaining <= 0:
                        goal, lease = self._pending
                        self._pending = None
                        break
                    self._condition.wait(remaining if remaining > 0 else None)
                else:
                    return
            next_tick = time.monotonic() + PERIOD
            try:
                self._step(goal, lease)
            except Exception:
                goal.session.close("tri-mode inference or submission failed")
                logger.exception("OpenYAM tri-mode tick failed")

    def _step(self, goal: Goal, lease: GroupCommandLease) -> None:
        now = time.monotonic()
        with goal.session.lock:
            if not goal.session.active or now >= min(goal.deadline, lease.expires_at):
                return
        if not bounded(goal.q, self._bounds) or len(goal.dq) != 6 or not np.isfinite(goal.dq).all():
            raise ValueError("invalid OpenYAM feedback")
        if isinstance(goal, JointGoal) and not bounded(goal.target, self._bounds):
            raise ValueError("out-of-range planned reference")
        tcp, ref, mode = goal_features(goal)
        if goal.session is self._session and self._previous is not None:
            history = self._history.copy()
            history[:-1] = self._history[1:]
            history[-1] = (*goal.q, *goal.dq, *self._previous, 1.0)
        else:
            history = np.tile(
                np.asarray((*goal.q, *goal.dq, *goal.q, 0.0), dtype=np.float32), (16, 1)
            )
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

        def submit() -> None:
            if goal.session.submit(goal, self._sink, q_des):
                self._session, self._history, self._previous = goal.session, history, q_des

        if not lease.try_write(submit):
            goal.session.close("tri-mode lease revoked")
