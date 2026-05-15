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

"""Deterministic sinusoidal `PolicyBackend` for wiring/integration tests.

`TestPolicy` requires no model files, no GPU, and no third-party
robot-learning packages. It produces joint position commands that follow
``center + amplitude * sin(2π · frequency · t + phase)`` per joint.

The backend's "time" is the wall-clock elapsed since the most recent
`reset()` (or `initialize()`), so calling `reset()` deterministically
restarts the trajectory from phase 0.

When `center` is not provided at construction, the backend captures the
current joint positions from the next `select_action()` observation and
uses them as the sinusoid center. This avoids the "snap to zero" jump
that an absolute open-loop policy would otherwise produce on every
reset (rollout start, teleop disengage, etc.). Re-captures on every
`reset()` so the trajectory always replans from the live pose.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
import time

from dimos.manipulation.policy.command import JointPositionCommand, PolicyCommand
from dimos.manipulation.policy.observation import PolicyObservation


class TestPolicy:
    """Sinusoidal joint position backend used for tests and dry runs.

    Args:
        joint_names: Joint names to emit positions for, in order.
        amplitude: Per-joint amplitude (radians or meters depending on
            joint type). Scalar broadcasts to every joint.
        frequency: Per-joint sinusoid frequency in Hz. Scalar broadcasts.
        phase: Per-joint phase offset in radians. Scalar broadcasts.
        center: Optional per-joint center value. When provided (scalar
            or per-joint sequence), the center is **static** — the
            sinusoid always oscillates around this value. When ``None``
            (the default), the center is captured from the next
            `select_action()` observation's `joint_state` after every
            `reset()` / `initialize()`, so the trajectory always replans
            from the live joint pose instead of snapping to zero.
    """

    # Tell pytest this is not a test class despite the `Test` prefix.
    __test__ = False

    def __init__(
        self,
        *,
        joint_names: Sequence[str],
        amplitude: float | Sequence[float] = 0.1,
        frequency: float | Sequence[float] = 0.5,
        phase: float | Sequence[float] = 0.0,
        center: float | Sequence[float] | None = None,
    ) -> None:
        if not joint_names:
            raise ValueError("TestPolicy requires at least one joint name")

        self._joint_names: tuple[str, ...] = tuple(joint_names)
        n = len(self._joint_names)
        self._amplitude: tuple[float, ...] = self._broadcast(amplitude, n, "amplitude")
        self._frequency: tuple[float, ...] = self._broadcast(frequency, n, "frequency")
        self._phase: tuple[float, ...] = self._broadcast(phase, n, "phase")
        # Center mode: caller-provided centers are static; `None` means
        # auto-capture from the observation on each reset.
        self._center_from_observation: bool = center is None
        center_seq: float | Sequence[float] = 0.0 if center is None else center
        self._center: tuple[float, ...] = self._broadcast(center_seq, n, "center")
        self._center_dirty: bool = self._center_from_observation

        self._t0: float = time.monotonic()

    @staticmethod
    def _broadcast(value: float | Sequence[float], n: int, name: str) -> tuple[float, ...]:
        if isinstance(value, (int, float)):
            return tuple(float(value) for _ in range(n))
        seq = tuple(float(v) for v in value)
        if len(seq) != n:
            raise ValueError(
                f"TestPolicy.{name}: expected scalar or length-{n} sequence, got length {len(seq)}"
            )
        return seq

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    def initialize(self) -> None:
        """Reset the phase clock so the first inference starts at t=0."""
        self._t0 = time.monotonic()
        if self._center_from_observation:
            self._center_dirty = True

    def select_action(self, observation: PolicyObservation) -> PolicyCommand:
        if self._center_dirty and self._center_from_observation:
            captured = self._capture_center_from_observation(observation)
            if captured is not None:
                self._center = captured
                self._center_dirty = False
                # Restart the phase clock at the moment of capture so the
                # first sample corresponds to t≈0 (within a few µs) and
                # equals the live joint pose. Without this, t accumulates
                # during the teleop-engaged window and the policy would
                # resume at an arbitrary point along the sinusoid.
                self._t0 = time.monotonic()
            # If capture failed (no joint_state yet, or missing joints), stay
            # dirty and try again on the next tick; the sinusoid temporarily
            # oscillates around the previous center.
        return JointPositionCommand(
            joint_names=self._joint_names,
            positions=self._sample_at(time.monotonic() - self._t0),
        )

    def _capture_center_from_observation(
        self, observation: PolicyObservation
    ) -> tuple[float, ...] | None:
        """Capture the joint state as the sinusoid center.

        Returns centers adjusted by ``-amplitude * sin(phase)`` per joint so
        that ``position(t=0) = center + amplitude*sin(phase)`` collapses to
        the captured pose. This guarantees a jump-free start regardless of
        the configured phases — the policy "continues from where teleop left
        off" on every reset.
        """
        js = observation.joint_state
        if js is None or not js.name or not js.position:
            return None
        index_by_name = {n: i for i, n in enumerate(js.name)}
        centers: list[float] = []
        for joint, amp, phase in zip(self._joint_names, self._amplitude, self._phase, strict=True):
            idx = index_by_name.get(joint)
            if idx is None or idx >= len(js.position):
                return None
            pose = float(js.position[idx])
            centers.append(pose - amp * math.sin(phase))
        return tuple(centers)

    def _sample_at(self, t: float) -> tuple[float, ...]:
        two_pi = 2.0 * math.pi
        return tuple(
            c + a * math.sin(two_pi * f * t + p)
            for c, a, f, p in zip(
                self._center, self._amplitude, self._frequency, self._phase, strict=True
            )
        )

    # Exposed for unit tests so behavior can be checked deterministically
    # without sleeping.
    def sample_at(self, t: float) -> Mapping[str, float]:
        """Return the position each joint would take at time `t` (seconds
        since the last `reset()` / `initialize()`)."""
        return dict(zip(self._joint_names, self._sample_at(t), strict=True))

    def reset(self) -> None:
        """Restart the sinusoid phase clock from zero.

        When the center was constructed as auto-from-observation, also
        marks the center dirty so the next `select_action()` re-captures
        it from the live joint state — the policy "replans from here"
        after every rollout start / teleop handoff.
        """
        self._t0 = time.monotonic()
        if self._center_from_observation:
            self._center_dirty = True

    def close(self) -> None:
        """No-op — `TestPolicy` holds no external resources."""


__all__ = ["TestPolicy"]
