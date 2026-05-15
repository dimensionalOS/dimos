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
``center + amplitude * sin(2π · frequency · t + phase - phase_offset)``
per joint.

The backend's "time" is the wall-clock elapsed since the most recent
`reset()` (or `initialize()`), so calling `reset()` deterministically
restarts the trajectory from phase 0.

When `center` is not provided at construction, the backend captures the
current joint positions on the next `select_action()` and uses them as
the sinusoid center; the configured per-joint phase is also captured as
a `phase_offset` so the effective phase at the capture moment is zero.
This means the wave starts at the live pose (no jump) AND oscillates
symmetrically around it in `[pose - amplitude, pose + amplitude]`,
rather than sweeping a full `2 · amplitude` to one side as the naive
offset-only fix would. Re-captures on every `reset()` so the
trajectory always replans from the live pose. The configured
per-joint phases therefore only seed the initial swing direction;
after the first capture all joints are phase-aligned at sin=0.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
import time

from dimos.manipulation.policy.command import (
    JointPositionCommand,
    NoOpCommand,
    PolicyCommand,
)
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
        # Per-joint phase offset captured on each reset (auto-capture mode
        # only). Subtracted from the configured phase so the effective phase
        # at the capture moment is zero — the wave starts at the live pose
        # AND stays symmetric around it. Stays at zero in explicit-center
        # mode so caller-provided phases keep their meaning.
        self._phase_offset: tuple[float, ...] = tuple(0.0 for _ in range(n))

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
        """Mark the trajectory dirty so the next select_action regenerates it.

        See `reset()` — same semantics; `initialize()` is just the lifecycle
        entry that runs once on `PolicyModule.start()`.
        """
        self._mark_dirty()

    def select_action(self, observation: PolicyObservation) -> PolicyCommand:
        if self._center_dirty:
            if not self._center_from_observation:
                # Explicit-center mode: no pose to capture; just anchor the
                # phase clock now and start sampling.
                self._t0 = time.monotonic()
                self._center_dirty = False
            else:
                captured = self._capture_center_from_observation(observation)
                if captured is None:
                    # No live joint state yet (or one of the configured
                    # joints is missing from it). Decline this tick rather
                    # than commanding the arm to a stale-trajectory pose;
                    # we'll retry on the next inference call.
                    return NoOpCommand(
                        reason="TestPolicy: awaiting joint_state to anchor trajectory"
                    )
                self._center = captured
                # Zero out the effective phase at the capture moment so the
                # first sample equals `center == captured_pose` AND the wave
                # then oscillates symmetrically around it. Without this, the
                # wave would still START at the pose (because the captured
                # center compensates) but would sweep a full `2·amplitude`
                # to one side as it walked through `sin(2π·f·t + phase)`
                # from its non-zero starting phase — the "extreme motion"
                # failure mode.
                self._phase_offset = self._phase
                self._center_dirty = False
                # Anchor the phase clock to the same moment as the capture,
                # so the first sample lands at t≈0 (within a few µs) and
                # equals the live joint pose.
                self._t0 = time.monotonic()
        return JointPositionCommand(
            joint_names=self._joint_names,
            positions=self._sample_at(time.monotonic() - self._t0),
        )

    def _capture_center_from_observation(
        self, observation: PolicyObservation
    ) -> tuple[float, ...] | None:
        """Snapshot the joint state as the per-joint sinusoid center.

        Pairs with the `_phase_offset` capture in `select_action` so that
        ``position(t=0) = center + amplitude·sin(phase - phase) = center =
        captured pose`` AND ``position(t>0) ∈ [pose - amplitude, pose +
        amplitude]`` — symmetric around the live pose, no jump.
        """
        js = observation.joint_state
        if js is None or not js.name or not js.position:
            return None
        index_by_name = {n: i for i, n in enumerate(js.name)}
        centers: list[float] = []
        for joint in self._joint_names:
            idx = index_by_name.get(joint)
            if idx is None or idx >= len(js.position):
                return None
            centers.append(float(js.position[idx]))
        return tuple(centers)

    def _sample_at(self, t: float) -> tuple[float, ...]:
        two_pi = 2.0 * math.pi
        return tuple(
            c + a * math.sin(two_pi * f * t + p - po)
            for c, a, f, p, po in zip(
                self._center,
                self._amplitude,
                self._frequency,
                self._phase,
                self._phase_offset,
                strict=True,
            )
        )

    # Exposed for unit tests so behavior can be checked deterministically
    # without sleeping.
    def sample_at(self, t: float) -> Mapping[str, float]:
        """Return the position each joint would take at time `t` (seconds
        since the last `reset()` / `initialize()`)."""
        return dict(zip(self._joint_names, self._sample_at(t), strict=True))

    def reset(self) -> None:
        """Mark the trajectory dirty so the next `select_action()` regenerates it.

        The trajectory state — center, phase_offset, and `_t0` — is
        recomputed inside the next `select_action()` from the live joint
        observation, NOT from any state captured at `reset()` time. The
        arm can move between a `reset()` call (e.g., teleop engage,
        rollout start) and the inference tick that consumes it; anchoring
        the trajectory at the inference tick guarantees the wave starts
        at the arm's pose at that moment, not at the pose from when
        reset was called.
        """
        self._mark_dirty()

    def _mark_dirty(self) -> None:
        self._center_dirty = True

    def close(self) -> None:
        """No-op — `TestPolicy` holds no external resources."""


__all__ = ["TestPolicy"]
