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

"""The backend seam: what a simulator must provide to be identified.

Everything above this seam — ingest, regimes, clip scheduling, channels, the
fit — is simulator-agnostic. The backend never sees a regime, a channel or a
fit: it is handed a :class:`RolloutPlan` and returns a :class:`Prediction`.
CPU MuJoCo implements it today
(:class:`~dimos.robot.unitree.go2.sim.engines.mujoco.MujocoBackend`). The
implementations live in ``engines/`` beside this module, and how much of the
plant a second one inherits depends on the engine: MJX compiles the same
:class:`mujoco.MjModel` that :mod:`~dimos.robot.unitree.go2.sim.engines.model`
builds, so the fitted VALUES carry and only the solver has to be re-verified;
PhysX shares no contact model with MuJoCo, so ``foot_solref_*`` and
``foot_solimp_*`` have no counterpart there and a PhysX backend is a
re-identification, not a port. What transfers to any of them is the same:
the recordings, the anchors, and the method.

NO ENGINE IMPORT LIVES HERE, by design: this module is the seam, and importing
the seam must never drag in a simulator — with two backends installed, using
either would import both. That is why the seam stays a leaf module and the
engines live in their own directory: importing ``...sim.backend`` gets the
protocols and cannot reach an engine.

Four expensive lessons are encoded in the types rather than left to comments:
``Knob.log`` because a bound must be judged in the parameter's own metric;
``Knob.why`` because a range without provenance is a guess; ``BaseCondition``
because a suspended robot needs a pinned trunk and no floor; and
``Prediction.imu_accel`` naming the sensor mount because reading it at the
body frame is the single easiest thing here to get silently wrong.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import enum
from typing import Protocol

import numpy as np

from dimos.robot.unitree.go2.sim.ranges import Knob

# The canonical channel names — everything the ROBOT measures, so everything a
# prediction could be scored against. The set a given backend can PREDICT is
# backend-specific (PhysX may expose no site-mounted virtual IMU and no
# delivered-torque readout) and is declared by ``Backend.channels()``, exactly
# as ``knobs()`` declares what it can vary. Scoring compares the intersection
# of what the recording has, what the backend predicts, and what the regime
# permits — see ``sysid.score``.
CHANNELS: tuple[str, ...] = ("joint", "dq", "tau", "accel", "gyro", "pos", "rot")


@dataclass(frozen=True)
class Commands:
    """Recorded joint-space commands, held zero-order between samples.

    The board's law is ``kp*(q_des-q) + kd*(dq_des-dq) + tau_ff``, and
    ``dq_des`` is NOT always zero: Unitree's built-in controller commands up
    to 20.7 rad/s. All arrays are in MuJoCo actuator order (FL FR RL RR).
    """

    t: np.ndarray  # (m,)
    q: np.ndarray  # (m, 12) target angles
    dq: np.ndarray  # (m, 12) target speeds
    kp: np.ndarray  # (m, 12)
    kd: np.ndarray  # (m, 12)
    tau_ff: np.ndarray  # (m, 12)


@dataclass(frozen=True)
class State:
    """One measured robot state, everything a backend needs to snap to it.

    The base HEIGHT is deliberately absent: the room frame's floor is unknown,
    so the backend places the lowest foot on its own floor (or clears the
    floor entirely under a pinned base). ``rot`` is yaw-stripped — flat ground
    is yaw-symmetric and the room's yaw is arbitrary.
    """

    t: float
    q: np.ndarray  # (12,) joint angles
    dq: np.ndarray  # (12,) joint speeds
    rot: np.ndarray  # (3, 3) gravity-referenced attitude, yaw stripped
    gyro: np.ndarray  # (3,) body angular rate
    v_body: np.ndarray | None = None  # (3,) base velocity in body frame; tracker only


class BaseCondition(enum.Enum):
    """What holds the trunk — the boundary condition the sim must impose."""

    FREE = "free"
    # The robot hung from a rope: hold the trunk at the MEASURED attitude
    # (it hangs 70-85 deg off level and swings — see BaseTrack), clear of a
    # floor the real robot never met. The hold must act DURING the step, not
    # as a reset after it: a post-step snap lets the whole robot free-fall
    # through mj_step, gravity cancels out of the relative leg dynamics, and
    # the legs live in a weightless plant (measured: passive pose after 3 s
    # identical with gravity on and zeroed, to 0.000000 rad).
    PINNED = "pinned"


@dataclass(frozen=True)
class BaseTrack:
    """The measured trunk attitude over the plan, for a PINNED base to follow.

    The robot swings on the rope — median 5.4 deg of attitude change within a
    0.4 s clip, p90 26 deg on the hanging recording's rope-free spans — so a
    trunk held rigid at each clip's start pose misrepresents the boundary
    condition by tens of degrees. The backend anchors this track to the
    re-initialised pose at each snap (a constant world-yaw offset), so the
    gravity direction through the legs matches the measurement at every
    sample, not just at clip starts. Without a track a PINNED base holds the
    snap pose rigidly.
    """

    t: np.ndarray  # (m,) sample times, the recording's lowstate clock
    rot: np.ndarray  # (m, 3, 3) measured trunk attitude, gravity-referenced


@dataclass(frozen=True)
class GhostTrack:
    """The recorded base pose in the ROOM frame, for a viewer to draw.

    Where a tracker exists the ghost is the measurement; where none exists the
    ghost is ABSENT rather than faked — build one with
    :func:`~dimos.robot.unitree.go2.sim.sysid.replay.ghost_track`, which
    returns ``None`` for a tracker-less recording. Viewer bookkeeping only:
    nothing scored reads it.
    """

    t: np.ndarray  # (m,) sample times, recording clock
    pos: np.ndarray  # (m, 3) base position, room frame
    rot: np.ndarray  # (m, 3, 3) base rotation, room frame


@dataclass(frozen=True)
class RolloutPlan:
    """One open-loop drive of the plant, fully determined before any physics.

    ``reinit`` is the multiple-shooting schedule: measured states the sim is
    snapped to, the first at ``t0``. The schedule is computed above the seam
    (seeded, shared across candidate plants — see ``sysid.regimes``) so every
    candidate is scored on identical clips.
    """

    t0: float
    duration: float
    commands: Commands
    reinit: Sequence[State]
    base: BaseCondition = BaseCondition.FREE
    base_track: BaseTrack | None = None  # PINNED only: measured attitude to follow


@dataclass(frozen=True)
class Prediction:
    """What the sim predicts of the signals the real robot also measures.

    Nothing here is a modelling choice: every array corresponds to a signal in
    ``rt/lowstate`` or the tracker. Two time bases on purpose — an impact is
    30-50 ms wide with a ~30 ms rise, so ``imu_accel``/``imu_gyro``/``tau``
    are sampled at the full physics rate (``at``) while the pose-rate arrays
    (``t``) match the 100 Hz the comparison has always used.

    ``reinit_*`` are the sim poses at each snap. They are bookkeeping, not
    prediction: open-loop body drift is only defined relative to the pose the
    clip started from, and the scorer needs it to place the recorded track in
    this rollout's world frame.
    """

    t: np.ndarray  # (n,) pose-rate sample times
    q: np.ndarray  # (n, 12) joint angles
    dq: np.ndarray  # (n, 12) joint speeds
    body_pos: np.ndarray  # (n, 3)
    body_rot: np.ndarray  # (n, 3, 3)
    at: np.ndarray  # (k,) physics-rate sample times
    imu_accel: np.ndarray  # (k, 3) SPECIFIC FORCE AT THE SENSOR MOUNT — the
    # model's `imu` site, 49 mm off the trunk frame. Off-axis it reads
    # alpha x r + omega x (omega x r) on top of the frame's acceleration:
    # 5-25 m/s2 during a landing, the same order as the impact itself.
    imu_gyro: np.ndarray  # (k, 3) angular rate, trunk frame
    tau: np.ndarray  # (k, 12) delivered joint torques
    reinit_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    reinit_pos: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    reinit_rot: np.ndarray = field(default_factory=lambda: np.zeros((0, 3, 3)))


class Backend(Protocol):
    """What a simulator must provide to be identified against a recording.

    A backend must be PICKLABLE: parallel fitting ships the configured
    backend object to spawned worker processes (``sysid.rollouts``,
    ``sysid.probe``), so a backend holds configuration — paths, knob values,
    an envelope — never live engine state. Compiling the engine's model per
    rollout (as :class:`~dimos.robot.unitree.go2.sim.engines.mujoco.MujocoBackend`
    does) satisfies this for free.
    """

    @property
    def name(self) -> str: ...

    @property
    def timestep(self) -> float:
        """The engine's physics step, s. Reinit times are quantized to it:
        a snap can only happen on a step boundary, and the measured state must
        be sampled at the exact time the snap will fire."""
        ...

    def knobs(self) -> Mapping[str, Knob]:
        """The parameters THIS engine exposes. The set is data, not code."""
        ...

    def channels(self) -> frozenset[str]:
        """Which of :data:`CHANNELS` this engine's predictions carry.

        Symmetric with :meth:`knobs`: what a backend can VARY and what it can
        PREDICT are both declarations, and the fit intersects the latter with
        what the recording measured. A ``Prediction`` field outside this set
        may be zero-length; nothing reads it.
        """
        ...

    def apply(self, values: Mapping[str, float]) -> None:
        """Push values into the engine. Absent keys keep engine defaults."""
        ...

    def rollout(self, plan: RolloutPlan) -> Prediction:
        """Drive the plan and return what the real robot also measures."""
        ...


@dataclass(frozen=True)
class LoopState:
    """The instantaneous state a closed-loop driver reads between steps.

    Exactly what the robot's own estimator hands its policy — nothing an
    engine could expose but a robot could not. Arrays are fresh copies: a
    session must never hand out views into live engine memory.
    """

    pos: np.ndarray  # (3,) base position, sim world
    quat: np.ndarray  # (4,) base attitude, wxyz
    q: np.ndarray  # (n_joints,) joint angles, actuator order
    dq: np.ndarray  # (n_joints,) joint speeds
    gyro: np.ndarray  # (3,) base angular rate, as an onboard gyro reads it


class LoopSession(Protocol):
    """One closed-loop episode: the engine's half of Mode B.

    Mode A's :class:`RolloutPlan` is a complete instruction sheet decided
    before any physics — which a policy IN the loop deliberately cannot be.
    So the closed-loop seam is a stepping primitive instead: the generic
    driver (:func:`~dimos.robot.unitree.go2.sim.sysid.ground.rollout_policy`)
    owns the policy, the observation build, the command slew and every loop
    mechanism; the session only steps physics and reports state. A second
    backend implements THIS, and inherits the whole driver.
    """

    @property
    def timestep(self) -> float:
        """The engine's physics step, s."""
        ...

    def state(self) -> LoopState:
        """The current state, as fresh copies."""
        ...

    def step(self, ctrl: np.ndarray) -> bool:
        """Apply per-joint torques and advance one physics step.

        Returns ``False`` when the episode cannot continue (an attached
        viewer was closed); a headless session always returns ``True``. A
        viewing session paces itself to wall clock here — THE VIEWER AND THE
        HEADLESS RUN ARE THE SAME FUNCTION.
        """
        ...

    def snap(self, state: State) -> None:
        """Re-initialise the episode to a measured state, mid-run — the
        divergence-rate instrument's multiple shooting. Mode A's placement
        convention, except the sim's current world yaw and x/y are
        preserved (flat ground is symmetric to both), keeping the
        trajectory continuous."""
        ...

    def show_ghost(self, pos: np.ndarray, quat: np.ndarray) -> None:
        """Place the recorded-pose ghost, sim world frame. Visual only."""
        ...

    def close(self) -> None:
        """Release the episode (detach any viewer)."""
        ...


class ClosedLoopBackend(Backend, Protocol):
    """A backend that can also put a policy IN the loop (Mode B).

    A declared capability, exactly like :meth:`Backend.channels`: a backend
    that cannot host a closed loop simply does not implement this protocol,
    and loop 2 asks for a :class:`ClosedLoopBackend` rather than discovering
    the gap by exception.
    """

    def session(
        self,
        pose: np.ndarray,
        *,
        ghost: bool = False,
        view: bool = False,
        view_speed: float = 1.0,
    ) -> LoopSession:
        """Start an episode: currently applied knobs, joints at ``pose``.

        The base starts where the engine's nominal standing pose puts it;
        ``ghost`` compiles in the recorded-pose ghost body; ``view`` attaches
        a viewer paced by ``view_speed``.
        """
        ...
