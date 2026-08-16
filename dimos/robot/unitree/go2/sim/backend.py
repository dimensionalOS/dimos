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
MuJoCo implements it today (:class:`~dimos.robot.unitree.go2.sim.model.MujocoBackend`);
IsaacLab is the next backend, and PhysX shares no contact model with MuJoCo,
so fitted VALUES will not transfer — the recordings, anchors, and method are
what transfer.

NO ENGINE IMPORT LIVES HERE, by design: this module is the seam, and importing
the seam must never drag in a simulator — with two backends installed, using
either would import both.

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
    # The robot hung from a rope: pin the trunk to the MEASURED pose each clip
    # (it hangs 70-85 deg off level; holding it level points gravity the wrong
    # way through every leg), clear of a floor the real robot never met.
    PINNED = "pinned"


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
    """What a simulator must provide to be identified against a recording."""

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

    def apply(self, values: Mapping[str, float]) -> None:
        """Push values into the engine. Absent keys keep engine defaults."""
        ...

    def rollout(self, plan: RolloutPlan) -> Prediction:
        """Drive the plan and return what the real robot also measures."""
        ...
