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

"""The Go2 actuator, as measured: the board's PD law, limits, envelope, lag.

The chain, in the order the hardware applies it and every replay must too:
PD demand -> clip to :data:`TORQUE_LIMITS` -> speed envelope -> first-order lag.
"""

from __future__ import annotations

from dataclasses import dataclass
import itertools

import numpy as np

# Unitree SDK LowCmd.motor_cmd / LowState.motor_state order for the 12 leg
# motors (indices 12-19 are unused on a Go2). Lives here rather than in
# backend/model.py so ingest can map wire order without importing mujoco.
UNITREE_MOTOR_NAMES: tuple[str, ...] = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)

# menagerie unitree_go2 actuator order — the joint order everything here uses.
MUJOCO_ACTUATOR_NAMES: tuple[str, ...] = (
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
)

# unitree_vec[UNITREE_TO_MUJOCO] == mujoco_vec
UNITREE_TO_MUJOCO: tuple[int, ...] = tuple(
    UNITREE_MOTOR_NAMES.index(n) for n in MUJOCO_ACTUATOR_NAMES
)

# Per-joint torque limits (hip, thigh, calf) x 4. Slightly tighter than the
# MJCF ctrlrange, so they bind first.
TORQUE_LIMITS = np.array([23.0, 23.0, 35.0] * 4)


def pd_torque(
    q_des: np.ndarray,
    dq_des: np.ndarray | float,
    kp: np.ndarray,
    kd: np.ndarray,
    tau_ff: np.ndarray,
    q: np.ndarray,
    dq: np.ndarray,
) -> np.ndarray:
    """The board's law: ``kp*(q_des-q) + kd*(dq_des-dq) + tau_ff``, unclipped.

    ``dq_des`` is NOT always zero. Our own executor sends zeros, but Unitree's
    built-in controller commands up to 20.7 rad/s with kd up to 1.8 — tens of
    Nm against a 41 Nm peak. Dropping it silently halves the damping term on
    exactly the recordings that carry the jumps.
    """
    out: np.ndarray = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
    return out


@dataclass(frozen=True)
class TorqueEnvelope:
    """What the real drive can deliver, as a function of joint speed.

    Above 3 rad/s a Go2 joint returns about half its demand (R8-R3, R8-SYSID:
    two recordings, monotone, n > 100k). Two measured mechanisms:

    ``gain`` — multiplicative derate ``g(|dq|)`` on the whole PD output; the
    term that carries the effect (position and damping components derate
    together, the signature of an output gain, not kd-style damping).

    ``ceiling`` — absolute back-EMF / voltage-limit bound, N·m. Real but
    narrow: identified only above ~4.5 rad/s, flat at 5.5 ± 0.5 N·m to
    20 rad/s. Alone it cannot reproduce the 3-6 rad/s droop.

    ``brake_gain`` — optional; replaces ``gain`` where the request OPPOSES the
    motion (``requested*dq < 0``): back-EMF subtracts from drive voltage only
    when torque acts WITH rotation, and the measured brake/drive ratio is
    1.10-1.33, rising with speed.

    Both curves are piecewise-linear on their own knot grids (identified on
    different parts of the speed range), non-increasing, flat past the last
    knot. Below 1 rad/s the measured delivered/demanded ratio is ~1.03 — a
    tau_est scale offset deliberately NOT folded into ``gain``.
    """

    name: str
    gain_speed: tuple[float, ...]
    gain: tuple[float, ...]
    ceiling_speed: tuple[float, ...]
    ceiling: tuple[float, ...]
    brake_gain: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        checks = [
            ("gain", self.gain_speed, self.gain),
            ("ceiling", self.ceiling_speed, self.ceiling),
        ]
        if self.brake_gain is not None:
            checks.append(("brake_gain", self.gain_speed, self.brake_gain))
        for what, knots, vals in checks:
            if len(knots) != len(vals) or len(knots) < 2:
                raise ValueError(f"{self.name}: {what} needs equal-length knots, at least two")
            if any(b <= a for a, b in itertools.pairwise(knots)):
                raise ValueError(f"{self.name}: {what} speed knots must be strictly ascending")
            if any(b > a for a, b in itertools.pairwise(vals)):
                raise ValueError(f"{self.name}: {what} must be non-increasing")
        if not all(0.0 < g <= 1.0 for g in self.gain):
            raise ValueError(f"{self.name}: gain must lie in (0, 1]")
        if self.brake_gain is not None and not all(0.0 < g <= 1.0 for g in self.brake_gain):
            raise ValueError(f"{self.name}: brake_gain must lie in (0, 1]")
        if not all(c > 0.0 for c in self.ceiling):
            raise ValueError(f"{self.name}: ceiling must be positive N.m")

    def deliverable(self, requested: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """The torque the drive would actually produce for this request, N·m."""
        speed = np.abs(dq)
        gain = np.interp(speed, self.gain_speed, self.gain)
        if self.brake_gain is not None:
            gain = np.where(
                requested * dq < 0.0,
                np.interp(speed, self.gain_speed, self.brake_gain),
                gain,
            )
        derated = requested * gain
        limit = np.interp(speed, self.ceiling_speed, self.ceiling)
        out: np.ndarray = np.clip(derated, -limit, limit)
        return out


# The measured envelope, as three named variants: the droop's EXISTENCE is
# robust, its SIZE only to ±0.15 at 3-6 rad/s, so the uncertainty ships as
# named plants and every result that uses one must name it. "central" is the
# only variant whose replayed delivered/demanded ratio lands in-band in every
# speed bin of both recordings.
_GAIN_SPEEDS = (0.0, 2.0, 4.5, 8.0, 14.0)
_CEIL_SPEEDS = (0.0, 8.0, 10.0, 30.0)
_NO_CEILING = float(TORQUE_LIMITS.max())
TORQUE_ENVELOPES: dict[str, TorqueEnvelope] = {
    e.name: e
    for e in (
        TorqueEnvelope(
            "conservative",
            _GAIN_SPEEDS,
            (1.0, 1.00, 0.70, 0.60, 0.56),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 6.10, 6.10),
        ),
        TorqueEnvelope(
            "central",
            _GAIN_SPEEDS,
            (1.0, 0.95, 0.55, 0.45, 0.41),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 5.50, 5.50),
        ),
        TorqueEnvelope(
            "aggressive",
            _GAIN_SPEEDS,
            (1.0, 0.80, 0.40, 0.30, 0.26),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 4.90, 4.90),
        ),
        # "central" driving with the braking quadrant measured separately; the
        # measured brake/drive ratio applied to central reproduces
        # "conservative" to within 0.01 at every knot.
        TorqueEnvelope(
            "central-signed",
            _GAIN_SPEEDS,
            (1.0, 0.95, 0.55, 0.45, 0.41),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 5.50, 5.50),
            brake_gain=(1.0, 1.00, 0.70, 0.60, 0.56),
        ),
    )
}


def actuator_step(
    applied: np.ndarray,
    requested: np.ndarray,
    dt: float,
    tau: float,
    *,
    dq: np.ndarray | None = None,
    envelope: TorqueEnvelope | None = None,
) -> np.ndarray:
    """One first-order step of the motor toward the requested torque.

    ``tau`` is the current-loop time constant, s; zero is the ideal MuJoCo
    motor. An ``envelope`` (with the joint speeds ``dq``) first reduces the
    request to what the drive can source at that speed — BEFORE the lag, so an
    over-ceiling request saturates the current loop rather than being tracked
    through it.
    """
    if envelope is not None:
        if dq is None:
            raise ValueError("actuator_step: an envelope needs the joint speeds dq")
        requested = envelope.deliverable(requested, dq)
    if tau <= 0.0:
        return requested
    alpha = dt / (tau + dt)
    stepped: np.ndarray = applied + alpha * (requested - applied)
    return stepped
