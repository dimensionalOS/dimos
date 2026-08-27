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

"""The actuator chain every replay applies: PD law, speed envelope, lag.

In the order the hardware applies it: PD demand -> clip to the robot's torque
limits -> speed envelope -> first-order lag. Everything here broadcasts over
however many joints the robot has; the limits and the measured envelopes are
the robot's data (:mod:`dimos.robot.unitree.go2.sim.plant`).
"""

from __future__ import annotations

from dataclasses import dataclass
import itertools

import numpy as np


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
    built-in Go2 controller commands up to 20.7 rad/s with kd up to 1.8: tens
    of Nm against a 41 Nm peak. Dropping it silently halves the damping term
    on exactly the recordings that carry the jumps.
    """
    out: np.ndarray = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
    return out


@dataclass(frozen=True)
class TorqueEnvelope:
    """What the real drive can deliver, as a function of joint speed.

    ``gain`` is a multiplicative derate ``g(|dq|)`` on the whole PD output;
    ``ceiling`` an absolute back-EMF / voltage-limit bound in N.m;
    ``brake_gain`` optionally replaces ``gain`` where the request OPPOSES the
    motion (``requested*dq < 0``), since back-EMF subtracts from drive voltage
    only when torque acts WITH rotation. Both curves are piecewise-linear on
    their own knot grids, non-increasing, flat past the last knot.
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
        """The torque the drive would actually produce for this request, N.m."""
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
    request to what the drive can source at that speed, BEFORE the lag, so an
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
