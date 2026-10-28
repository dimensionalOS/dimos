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

"""Experimental OpenYAM tracking policies, expressed in reported torque units.

The holding policy was measured on all six joints. The motion model was measured
only for J3 in the 0.16--0.40 rad interval and must be invoked explicitly by a
trajectory controller that can supply desired velocity and acceleration.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

_GAINS = np.array([25.0, 30.0, 30.0, 10.0, 8.0, 8.0])
_LIMITS = np.array([1.5, 3.0, 5.0, 1.0, 0.5, 0.5])
_RATES = np.array([0.5, 0.6, 0.8, 0.15, 0.1, 0.1])
_TORQUE_GUARD = np.array([10.0, 15.0, 18.0, 5.0, 3.0, 3.0])
_HOLD_ENTER = np.radians([0.08, 0.08, 0.15, 0.12, 0.08, 0.08])
_HOLD_EXIT = np.radians([0.12, 0.12, 0.20, 0.17, 0.12, 0.12])

# Controller coefficients from feedforward_validation5/model.json. These are
# local empirical corrections, not calibrated physical torque or friction.
J3_MOTION_COEFFICIENTS: dict[str, tuple[float, ...]] = {
    "left": (
        2.0436384801191947,
        -0.08692096854453558,
        2.235594407436067,
        -0.051256801067393755,
        0.23117549639258603,
    ),
    "right": (
        1.9797330617680966,
        -0.15643829758188602,
        2.575525518285589,
        -0.10379446282291521,
        0.23117549639258603,
    ),
}


class BoundedHoldingCorrection:
    """Rate-limited integral torque with a deadband and holding hysteresis."""

    def __init__(self) -> None:
        self.bias: NDArray[np.float64] = np.zeros(6)
        self.holding: NDArray[np.bool_] = np.zeros(6, dtype=bool)

    def reset(self) -> None:
        self.bias.fill(0.0)
        self.holding.fill(False)

    def update(
        self,
        error: NDArray[np.float64],
        velocity: NDArray[np.float64],
        target_velocity: NDArray[np.float64],
        base_torque: NDArray[np.float64],
        dt: float,
        *,
        eligible: bool,
    ) -> NDArray[np.float64]:
        values = (error, velocity, target_velocity, base_torque)
        if any(np.shape(value) != (6,) or not np.isfinite(value).all() for value in values):
            raise ValueError("holding correction requires finite six-joint vectors")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("holding correction requires a positive time step")

        moving = (np.abs(target_velocity) >= 0.002) | (np.abs(velocity) >= 0.03)
        if eligible and dt <= 0.03:
            self.holding = np.where(
                moving | (np.abs(error) >= _HOLD_EXIT),
                False,
                np.where(np.abs(error) <= _HOLD_ENTER, True, self.holding),
            )
        gate = eligible & (dt <= 0.03) & ~moving & ~self.holding & (np.abs(error) < 0.08)
        effective = np.where(np.abs(error) >= 0.0008, error, 0.0)
        delta = np.clip(_GAINS * effective, -_RATES, _RATES) * min(dt, 0.02)
        proposed = self.bias + np.where(gate, delta, 0.0)
        candidate = np.clip(proposed, -_LIMITS, _LIMITS)
        wanted = base_torque + candidate
        blocked = (np.abs(wanted) > 0.9 * _TORQUE_GUARD) & ((candidate - self.bias) * wanted > 0)
        self.bias = np.where(blocked, self.bias, candidate)
        return self.bias.copy()


class J3MotionCorrection:
    """The validated local J3 moving model, for an explicit trajectory caller."""

    def __init__(self, coefficients: tuple[float, ...]) -> None:
        self.coefficients = np.asarray(coefficients, dtype=float)
        if self.coefficients.shape != (5,) or not np.isfinite(self.coefficients).all():
            raise ValueError("J3 model requires five finite coefficients")
        self.active = False
        self.direction = 0
        self.delivered = 0.0
        self.trim = 0.0
        self.trim_memory = {-1: 0.0, 1: 0.0}
        self.last_speed = 0.0
        self.peak_speed = 0.0
        self.stopping = False

    def begin(self, holding_torque: float, direction: int) -> None:
        if direction not in (-1, 1) or not np.isfinite(holding_torque) or abs(holding_torque) > 5:
            raise ValueError("invalid J3 motion handoff")
        self.active = True
        self.direction = direction
        self.delivered = holding_torque
        self.trim = self.trim_memory[direction]
        self.last_speed = 0.0
        self.peak_speed = 0.0
        self.stopping = False

    def update(
        self,
        q: float,
        velocity: float,
        acceleration: float,
        error: float,
        base_torque: float,
        dt: float,
    ) -> float:
        if not self.active:
            raise RuntimeError("J3 motion correction has not begun")
        if not np.isfinite([q, velocity, acceleration, error, base_torque, dt]).all() or dt <= 0:
            raise ValueError("invalid J3 motion input")
        if not 0.16 <= q <= 0.40 or abs(velocity) > 0.04 or abs(acceleration) > 0.1:
            raise ValueError("J3 motion is outside the measured envelope")
        speed = abs(velocity)
        self.peak_speed = max(self.peak_speed, speed)
        if self.peak_speed > 0.004 and speed < self.last_speed - 1e-5:
            self.stopping = True
        self.last_speed = speed
        fade_speed = max(0.002, 0.2 * self.peak_speed)
        ratio = float(np.clip(speed / fade_speed, 0.0, 1.0))
        blend = ratio * ratio * (3.0 - 2.0 * ratio) if self.stopping else 1.0
        features = np.array(
            [1.0, (q - 0.28) / 0.1, self.direction * blend, velocity / 0.02, acceleration]
        )
        model = float(self.coefficients @ features)
        if (
            abs(velocity) > 0.002
            and np.radians(0.05) < abs(error) < 0.03
            and dt <= 0.03
            and abs(model + self.trim - self.delivered) < 0.25
        ):
            delta = float(np.clip(60.0 * error, -1.0, 1.0)) * min(dt, 0.02)
            candidate = float(np.clip(self.trim + delta, -1.0, 1.0))
            wanted = model + candidate
            if not (
                (abs(wanted) > 5 and delta * wanted > 0)
                or (abs(base_torque + wanted) > 16.2 and delta * (base_torque + wanted) > 0)
            ):
                self.trim = candidate
        desired = float(np.clip(model + self.trim, -5.0, 5.0))
        if abs(base_torque) < 21.2:
            desired = float(
                np.clip(desired, max(-5.0, -16.2 - base_torque), min(5.0, 16.2 - base_torque))
            )
        else:
            desired = float(-np.sign(base_torque) * 5.0)
        step = 8.0 * min(dt, 0.02)
        self.delivered = float(np.clip(desired, self.delivered - step, self.delivered + step))
        return self.delivered

    def end(self) -> float:
        if not self.active:
            raise RuntimeError("J3 motion correction has not begun")
        self.trim_memory[self.direction] = self.trim
        self.active = False
        return self.delivered
