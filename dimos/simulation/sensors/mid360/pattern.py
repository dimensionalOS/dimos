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

"""Livox Mid-360 scan pattern.

Two rotors, a fast azimuth one and a slow elevation one, feeding four channels round-robin.
Each channel's direction is a 2D Fourier series in the two rotor phases.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

POINT_RATE = 200_000.0
CHANNELS = 4


class Mid360Pattern:
    def __init__(
        self,
        freq_fast: float,
        freq_slow: float,
        order_fast: int,
        order_slow: int,
        coefs: NDArray[np.float64],
    ) -> None:
        self.freq_fast = freq_fast
        self.freq_slow = freq_slow
        self.order_fast = order_fast
        self.order_slow = order_slow
        pairs = [
            (m, q)
            for m in range(order_fast + 1)
            for q in range(-order_slow, order_slow + 1)
            if not (m == 0 and q < 0)
        ]
        columns = sum(1 if pair == (0, 0) else 2 for pair in pairs)
        if columns != coefs.shape[1]:
            raise ValueError(f"pattern has {coefs.shape[1]} coefficients, orders imply {columns}")
        self._m_idx = np.array([m for m, _ in pairs])
        self._q_idx = np.array([q + order_slow for _, q in pairs])
        folded = np.zeros((len(pairs), CHANNELS * 3), dtype=np.complex128)
        col = 0
        for p, pair in enumerate(pairs):
            for c in range(CHANNELS):
                folded[p, 3 * c : 3 * c + 3] = coefs[c, col]
            col += 1
            if pair != (0, 0):
                for c in range(CHANNELS):
                    folded[p, 3 * c : 3 * c + 3] -= 1j * coefs[c, col]
                col += 1
        self._folded = folded

    @classmethod
    def load(cls, path: str | Path) -> Mid360Pattern:
        archive = np.load(path)
        return cls(
            freq_fast=float(archive["f1"]),
            freq_slow=float(archive["f2"]),
            order_fast=int(archive["m1"]),
            order_slow=int(archive["m2"]),
            coefs=np.asarray(archive["coefs"], dtype=np.float64),
        )

    def directions(self, k0: int, n: int) -> NDArray[np.float64]:
        """Unit sensor-frame directions of global point indices k0 .. k0 + n."""
        g0 = k0 // CHANNELS
        groups = np.arange(g0, (k0 + n - 1) // CHANNELS + 1)
        t = groups / (POINT_RATE / CHANNELS)
        fast = np.exp(2j * np.pi * self.freq_fast * t)
        slow = np.exp(2j * np.pi * self.freq_slow * t)
        ones = np.ones(len(groups))
        fast_pow = np.cumprod(np.column_stack([ones, *[fast] * self.order_fast]), axis=1)
        slow_pos = np.cumprod(np.column_stack([ones, *[slow] * self.order_slow]), axis=1)
        slow_pow = np.concatenate([np.conj(slow_pos[:, :0:-1]), slow_pos], axis=1)
        harmonic = fast_pow[:, self._m_idx] * slow_pow[:, self._q_idx]
        points = (harmonic @ self._folded).real.reshape(-1, 3)
        out: NDArray[np.float64] = points[k0 - g0 * CHANNELS :][:n]
        out /= np.linalg.norm(out, axis=1, keepdims=True)
        return out
