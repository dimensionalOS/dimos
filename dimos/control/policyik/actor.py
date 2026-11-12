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

"""Portable deterministic rl_games ELU actor; no training runtime or GPU required."""

from pathlib import Path
from typing import cast

import numpy as np
from numpy.typing import NDArray


class PolicyIKActor:
    """Load an exported 124-input, 18-output coordinated PolicyIK actor.

    The export retains float32 rl_games input normalization, epsilon 1e-5,
    normalized-observation clipping at five, and environment clipping at 100.
    The native player has clip_actions=False: there is no tanh or +/-1 clamp.
    """

    def __init__(self, path: Path) -> None:
        with np.load(path, allow_pickle=False) as arrays:
            self.mean = arrays["mean"].astype(np.float32)
            self.variance = arrays["variance"].astype(np.float32)
            self.weights = [arrays[f"weight{i}"].astype(np.float32) for i in range(3)]
            self.biases = [arrays[f"bias{i}"].astype(np.float32) for i in range(3)]
        shapes = [(256, 124), (256, 256), (18, 256)]
        if self.mean.shape != (124,) or self.variance.shape != (124,):
            raise ValueError("PolicyIK requires 124 normalization channels")
        for w, b, shape in zip(self.weights, self.biases, shapes, strict=True):
            if w.shape != shape or b.shape != (shape[0],):
                raise ValueError("Unexpected PolicyIK actor architecture")
        if not all(
            np.isfinite(x).all() for x in [self.mean, self.variance, *self.weights, *self.biases]
        ) or np.any(self.variance < 0):
            raise ValueError("Nonfinite weights or invalid normalization variance")
        self.scale = np.sqrt(self.variance + np.float32(1e-5))

    def __call__(self, observation: NDArray[np.floating]) -> NDArray[np.float32]:
        x = np.asarray(observation, dtype=np.float32)
        if x.shape[-1] != 124 or not np.isfinite(x).all():
            raise ValueError("Expected finite PolicyIK observation with 124 channels")
        x = np.clip((np.clip(x, -100, 100) - self.mean) / self.scale, -5, 5)
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases, strict=True)):
            x = x @ weight.T + bias
            if index < 2:
                x = np.maximum(x, 0) + np.expm1(np.minimum(x, 0))
        if not np.isfinite(x).all():
            raise ValueError("Nonfinite PolicyIK action")
        return cast("NDArray[np.float32]", np.clip(x, -100, 100).astype(np.float32))
