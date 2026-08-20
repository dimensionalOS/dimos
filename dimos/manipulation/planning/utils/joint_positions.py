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

"""Utilities for complete native-model joint position vectors."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray


def repair_unconfigured_joint_positions(
    positions: NDArray[np.float64],
    lower_limits: NDArray[np.float64],
    upper_limits: NDArray[np.float64],
    configured_indices: list[int],
) -> NDArray[np.float64]:
    """Place invalid, unconfigured joints at the valid position nearest zero."""
    if lower_limits.shape != positions.shape or upper_limits.shape != positions.shape:
        raise ValueError("Joint positions and limit vectors do not match")

    configured = np.zeros(positions.shape, dtype=np.bool_)
    configured[configured_indices] = True
    invalid = ~((lower_limits <= positions) & (positions <= upper_limits))
    fallback = np.clip(np.zeros_like(positions), lower_limits, upper_limits)
    return np.where(~configured & invalid, fallback, positions)
