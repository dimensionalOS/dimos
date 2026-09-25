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

"""How far apart neighbouring returns are."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree


def point_spacing(
    points: NDArray[np.float32] | NDArray[np.float64], min_m: float = 0.01, max_m: float = 0.5
) -> NDArray[np.float64]:
    """Distance from each return to its nearest neighbour, clamped to [``min_m``, ``max_m``]:
    the width to draw it at so that neighbours just touch. A lone return gets ``max_m``."""
    if len(points) < 2:
        return np.full(len(points), max_m, dtype=np.float64)
    d, _ = cKDTree(points).query(points, k=2)
    spacing: NDArray[np.float64] = np.clip(d[:, 1], min_m, max_m).astype(np.float64)
    return spacing
