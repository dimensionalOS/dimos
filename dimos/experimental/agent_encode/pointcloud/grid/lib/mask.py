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

"""Three-state mask logic: 1, 0, or NaN for no data."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def is_mask(values: NDArray[np.float64]) -> bool:
    """Whether every value with data is 1 or 0."""
    finite = values[np.isfinite(values)]
    return bool(((finite == 0) | (finite == 1)).all())


def both(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """0 where either is 0, else NaN where either is, else 1."""
    missing = ~np.isfinite(a) | ~np.isfinite(b)
    out: NDArray[np.float64] = np.where((a == 0) | (b == 0), 0.0, np.where(missing, np.nan, 1.0))
    return out


def either(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """1 where either is 1, else NaN where either is, else 0."""
    missing = ~np.isfinite(a) | ~np.isfinite(b)
    out: NDArray[np.float64] = np.where((a == 1) | (b == 1), 1.0, np.where(missing, np.nan, 0.0))
    return out


def negate(a: NDArray[np.float64]) -> NDArray[np.float64]:
    """1 and 0 swap; NaN stays."""
    out: NDArray[np.float64] = np.where(np.isfinite(a), 1 - a, np.nan)
    return out
