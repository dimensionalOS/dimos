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

from __future__ import annotations

import math
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray

ZBand: TypeAlias = tuple[float | None, float | None]
"""(low, high) absolute z, both ends included; None leaves that end unbounded."""


def check_z_band(band: ZBand) -> None:
    """Raise ValueError unless ``band`` is a (low, high) pair of finite numbers or None,
    low not above high."""
    if not isinstance(band, tuple) or len(band) != 2:
        raise ValueError("z must be a (low, high) pair; use None for an unbounded end")
    low, high = band
    for end in (low, high):
        if end is not None and (not isinstance(end, (int, float)) or not math.isfinite(end)):
            raise ValueError(f"z ends must be finite numbers or None, not {end!r}")
    if low is not None and high is not None and low > high:
        raise ValueError(f"z low {low:g} is above z high {high:g}")


def in_z_band(z: NDArray[np.float32] | NDArray[np.float64], band: ZBand) -> NDArray[np.bool_]:
    """Whether each height lies in ``band``."""
    low, high = band
    inside: NDArray[np.bool_] = (z >= (-np.inf if low is None else low)) & (
        z <= (np.inf if high is None else high)
    )
    return inside


def closed_z_band(band: ZBand, extent: tuple[float, float]) -> tuple[float, float]:
    """``band`` with each unbounded end replaced by that end of ``extent``."""
    low, high = band
    return (extent[0] if low is None else low, extent[1] if high is None else high)
