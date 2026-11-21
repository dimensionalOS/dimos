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

"""Colour tables and the scales that read values back off an image's colours."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


def colour_table() -> tuple[str, NDArray[np.uint8]]:
    """The name and 256 x 3 uint8 rows of the colour table: matplotlib turbo, or a
    black-to-white ramp without matplotlib."""
    try:
        # matplotlib is an optional dependency
        from matplotlib import colormaps
    except ImportError:
        return "grey", np.repeat(np.arange(256, dtype=np.uint8)[:, None], 3, axis=1)
    table = colormaps["turbo"](np.linspace(0.0, 1.0, 256))[:, :3]
    return "matplotlib turbo", (table * 255).astype(np.uint8)


@dataclass(frozen=True)
class ColourStop:
    """One value and the colour that shows it."""

    value: float
    rgb: tuple[int, int, int]


@dataclass(frozen=True)
class ColourScale:
    """How to read a value off an image's colours."""

    table: str
    """The colour table: matplotlib turbo, or grey without matplotlib."""
    stops: tuple[ColourStop, ...]
    """Evenly spaced along the table, from the first colour to the last."""


def colour_scale(
    first: float, last: float, *, log: bool = False, reverse: bool = False, stops: int = 5
) -> ColourScale:
    """``stops`` colours from ``first`` to ``last``, spaced linearly or on a log scale;
    ``reverse`` runs the table from its end."""
    name, table = colour_table()
    out = []
    for i in range(stops):
        f = i / (stops - 1)
        value = first * (last / first) ** f if log else first + (last - first) * f
        row = table[round(255 * (1.0 - f if reverse else f))]
        out.append(ColourStop(round(value, 3), (int(row[0]), int(row[1]), int(row[2]))))
    return ColourScale(name, tuple(out))


def depth_range(depth: NDArray[np.float32], max_depth: float | None) -> tuple[float, float]:
    """The depth span the colours cover: nearest to farthest return, so the palette
    always spans what is in view. (0, max_depth or 1) when nothing was hit."""
    hit = depth[np.isfinite(depth)]
    if len(hit) == 0:
        return 0.0, max_depth if max_depth is not None else 1.0
    near, far = float(hit.min()), float(hit.max())
    if far - near < 0.5:
        far = near + 0.5
    return round(near, 2), round(far, 2)


def depth_fraction(depth: NDArray[np.float32], near: float, far: float) -> NDArray[np.float32]:
    """0 at ``near``, 1 at ``far``, on a log scale so that a metre of difference is
    visible up close and a far wall is still distinguishable from the floor in front
    of it."""
    near = max(near, 0.05)
    with np.errstate(divide="ignore", invalid="ignore"):
        f = np.log(np.maximum(depth, near) / near) / math.log(far / near)
    clipped: NDArray[np.float32] = np.clip(np.nan_to_num(f, nan=1.0, posinf=1.0), 0.0, 1.0)
    return clipped


def depth_rgb(depth: NDArray[np.float32], near: float, far: float) -> NDArray[np.uint8]:
    """Near is red through yellow and green to blue at ``far``, or bright to dark
    without matplotlib; no return is black."""
    hit = np.isfinite(depth)
    fraction = np.zeros(depth.shape, dtype=np.float32)
    fraction[hit] = depth_fraction(depth[hit], near, far)
    _, table = colour_table()
    rgb: NDArray[np.uint8] = table[(255 * (1.0 - fraction)).astype(np.uint8)]
    rgb[~hit] = 0
    return rgb
