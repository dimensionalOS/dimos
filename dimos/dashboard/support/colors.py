# Copyright 2025 Dimensional Inc.
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

import numpy as np


def color_by_height(points: np.ndarray, colormap: str = "turbo_r") -> np.ndarray:
    heights = points[:, 2]

    hmin = heights.min()
    hmax = heights.max()
    denom = (hmax - hmin) if hmax > hmin else 1.0
    normalized_heights = (heights - hmin) / denom  # in [0, 1]

    try:
        import matplotlib

        cmap = matplotlib.colormaps[colormap]
        norm = matplotlib.colors.Normalize(vmin=0.0, vmax=1.0)
        return cmap(norm(normalized_heights))

    except ImportError:
        # ---- Simple fallback: blue -> cyan -> yellow -> red ----
        r = np.clip(4 * normalized_heights - 1.5, 0.0, 1.0)
        g = np.clip(4 * normalized_heights - 0.5, 0.0, 1.0)
        b = np.clip(1.5 - 4 * normalized_heights, 0.0, 1.0)

        rgb = np.stack([r, g, b], axis=1)
        alpha = np.ones((rgb.shape[0], 1), dtype=rgb.dtype)
        return np.concatenate([rgb, alpha], axis=1)


def color_by_distance(points: np.ndarray, colormap: str = "turbo_r") -> np.ndarray:
    point_distances = np.linalg.norm(points, axis=1)

    dmin = point_distances.min()
    dmax = point_distances.max()
    denom = (dmax - dmin) if dmax > dmin else 1.0
    normalized_distances = (point_distances - dmin) / denom  # in [0, 1]

    try:
        import matplotlib

        cmap = matplotlib.colormaps[colormap]
        norm = matplotlib.colors.Normalize(vmin=0.0, vmax=1.0)
        return cmap(norm(normalized_distances))

    except ImportError:
        # ---- Simple fallback: blue -> cyan -> yellow -> red ----
        # normalized_distances in [0, 1]
        r = np.clip(4 * normalized_distances - 1.5, 0.0, 1.0)
        g = np.clip(4 * normalized_distances - 0.5, 0.0, 1.0)
        b = np.clip(1.5 - 4 * normalized_distances, 0.0, 1.0)

        rgb = np.stack([r, g, b], axis=1)
        alpha = np.ones((rgb.shape[0], 1), dtype=rgb.dtype)
        return np.concatenate([rgb, alpha], axis=1)
